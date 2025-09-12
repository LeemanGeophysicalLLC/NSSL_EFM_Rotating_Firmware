#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "ICM_20948.h"
#include "Adafruit_BME680.h"
#include "TinyGPS++.h"
#include "pins.h"
#include "IWatchdog.h"
#include "ADS131M04.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"
#include <math.h>   // for lroundf
#include <limits.h> // for INT16_MIN

// Uncomment to enable debug prints
//#define DEBUG_PRINT_ENABLE
#define WAIT_ON_GPS_FIX // only disabled for testing purposes
//#define WATCHDOG_ENABLE

#define DEBUG_BAUD 115200
#define GPS_BAUD        115200


// ==================== Debug Macros ====================
#ifdef DEBUG_PRINT_ENABLE
  #define debugPrint(x)    SerDebug.print(x)
  #define debugPrintln(x)  SerDebug.println(x)
#else
  #define debugPrint(x)
  #define debugPrintln(x)
#endif  // DEBUG_PRINT_ENABLE

// ==================== Globals & Interfaces ====================
HardwareSerial    SerDebug(PIN_RADIO_TX, PIN_RADIO_RX);
HardwareSerial    SerGPS(PIN_GPS_TX, PIN_GPS_RX);
TinyGPSPlus       gps;
Adafruit_BME680   bme;
ICM_20948_I2C     imu;
ADS131M04 adc;
SFE_UBLOX_GNSS myGNSS;

volatile bool flag_read_hf = false;
volatile bool flag_read_lf = false;
volatile uint32_t adc_ms = 0;
volatile uint32_t adc_pps_offset = 0;


// Buffer template
template<typename T, size_t N>
struct CircularBuffer {
  T      buf[N];
  size_t head = 0, tail = 0;

  bool isEmpty() const { 
    return head == tail; 
  }
  bool isFull()  const { 
    return ((head + 1) % N) == tail; 
  }

  void push(const T &v) {
    buf[head] = v;
    head = (head + 1) % N;
    if (head == tail)            // buffer overflow → drop oldest
      tail = (tail + 1) % N;
  }

  bool pop(T &out) {
    if (isEmpty()) return false; // nothing to retrieve
    out = buf[tail];
    tail = (tail + 1) % N;
    return true;
  }

  size_t size() const {
    // number of elements stored
    return (head + N - tail) % N;
  }
};


// ==================== Record Types ====================
enum RecordType : uint8_t { REC_HF=1, REC_LF=2 };

struct HFRecord {
  uint32_t pps_utc;
  uint32_t ms_offset;
  uint32_t processor_ms;
  int32_t adc[4];
  float    imu[9];
};

struct LFRecord {
  uint32_t pps_utc;
  uint32_t ms_offset;
  uint32_t processor_ms;
  int32_t  lat;
  int32_t  lon;
  uint16_t alt;
  int16_t  temp;
  int16_t  pressure;
  int16_t  humidity;
};

struct LogEntry {
  RecordType type;
  union {
    HFRecord hf;
    LFRecord lf;
  } data;
};


// ======================= Runtime State ======================= //
/**
 * @brief Struct to hold runtime flags and timestamps for system state management.
 */
struct RuntimeStatus {
  volatile uint32_t ppsMillis = 0;              ///< Timestamp of last PPS signal
  volatile bool ppsSeen = false;                ///< PPS pulse seen flag
  uint32_t gpsEpochMillis = 0;                  ///< Millis at last GPS PPS sync
  uint32_t lastSample = 0;                      ///< Timestamp of last sensor sample
  uint32_t lastUTC = 0;                         ///< Last UTC value from GPS
  uint32_t gpsLastValidMillis = 0;              ///< Last millis() when GPS time was valid
  bool gpsLocked = false;                       ///< GPS has valid time and PPS sync
  bool gpsError = false;                        ///< GPS timeout occurred
  bool logError = false;                        ///< SD card write or open failure
  uint32_t utc_hhmmss00 = 0;   // hhmmss00 (centiseconds=00)
  bool     utcValid = false;   // have we ever parsed a valid NMEA time?


  /**
   * @brief High-level system state.
   */
  enum SystemState {
    STATE_ACQUIRING_GPS, ///< Waiting for valid GPS
    STATE_LOGGING,       ///< Normal logging operation
    STATE_ERROR          ///< Error state due to sensor, GPS, or SD failure
  } currentState = STATE_ACQUIRING_GPS;
};
RuntimeStatus runtime;

// Buffer for up to 256 entries
static CircularBuffer<LogEntry, 128> logBuf;

static inline uint32_t bump_hhmmss00(uint32_t t) {
  uint8_t hh = (t / 1000000UL) % 100;
  uint8_t mm = (t / 10000UL)   % 100;
  uint8_t ss = (t / 100UL)     % 100;  // cc stays 00 for us
  ss++;
  if (ss == 60) { ss = 0; mm++; if (mm == 60) { mm = 0; hh = (hh + 1) % 24; } }
  return (uint32_t)hh * 1000000UL + (uint32_t)mm * 10000UL + (uint32_t)ss * 100UL;
}

// ------------------------------------------------------
// Low-frequency sampling (1 Hz): GPS + BME → LFRecord
// ------------------------------------------------------
void logLowFreqFields() {
  debugPrintln("logLowFreqFields: Starting LF record logging");
  // 1) Build a fresh LFRecord
  LFRecord rec{};
  rec.pps_utc = runtime.utc_hhmmss00;
  rec.ms_offset = millis() - runtime.ppsMillis;
  rec.processor_ms = millis();

  // 2) GPS position & altitude (TinyGPS++)
  debugPrintln("logLowFreqFields: Getting GPS position and altitude");
  double lat_d = gps.location.lat();
  double lon_d = gps.location.lng();
  rec.lat      = int32_t(lat_d * 1e5);              // ×1e5 fixed-point
  rec.lon      = int32_t(lon_d * 1e5);
  rec.alt      = uint16_t(gps.altitude.meters() + 0.5);  // metres

  // 3) BME280 readings (Adafruit_BME280)
  debugPrintln("logLowFreqFields: Getting BME readings");

  if (bme.performReading()) {
    // Temperature (°C) ×10
    rec.temp = (int16_t)lroundf(bme.temperature * 10.0f);

    // Pressure is in Pa -> convert to hPa, then ×10
    float p_hPa = bme.pressure / 100.0f;
    rec.pressure = (int16_t)lroundf(p_hPa * 10.0f);

    // Humidity (%RH) ×10
    rec.humidity = (int16_t)lroundf(bme.humidity * 10.0f);
  } else {
    // Reading failed: mark clearly (you can also keep prior values if you prefer)
    rec.temp     = INT16_MIN;
    rec.pressure = INT16_MIN;
    rec.humidity = INT16_MIN;
    debugPrintln("BME680: performReading() failed");
  }

  debugPrint("Temperature: ");
  debugPrint(rec.temp / 10.0); // Print temperature in °C
  debugPrint(" degC\nPressure: ");
  debugPrint(rec.pressure / 10.0); // Print pressure in hPa
  debugPrint(" hPa\nHumidity: ");
  debugPrint(rec.humidity / 10.0); // Print humidity in %
  debugPrintln(" %");
  debugPrint("GPS Lat: ");
  debugPrintln(lat_d); // Print latitude with 6 decimal places
  debugPrint(" Lon: ");
  debugPrintln(lon_d); // Print longitude with 6 decimal places

  // 4) Push into the circular buffer
  LogEntry entry;
  entry.type    = REC_LF;
  entry.data.lf = rec;
  logBuf.push(entry);
  debugPrintln("logLowFreqFields: LF record queued");
}


/**
 * @brief Set RGB LED state using PWM values.
 *
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(PIN_LED_RED, r);
  analogWrite(PIN_LED_GREEN, g);
  analogWrite(PIN_LED_BLUE, b);
}

/**
 * @brief Update LED color to reflect current system state.
 */
void updateLED() {
  switch (runtime.currentState) {
    case RuntimeStatus::STATE_ACQUIRING_GPS: setLED(0, 0, 255); break;
    case RuntimeStatus::STATE_LOGGING:       setLED(0, 255, 0); break;
    case RuntimeStatus::STATE_ERROR:         setLED(255, 0, 0); break;
    default:                                 setLED(0, 0, 0); break;
  }
}

// ------------------------------------------------------
// Cycle RGB LEDs on power-up to show we’re alive
// ------------------------------------------------------
void startupLedSequence() {
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE,  OUTPUT);

  setLED(255, 0, 0);
  delay(1000);
  setLED(0, 255, 0);
  delay(1000);
  setLED(0, 0, 255);
  delay(1000);
  setLED(0, 0, 0);
  delay(1000);

  // Now let updateLED() drive the LED according to state
  updateLED();
}

/**
 * @brief Checks if the GPS has a valid fix and accurate time.
 * 
 * This function verifies that:
 * - The GPS time is valid (gps.time.isValid() is true)
 * - The GPS location is valid (gps.location.isValid() is true)
 * 
 * If both conditions are met, the GPS is considered "locked" and time is reliable.
 * This avoids trusting GPS-estimated time from cold boot or almanac restore.
 * 
 * @return true if GPS has a valid fix and time; false otherwise.
 */
bool isGPSLocked() {
  const unsigned long maxAge = 2000; // milliseconds
  return gps.time.isValid() &&
         gps.location.isValid() &&
         gps.time.age() < maxAge &&
         gps.location.age() < maxAge;
}

/**
 * @brief Poll a limited number of GPS characters and sync time if valid.
 */
void pollGPS() {
  while (SerGPS.available()) {
    char c = SerGPS.read();
    //SerDebug.write(c); // Echo to debug console
    if (gps.encode(c)) {
      // Always keep lastUTC fresh when valid
      if (gps.time.isValid()) {
        uint32_t nmea = gps.time.value();       // hhmmsscc
        // snap our counter to NMEA, forcing cc to 00
        runtime.utc_hhmmss00 = (nmea / 100U) * 100U;
        runtime.utcValid = true;
        runtime.gpsLastValidMillis = millis();
      }

      // Align millis to the top-of-second when we see PPS
      if (isGPSLocked() && runtime.ppsSeen) {
        runtime.gpsEpochMillis = runtime.ppsMillis; // millisecond 0 of current second
        runtime.gpsLocked      = true;
        runtime.ppsSeen        = false;
      }
    }
  }
  if (millis() - runtime.gpsLastValidMillis > 3000) runtime.gpsLocked = false;
}


/**
 * @brief GPS PPS interrupt handler.
 */
void ppsISR() {
  runtime.ppsMillis = millis();
  runtime.ppsSeen   = true;
  if (runtime.gpsLocked && runtime.utcValid) {
    runtime.utc_hhmmss00 = bump_hhmmss00(runtime.utc_hhmmss00);
  }
  flag_read_lf = true;
}


// ------------------------------------------------------
// ADC DRDY ISR: sample ADS131M04 at 250 Hz, keep 1/5 → 50 Hz
// ------------------------------------------------------
void adcISR() {
  static uint8_t ready_cnt = 0;
  if (++ready_cnt >= 5) {     // pre-increment → 1-in-5
    ready_cnt = 0;
    flag_read_hf = true;
    adc_ms = millis();
    //adc_pps_offset = adc_ms - runtime.ppsMillis;  // ms since last PPS
    adc_pps_offset = (adc_ms - runtime.ppsMillis + 1000) % 1000;  // 0..999

  }
}

// Scales for IMU serialization (units → scaled ints)
static inline int16_t acc_to_mg(float g)      { return (int16_t)lroundf(g * 1.0f); } // ±16g -> ±16000
static inline int16_t gyr_to_dps10(float dps) { return (int16_t)lroundf(dps * 10.0f); } // ±2000 dps -> ±20000
static inline int16_t mag_to_uT(float uT)     { return (int16_t)lroundf(uT * 1.0f); }   // typical ±4900 uT


bool initADC()
{
  digitalWrite(PIN_ADC_RESET, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_ADC_RESET, HIGH);
  delay(1);
  adc.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ADC_CS, PIN_ADC_DRDY);
  delay(10);

  adc.setInputChannelSelection(0, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(1, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(2, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setInputChannelSelection(3, INPUT_CHANNEL_MUX_AIN0P_AIN0N);
  adc.setChannelEnable(0, 1);
  adc.setChannelEnable(1, 1);
  adc.setChannelEnable(2, 1);
  adc.setChannelEnable(3, 1);
  adc.setChannelPGA(0, CHANNEL_PGA_1);
  adc.setChannelPGA(1, CHANNEL_PGA_1);
  adc.setChannelPGA(2, CHANNEL_PGA_1);
  adc.setChannelPGA(3, CHANNEL_PGA_1);
  adc.setOsr(OSR_16384);
  return true; // TODO any checks?
}

bool initIMU()
{
  imu.begin(Wire, 1); // 1 for AD0 high
  if (imu.status != ICM_20948_Stat_Ok)
  {
    return false;
  }
  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (imu.status != ICM_20948_Stat_Ok)
  {
    return false;
  }

  return true;
}

bool initBME()
{
  if (bme.begin(0x76)) {
    // Set up oversampling and filter initialization
    bme.setGasHeater(0, 0); 
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.beginReading();

    debugPrintln("BME680: Initialized successfully");
  } else {
    debugPrintln("BME680: Initialization failed");
    return false;
  }
  return true;
}

bool initGPS() {

  if (!myGNSS.begin()) {
    debugPrintln("GPS not detected. Check I2C wiring.");
    return false;
  }

  debugPrintln("Configuring GPS...");
  myGNSS.factoryReset();
  myGNSS.factoryDefault();
  delay(2000);

  // Set dynamic model to AIRBORNE <4g>
  if (!myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g)) {
    debugPrintln("Failed to set dynamic model");
    return false;
  }

  // Set UART1 baud rate to 115200
  myGNSS.setSerialRate(115200, COM_PORT_UART1); // No return value
  debugPrintln("Set UART1 baud rate to 115200.");
  delay(100);
  SerGPS.begin(GPS_BAUD);
  delay(100);

  // Set navigation rate to 1 Hz
  if (!myGNSS.setNavigationFrequency(1))
  {
    debugPrintln("Failed to set navigation frequency");
    return false;
  }

  // Disable all common NMEA messages
  myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_UART1);
  myGNSS.disableNMEAMessage(UBX_NMEA_GNS, COM_PORT_UART1);

  // Enable RMC and ZDA only
  myGNSS.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);
  myGNSS.enableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);
  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);

  UBX_CFG_TP5_data_t timePulseSettings;
  memset(&timePulseSettings, 0, sizeof(UBX_CFG_TP5_data_t)); // Clear struct

  timePulseSettings.tpIdx = 0;                   // TIMEPULSE pin 0
  timePulseSettings.version = 0x01;
  timePulseSettings.flags.bits.active = 1;       // Enable output
  timePulseSettings.flags.bits.lockedOtherSet = 1; // Align to top of second
  timePulseSettings.flags.bits.isFreq = 1;       // Frequency mode
  timePulseSettings.flags.bits.isLength = 1;     // Length is valid
  //timePulseSettings.flags.bits.pulseDef = 0;     // Pulse is 'time high'
  timePulseSettings.freqPeriod = 1;              // 1 Hz
  timePulseSettings.freqPeriodLock = 1;
  timePulseSettings.pulseLenRatio = 100000;      // 100ms = 100,000 ns
  timePulseSettings.pulseLenRatioLock = 100000;

  if (!myGNSS.setTimePulseParameters(&timePulseSettings))
  {
    debugPrintln("Failed to configure time pulse!");
  }
  else
  {
    debugPrintln("Time pulse configured.");
  }

  // Save settings
  if (!myGNSS.saveConfiguration())
  {
    debugPrintln("Failed to save configuration!");
    return false;
  }
  else
  {
    debugPrintln("GPS configuration saved.");
  }

  debugPrintln("Setup complete. GPS will output RMC/ZDA/GGA at 115200 baud over UART1.");
  return true;
}


// ------------------------------------------------------
// Top-level Arduino setup()
// ------------------------------------------------------
void setup() {

  SerDebug.begin(DEBUG_BAUD);
  delay(500);
  #ifdef WATCHDOG_ENABLE
    debugPrintln("Starting watchdog...");
    IWatchdog.begin(26000000);    // max ~26 208 000 µs  
    debugPrintln("Watchdog running");
  #endif

  
  debugPrintln("Configuring pins...");
  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_BLUE,   OUTPUT);
  pinMode(PIN_SD_CS,      OUTPUT);
  pinMode(PIN_SD_CD,      INPUT);
  pinMode(PIN_ADC_CS,     OUTPUT);
  pinMode(PIN_ADC_DRDY,   INPUT);
  pinMode(PIN_GPS_PPS,    INPUT);
  pinMode(PIN_ADC_RESET, OUTPUT);

  digitalWrite(PIN_ADC_CS, HIGH); // ensure ADC CS is high
  digitalWrite(PIN_SD_CS, HIGH);   // ensure SD CS is high

  delay(50);
  debugPrintln("Initializing SPI...");
  SPI.begin();
  delay(50);
  debugPrintln("Initializing I2C...");
  Wire.begin();
  Wire.setClock(400000);
  delay(300);

  bool ok = true;
  
  debugPrint("Initializing ADC... ");
  if (!initADC()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");

  delay(10);
  
  debugPrint("Initializing IMU... ");
  if (!initIMU()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  delay(10);

  debugPrint("Initializing BME688... ");
  if (!initBME()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  delay(10);

  debugPrint("Initializing GPS... ");
   if (!initGPS()) {
      debugPrintln("failed");
      ok = false;
  } else debugPrintln("ok");

  delay(10);
  
  startupLedSequence();
  if (!ok) {
    debugPrintln("Setup failed — entering ERROR state");
    // show red LED
    runtime.currentState = RuntimeStatus::STATE_ERROR;
    updateLED();
    while (1) {setLED(255, 0, 0); delay(100); setLED(0, 0, 0); delay(100);} // blink red
  }

  #ifdef WAIT_ON_GPS_FIX
  debugPrintln("Waiting for GPS lock...");
  runtime.currentState = RuntimeStatus::STATE_ACQUIRING_GPS;
  updateLED();
  while (!isGPSLocked()) {
    pollGPS();
    SerDebug.println("Waiting for valid GPS fix and time...");
    delay(100);
    #ifdef WATCHDOG_ENABLE
      IWatchdog.reload();
    #endif
  }
  #endif

  debugPrintln("Setting up GPS PPS Interrupt...");
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ppsISR, RISING);

  delay(10);

  debugPrintln("Setting up ADC Interrupt...");
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY), adcISR, FALLING);

  runtime.currentState = RuntimeStatus::STATE_LOGGING;
  updateLED();
  debugPrintln("GPS lock acquired... setup complete");
}

// Format one entry into a line buffer; returns number of bytes written.
static int formatLine(const LogEntry &e, char *out, size_t cap) {
  if (e.type == REC_HF) {
    const HFRecord &r = e.data.hf;
    // Convert IMU floats to compact ints for lightweight printing
    int16_t ax = acc_to_mg(r.imu[0]), ay = acc_to_mg(r.imu[1]), az = acc_to_mg(r.imu[2]);
    int16_t gx = gyr_to_dps10(r.imu[3]), gy = gyr_to_dps10(r.imu[4]), gz = gyr_to_dps10(r.imu[5]);
    int16_t mx = mag_to_uT(r.imu[6]), my = mag_to_uT(r.imu[7]), mz = mag_to_uT(r.imu[8]);

    // CSV: 1,ppsUTC,msOff,ms, ch0..ch3, ax,ay,az, gx,gy,gz, mx,my,mz
    return snprintf(out, cap,
      "1,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
      (unsigned long)r.pps_utc,
      (unsigned long)r.ms_offset,
      (unsigned long)r.processor_ms,
      (long)r.adc[0], (long)r.adc[1], (long)r.adc[2], (long)r.adc[3],
      ax, ay, az, gx, gy, gz, mx, my, mz
    );
  } else {
    const LFRecord &r = e.data.lf;
    // CSV: 2,ppsUTC,msOff,ms, latE5,lonE5, altM, temp_x10, pres_hPa_x10, hum_x10
    return snprintf(out, cap,
      "2,%lu,%lu,%lu,%ld,%ld,%u,%d,%d,%d\n",
      (unsigned long)r.pps_utc,
      (unsigned long)r.ms_offset,
      (unsigned long)r.processor_ms,
      (long)r.lat, (long)r.lon, (unsigned)r.alt, (int)r.temp, (int)r.pressure, (int)r.humidity
    );
  }
}

// Drain a few queued entries to Serial without blocking long.
// Writes as much as Serial’s TX buffer can accept.
static void drainSerial() {
  static char  line[196];
  static int   pendingLen = 0;
  static int   pendingOff = 0;
  static bool  hasPending = false;
  const int    MAX_PER_CALL = 8;

  int sent = 0;
  LogEntry e;

  auto push_pending = [&](const LogEntry& le){
    pendingLen = formatLine(le, line, sizeof(line));
    pendingOff = 0;
    hasPending = (pendingLen > 0);
  };

  while (sent < MAX_PER_CALL) {
    // If nothing pending, pop a new entry
    if (!hasPending) {
      if (logBuf.isEmpty()) break;
      if (!logBuf.pop(e)) break;
      push_pending(e);
      if (!hasPending) continue; // skip if format failed
    }

    // Try to write as much of the pending line as fits
    int room = SerDebug.availableForWrite();   // may be small (even 1–2 bytes)
    if (room <= 0) break;
    int chunk = pendingLen - pendingOff;
    if (chunk > room) chunk = room;

    SerDebug.write((uint8_t*)line + pendingOff, chunk);
    pendingOff += chunk;

    // Finished the line?
    if (pendingOff >= pendingLen) {
      hasPending = false;
      ++sent;
    }
  }
}


void readHFSensors()
{
  adcOutput tmp = adc.readADC();  // one SPI read per decimated sample

  HFRecord rec{};
  rec.pps_utc = runtime.utc_hhmmss00;
  rec.ms_offset    = adc_pps_offset;              // *** exact 20 ms grid ***
  rec.processor_ms = adc_ms;

  rec.adc[0] = tmp.ch0;
  rec.adc[1] = tmp.ch1;
  rec.adc[2] = tmp.ch2;
  rec.adc[3] = tmp.ch3;

  if (imu.status == ICM_20948_Stat_Ok) {
    imu.getAGMT();
    rec.imu[0] = imu.accX(); rec.imu[1] = imu.accY(); rec.imu[2] = imu.accZ();
    rec.imu[3] = imu.gyrX(); rec.imu[4] = imu.gyrY(); rec.imu[5] = imu.gyrZ();
    rec.imu[6] = imu.magX(); rec.imu[7] = imu.magY(); rec.imu[8] = imu.magZ();
  } else {
    for (int i=0;i<9;i++) rec.imu[i] = 0.0f;
  }

  LogEntry e; e.type = REC_HF; e.data.hf = rec;
  logBuf.push(e);
}

void readLFSensors()
{
  // 1) Build a fresh LFRecord
  LFRecord rec{};
  rec.pps_utc = runtime.utc_hhmmss00;
  rec.ms_offset = millis() - runtime.ppsMillis;
  rec.processor_ms = millis();

  // 2) GPS position & altitude (TinyGPS++)
  debugPrintln("logLowFreqFields: Getting GPS position and altitude");
  double lat_d = gps.location.lat();
  double lon_d = gps.location.lng();
  rec.lat      = int32_t(lat_d * 1e5);              // ×1e5 fixed-point
  rec.lon      = int32_t(lon_d * 1e5);
  rec.alt      = uint16_t(gps.altitude.meters() + 0.5);  // metres

  // 3) BME280 readings (Adafruit_BME280)
  debugPrintln("logLowFreqFields: Getting BME readings");
  
  if (bme.endReading()) {
    // Temperature (°C) ×10
    rec.temp = (int16_t)lroundf(bme.temperature * 10.0f);

    // Pressure is in Pa -> convert to hPa, then ×10
    float p_hPa = bme.pressure / 100.0f;
    rec.pressure = (int16_t)lroundf(p_hPa * 10.0f);

    // Humidity (%RH) ×10
    rec.humidity = (int16_t)lroundf(bme.humidity * 10.0f);
  } else {
    // Reading failed: mark clearly (you can also keep prior values if you prefer)
    rec.temp     = INT16_MIN;
    rec.pressure = INT16_MIN;
    rec.humidity = INT16_MIN;
    debugPrintln("BME680: performReading() failed");
  }
  bme.beginReading(); // start next reading

  debugPrint("Temperature: ");
  debugPrint(rec.temp / 10.0); // Print temperature in °C
  debugPrint(" degC\nPressure: ");
  debugPrint(rec.pressure / 10.0); // Print pressure in hPa
  debugPrint(" hPa\nHumidity: ");
  debugPrint(rec.humidity / 10.0); // Print humidity in %
  debugPrintln(" %");
  debugPrint("GPS Lat: ");
  debugPrintln(lat_d); // Print latitude with 6 decimal places
  debugPrint(" Lon: ");
  debugPrintln(lon_d); // Print longitude with 6 decimal places

  // 4) Push into the circular buffer
  LogEntry entry;
  entry.type    = REC_LF;
  entry.data.lf = rec;
  logBuf.push(entry);
  debugPrintln("logLowFreqFields: LF record queued");
}


void loop() {

  if (flag_read_hf)
  {
    //SerDebug.println("HF read");
    flag_read_hf = false;
    readHFSensors();
  }

  if (flag_read_lf)
  {
    //SerDebug.println("LF read");
    flag_read_lf = false;
    readLFSensors();
  }

  pollGPS();

  drainSerial();

  updateLED();

  #ifdef WATCHDOG_ENABLE
    IWatchdog.reload();
  #endif
}
