#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include "ICM_20948.h"
#include "Adafruit_BME680.h"
#include "TinyGPS++.h"
#include "pins.h"
#include "IWatchdog.h"
#include "ADS131M04.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

// Uncomment to enable debug prints
//#define DEBUG_PRINT_ENABLE
#define WAIT_ON_GPS_FIX // only disabled for testing purposes
//#define WATCHDOG_ENABLE

#define DEBUG_BAUD 115200
#define GPS_BAUD        115200
const uint32_t SPI_SPEED = SD_SCK_MHZ(18);


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
SdFat             sd;
TinyGPSPlus       gps;
Adafruit_BME680   bme;
ICM_20948_I2C     imu;
File              logFile;
ADS131M04 adc;
SFE_UBLOX_GNSS myGNSS;

volatile uint8_t adcSampleCnt = 0;

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
  uint32_t adc[4];
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

// Raw ADC storage
static uint32_t rawAdc[4];

// Timing
static uint32_t lastLfLogMs = 0;

// ------------------------------------------------------
// Low-frequency sampling (1 Hz): GPS + BME → LFRecord
// ------------------------------------------------------
void logLowFreqFields() {
  debugPrintln("logLowFreqFields: Starting LF record logging");
  // 1) Build a fresh LFRecord
  LFRecord rec{};
  rec.pps_utc   = runtime.lastUTC;
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

  // TODO - Uncomment and use BME readings
  bme.performReading();
  rec.temp     = int16_t(bme.temperature * 10);            // ×10 fixed-point
  float p_hPa = bme.pressure / 100.0f;
  rec.pressure = int16_t(p_hPa * 10);
  rec.humidity = int16_t(bme.humidity * 10);

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
  const int maxChars = 100;
  int count = 0;
  while (SerGPS.available() && count < maxChars) {
    char c = SerGPS.read();
    debugPrint(c); // Print GPS character for debugging
    if (gps.encode(c)) {
      if (isGPSLocked() && runtime.ppsSeen) {
        runtime.lastUTC = gps.time.value();
        runtime.gpsEpochMillis = runtime.ppsMillis;
        runtime.gpsLocked = true;
        runtime.gpsLastValidMillis = millis();
        runtime.ppsSeen = false;
        debugPrintln("GPS synced to PPS");
      }
    }
    count++;
  }
// Reset lock if location or time becomes stale
if (millis() - runtime.gpsLastValidMillis > 3000) {
  runtime.gpsLocked = false;
}

}

/**
 * @brief GPS PPS interrupt handler.
 */
void ppsISR() {
  runtime.ppsMillis = millis();
  runtime.ppsSeen = true;
}

// ------------------------------------------------------
// ADC DRDY ISR: sample ADS131M04 at 250 Hz, keep 1/5 → 50 Hz
// ------------------------------------------------------

void adcISR() {
  // 1) Read all 4 channels into a local
  adcOutput tmp = adc.readADC();    // no volatile here
  rawAdc[0] = tmp.ch0;
  rawAdc[1] = tmp.ch1;
  rawAdc[2] = tmp.ch2;
  rawAdc[3] = tmp.ch3;

  // 2) Only keep every 5th sample → 50 Hz
  if (++adcSampleCnt < 5) {
    return;
  }
  adcSampleCnt = 0;

  // 3) Build HFRecord
  HFRecord rec{};
  rec.pps_utc      = runtime.lastUTC;
  rec.ms_offset    = (millis() - runtime.ppsMillis);
  rec.processor_ms = millis();

  // 4) Copy ADC data
  for (int i = 0; i < 4; i++) {
    rec.adc[i] = rawAdc[i];
  }

  // 5) Grab IMU (or zero on error)
  if (imu.status == ICM_20948_Stat_Ok) {
    imu.getAGMT();
    rec.imu[0] = imu.accX(); rec.imu[1] = imu.accY(); rec.imu[2] = imu.accZ();
    rec.imu[3] = imu.gyrX(); rec.imu[4] = imu.gyrY(); rec.imu[5] = imu.gyrZ();
    rec.imu[6] = imu.magX(); rec.imu[7] = imu.magY(); rec.imu[8] = imu.magZ();
  } else {
    for (int i = 0; i < 9; i++) rec.imu[i] = 0.0f;
  }

  // 6) Push into circular buffer
  LogEntry entry;
  entry.type     = REC_HF;
  entry.data.hf  = rec;
  logBuf.push(entry);
}


bool initADC()
{
  digitalWrite(PIN_ADC_RESET, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_ADC_RESET, HIGH);
  delay(1);
  adc.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ADC_CS, PIN_ADC_DRDY);
  delay(10);
  // Try to read the ID register (address 0x00) // TODO Remove
  uint8_t id = adc.readRegister(0x00);
  debugPrint("ADS131M04 ID = ");
  debugPrintln(id);

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
  SerGPS.begin(GPS_BAUD);


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

bool initSD()
{
  if (!sd.begin(PIN_SD_CS, SPI_SPEED))
  {
    return false;
  }
  return true;
}

bool testSDwrite() {
  debugPrintln("testSDwrite: Starting SD init test");

  // 1) Remove old file if it exists
  if (sd.exists("testinit.txt")) {
    if (sd.remove("testinit.txt")) {
      debugPrintln("testSDwrite: Old testinit.txt removed");
    } else {
      debugPrintln("testSDwrite: Failed to remove old testinit.txt");
      return false;
    }
  }

  // 2) Open for write (creates new/truncates existing)
  File writeFile = sd.open("testinit.txt", FILE_WRITE);
  if (!writeFile) {
    debugPrintln("testSDwrite: Failed to open testinit.txt for write");
    return false;
  }

  // 3) Write test string
  writeFile.println("SD Init Test");
  writeFile.close();
  debugPrintln("testSDwrite: Wrote \"SD Init Test\"");

  // 4) Re-open for read
  File readFile = sd.open("testinit.txt", FILE_READ);
  if (!readFile) {
    debugPrintln("testSDwrite: Failed to open testinit.txt for read");
    return false;
  }

  // 5) Read back and print over debug
  debugPrint("testSDwrite: Read back: ");
  while (readFile.available()) {
    char c = (char)readFile.read();
    debugPrint(c);
  }
  debugPrintln("");  
  readFile.close();
  debugPrintln("testSDwrite: SD read/write test complete");

  return true;
}

// ------------------------------------------------------
// Check the “tens” digit of the minute and roll over the log file
// ------------------------------------------------------
void rolloverFileIfNeeded() {
  static int8_t lastTensMinute = -1;

  // Figure out which 10-minute slot we’re in
  int currentTens = runtime.gpsLocked
    ? gps.time.minute() / 10
    : 0;

  // Nothing to do until that slot changes
  if (currentTens == lastTensMinute) return;
  lastTensMinute = currentTens;

  // Build the filename
  char fname[13];  // "MMDDHHMM.TXT\0"
  if (runtime.gpsLocked) {
    snprintf(fname, sizeof(fname),
             "%02d%02d%02d%02d.TXT",
             gps.date.month(),
             gps.date.day(),
             gps.time.hour(),
             currentTens * 10);
  } else {
    strcpy(fname, "00000000.TXT");
  }

  debugPrint("rollover: opening "); debugPrintln(fname);

  // Temporarily pause ADC ISR so it can't steal SPI while we touch the card
  detachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY));

  // 1) Close the old log file (if any)
  if (logFile) {
    debugPrintln("rollover: closing old log file");
    logFile.close();
  }

  // 2) Delete any existing file with this name
  if (sd.exists(fname)) {
    debugPrintln("rollover: removing existing file");
    sd.remove(fname);
  }

  // 3) Open a fresh file for this 10-minute block
  logFile = sd.open(fname, O_CREAT | O_TRUNC | O_WRITE);

  // Re-enable ADC ISR
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY), adcISR, FALLING);

  if (!logFile) {
    debugPrintln("rollover: ERROR opening log file!");
    runtime.currentState = RuntimeStatus::STATE_ERROR;
    updateLED();
    return;
  }

  debugPrintln("rollover: new log file ready");
}



// ------------------------------------------------------
// Flush pending log entries to the SD card
// ------------------------------------------------------
void flushLog() {
  if (logBuf.size() < 100) {
    debugPrintln("flushLog: Not enough entries to flush");
    return;
  }
  LogEntry entry;

  // 1) Stop ADC interrupts so the ISR can't steal SPI during any of our writes
  detachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY));

  // 2) Write out everything in the buffer
  while (logBuf.pop(entry)) {
    if (entry.type == REC_HF) {
      HFRecord &f = entry.data.hf;
      logFile.print("1,");
      logFile.print(f.pps_utc);      logFile.print(',');
      logFile.print(f.ms_offset);    logFile.print(',');
      logFile.print(f.processor_ms);
      for (int i = 0; i < 4; i++) {
        logFile.print(','); logFile.print(f.adc[i]);
      }
      for (int i = 0; i < 9; i++) {
        logFile.print(','); logFile.print(f.imu[i]);
      }
      logFile.println();

    } 
  
    else { // REC_LF
      LFRecord &l = entry.data.lf;
      logFile.print("2,");
      logFile.print(l.pps_utc);      logFile.print(',');
      logFile.print(l.ms_offset);    logFile.print(',');
      logFile.print(l.processor_ms); logFile.print(',');
      logFile.print(l.lat);          logFile.print(',');
      logFile.print(l.lon);          logFile.print(',');
      logFile.print(l.alt);          logFile.print(',');
      logFile.print(l.temp);         logFile.print(',');
      logFile.print(l.pressure);     logFile.print(',');
      logFile.print(l.humidity);
      logFile.println();
    }

    if (!logFile) {
      debugPrintln("flushLog: SD write failed");
      runtime.currentState = RuntimeStatus::STATE_ERROR;
      updateLED();
      break;
    }
  }

  // 3) Make sure all buffered data is committed to the card
  logFile.flush();

  // 4) Re-enable ADC interrupts now that we're done with SD
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY), adcISR, FALLING);
}

// ------------------------------------------------------
// Top-level Arduino setup()
// ------------------------------------------------------
void setup() {

  SerDebug.begin(DEBUG_BAUD);
  delay(100);
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
  delay(100);

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

  debugPrint("Initializing SD card... ");
  if (!initSD()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  delay(10);

  debugPrint("Testing SD write... ");
  if (!testSDwrite()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  delay(10);

  debugPrintln("Setting up GPS PPS Interrupt...");
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ppsISR, RISING);

  delay(10);

  debugPrintln("Setting up ADC Interrupt...");
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY), adcISR, FALLING);
  
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
    #ifdef WATCHDOG_ENABLE
      IWatchdog.reload();
    #endif
  }
  #endif

  runtime.currentState = RuntimeStatus::STATE_LOGGING;
  updateLED();
  debugPrintln("GPS lock acquired... setup complete");
  lastLfLogMs = millis();
}

void loop() {

  pollGPS();
  debugPrintln("loop: pollgps returned");
  
  if (millis() - lastLfLogMs >= 1000) {
    debugPrintln("loop: calling logLowFreqFields");
    logLowFreqFields();
     debugPrintln("loop: returned from logLowFreqFields");
    lastLfLogMs = millis();
    debugPrintln("loop: LF read returned");
  }

  rolloverFileIfNeeded();
  debugPrintln("loop: rolloverfile returned");
  flushLog();
  debugPrintln("loop: flushLog returned");
  updateLED();
  debugPrintln("loop: updateLED returned");

  #ifdef WATCHDOG_ENABLE
    IWatchdog.reload();
  #endif
}
