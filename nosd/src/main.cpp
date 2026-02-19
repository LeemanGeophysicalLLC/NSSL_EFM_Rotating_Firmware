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
#define WATCHDOG_ENABLE

#define LOG_BAUD 115200
#define GPS_BAUD 115200


// ==================== Debug Macros ====================
#ifdef DEBUG_PRINT_ENABLE
  #define debugPrint(x)    SerLog.print(x)
  #define debugPrintln(x)  SerLog.println(x)
#else
  #define debugPrint(x)
  #define debugPrintln(x)
#endif  // DEBUG_PRINT_ENABLE

// ==================== Globals & Interfaces ====================
HardwareSerial    SerLog(PIN_LOG_TX, PIN_LOG_RX);
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
  uint32_t pps_date;
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


/**
 * @brief Struct to hold runtime flags and timestamps for system state management.
 */
struct RuntimeStatus {
  volatile uint32_t ppsMillis = 0;              ///< Timestamp of last PPS signal
  volatile bool ppsSeen = false;                ///< PPS pulse seen flag
  volatile uint32_t gpsEpochMillis = 0;  // millis() at PPS edge for utcAtLastPps
  volatile uint32_t utcAtLastPps_hhmmss00 = 0; // UTC second aligned to gpsEpochMillis
  volatile uint32_t dateAtLastPps_yyyymmdd = 0; // date aligned to gpsEpochMillis
  volatile bool haveTimeAnchor = false;
  uint32_t gpsLastValidMillis = 0;              ///< Last millis() when GPS time was valid
  bool gpsLocked = false;                       ///< GPS has valid time and PPS sync
  uint32_t utc_yyyymmdd = 0; // YYYYMMDD packed date
  bool dateValid = false;   // have we ever parsed a valid NMEA date?
  uint32_t utc_hhmmss00 = 0;   // hhmmss00 (centiseconds=00)
  bool utcValid = false;   // have we ever parsed a valid NMEA time?


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

static CircularBuffer<LogEntry, 128> logBuf;

/**
 * @brief Bump a packed UTC time (hhmmsscc) forward by exactly one second.
 *
 * This treats @p t as a 32-bit integer where hours, minutes, seconds, and
 * centiseconds are packed as HHMMSSCC (e.g., 13:07:59.00 -> 13075900).
 * The function:
 *   - extracts HH, MM, SS,
 *   - increments SS by one,
 *   - rolls MM and HH as needed (59->00 with carry; hours wrap at 24),
 *   - forces centiseconds (CC) to 00 on output.
 *
 * @param t  Packed time in the form HHMMSSCC. CC is ignored on input and the
 *           output will always have CC = 00.
 *
 * @return   The next second as a packed HHMMSSCC value with CC = 00.
 *
 * @note
 * - Valid input range is 00000000..23595999 (though only CC=00 is meaningful).
 * - 23:59:59.XX rolls over to 00:00:00.00.
 * - Centiseconds are not tracked; use a separate millisecond/offset field
 *   for sub-second timing.
 *
 * @code
 *   uint32_t t = 12595900;                 // 12:59:59.00
 *   t = bump_hhmmss00(t);                  // -> 13000000 (13:00:00.00)
 *   t = bump_hhmmss00(23595942);           // -> 00000000 (CC ignored)
 * @endcode
 */
static inline uint32_t bump_hhmmss00(uint32_t t) {
  uint8_t hh = (t / 1000000UL) % 100;
  uint8_t mm = (t / 10000UL)   % 100;
  uint8_t ss = (t / 100UL)     % 100;  // cc stays 00 for us
  ss++;
  if (ss == 60) { ss = 0; mm++; if (mm == 60) { mm = 0; hh = (hh + 1) % 24; } }
  return (uint32_t)hh * 1000000UL + (uint32_t)mm * 10000UL + (uint32_t)ss * 100UL;
}

/**
 * @brief Determine if a given year is a leap year (Gregorian calendar).
 *
 * Implements the standard rule:
 *  - Years divisible by 4 are leap years,
 *  - except those divisible by 100,
 *  - except those divisible by 400 (which are leap years).
 *
 * @param y  Four-digit year (e.g., 2025).
 * @return true if @p y is a leap year; false otherwise.
 */
static inline bool isLeap(uint16_t y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

/**
 * @brief Increment a packed date (YYYYMMDD) by exactly one day.
 *
 * Parses the input into year/month/day, advances the day, and rolls month/year
 * as needed, accounting for month lengths and leap years (via isLeap()).
 *
 * @param d  Packed date as YYYYMMDD (e.g., 20250912).
 * @return   Next calendar day as YYYYMMDD (e.g., 20250913).
 *
 * @note Expects a valid Gregorian date with 1 ≤ month ≤ 12 and a day within
 *       that month. February is treated as 29 days on leap years.
 * @code
 *   bump_yyyymmdd(20240228) -> 20240229
 *   bump_yyyymmdd(20240229) -> 20240301
 *   bump_yyyymmdd(20231231) -> 20240101
 * @endcode
 */
static inline uint32_t bump_yyyymmdd(uint32_t d) {
  uint16_t y = d / 10000U;
  uint8_t  m = (d / 100U) % 100U;
  uint8_t  day = d % 100U;

  static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  uint8_t dim = mdays[(m ? m : 1) - 1];
  if (m == 2 && isLeap(y)) dim = 29;

  day++;
  if (day > dim) { day = 1; m++; if (m > 12) { m = 1; y++; } }

  return (uint32_t)y * 10000UL + (uint32_t)m * 100UL + (uint32_t)day;
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


/**
 * @brief Display a power-on RGB LED sequence to indicate the device is alive.
 *
 * Cycles the status LED through RED → GREEN → BLUE → OFF, pausing ~1 s between
 * each color (total duration ≈ 4 s). After the sequence completes, control of
 * the LED is handed back to the normal state indicator via updateLED().
 *
 * @details
 * - This routine is **blocking** (uses delay()) and is intended to be called
 *   once from setup(). Avoid calling during time-critical acquisition.
 * - Uses setLED(r,g,b) which typically relies on PWM/analogWrite. On boards
 *   without PWM on these pins, the behavior may degrade to ON/OFF.
 *
 * @pre PIN_LED_RED, PIN_LED_GREEN, and PIN_LED_BLUE must be defined and wired.
 * @pre setLED(...) and updateLED() must be available.
 *
 * @return void
 */

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
    if (gps.encode(c)) {
      // Always keep lastUTC fresh when valid
      if (gps.time.isValid()) {
        uint32_t nmea = gps.time.value();       // hhmmsscc
        // snap our counter to NMEA, forcing cc to 00
        runtime.utc_hhmmss00 = (nmea / 100U) * 100U;
        runtime.utcValid = true;
        runtime.gpsLastValidMillis = millis();
      }

      if (gps.date.isValid()) {
        // TinyGPS++ gives full year/month/day
        uint16_t y = gps.date.year();
        uint8_t  m = gps.date.month();
        uint8_t  d = gps.date.day();
        runtime.utc_yyyymmdd = (uint32_t)y * 10000UL + (uint32_t)m * 100UL + (uint32_t)d;
        runtime.dateValid = true;
      }

      // Align millis to the top-of-second when we see PPS
      if (isGPSLocked() && runtime.ppsSeen) {
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
    const uint32_t prev = runtime.utc_hhmmss00;
    runtime.utc_hhmmss00 = bump_hhmmss00(runtime.utc_hhmmss00);

    // If we just rolled over 23:59:59 → 00:00:00, bump date too
    if (prev == 23595900U && runtime.dateValid) {
      runtime.utc_yyyymmdd = bump_yyyymmdd(runtime.utc_yyyymmdd);
    }
  }
  flag_read_lf = true;
}



// ------------------------------------------------------------
// ADC DRDY ISR: sample ADS131M04 at 250 Hz, keep 1/5 for 50 Hz
// ------------------------------------------------------------
void adcISR() {
  static uint8_t ready_cnt = 0;
  if (++ready_cnt >= 5)
  {
    ready_cnt = 0;
    flag_read_hf = true;
    adc_ms = millis();
    adc_pps_offset = (adc_ms - runtime.ppsMillis + 1000) % 1000;  // 0..999

  }
}

// Scales for IMU serialization (units → scaled ints)
static inline int32_t acc_to_mg(float g)      { return (int32_t)lroundf(g * 1000.0f); } // ±16g -> ±16000
static inline int32_t gyr_to_dps10(float dps) { return (int32_t)lroundf(dps * 10.0f); } // ±2000 dps -> ±20000
static inline int32_t mag_to_nT(float uT)     { return (int32_t)lroundf(uT * 1000.0f); }   // typical ±4900 uT


/**
 * @brief Initialize and configure the ADS131M04 ADC.
 *
 * Performs a hardware reset, begins the ADC with specified SPI/DRDY pins,
 * enables all four channels, sets each channel’s PGA to 1×, assigns the
 * input mux (AIN0P/AIN0N as currently coded), and applies OSR=16384.
 *
 * @note Uses small delays to satisfy reset/startup timing.
 * @note Returns true unconditionally; add register reads/status checks if you
 *       need to detect init failures.
 *
 * @return true on completion.
 */

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
  return true;
}


/**
 * @brief Initialize the ICM-20948 IMU over I²C and set full-scale ranges.
 *
 * Calls begin() using AD0=HIGH (address select = 1), then configures
 * accelerometer to ±16 g and gyro to ±2000 dps. Returns false if either
 * the device init or the full-scale configuration fails.
 *
 * @return true on success; false if imu.status != ICM_20948_Stat_Ok after
 *         begin() or setFullScale().
 */
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


/**
 * @brief Initialize the BME680 environmental sensor and start the first read.
 *
 * Probes the device at I2C address 0x76. On success, disables the gas heater,
 * sets oversampling (Temp=8x, Hum=2x, Press=4x), applies IIR filter size 3,
 * and calls beginReading() to kick off an asynchronous conversion.
 *
 * @return true on success; false if bme.begin(0x76) fails.
 *
 * @note Pair beginReading() with endReading() later to retrieve results
 * without blocking.
 */
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

    SerLog.println("# BME680: Initialized successfully");
  } else {
    SerLog.println("# BME680: Initialization failed");
    return false;
  }
  return true;
}


/**
 * @brief Initialize and configure the u-blox GNSS module.
 *
 * Sequence:
 *  - Probe device over I²C with myGNSS.begin().
 *  - (Re)apply defaults (factoryReset/factoryDefault) and wait briefly.
 *  - Set dynamic model to AIRBORNE <2g>.
 *  - Configure UART1 to 115200 baud and start SerGPS at GPS_BAUD.
 *  - Set navigation rate to 1 Hz.
 *  - Disable common NMEA, enable RMC/ZDA/GGA on UART1.
 *  - Configure TIMEPULSE (TP5) at 1 Hz, 100 ms, aligned when locked.
 *  - Save configuration to NVM.
 *
 * @return true on success; false if begin() fails or required config calls
 *         (dynamic model, navigation frequency, saveConfiguration) fail.
 */
bool initGPS() {

  if (!myGNSS.begin()) {
    SerLog.println("# GPS not detected on I2C bus.");
    return false;
  }

  SerLog.println("# Configuring GPS...");
  myGNSS.factoryReset();
  myGNSS.factoryDefault();
  delay(2000);

  // Set dynamic model to AIRBORNE <4g>
  if (!myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g)) {
    SerLog.println("# Failed to set dynamic model");
    return false;
  }

  // Set UART1 baud rate to 115200
  myGNSS.setSerialRate(115200, COM_PORT_UART1); // No return value
  SerLog.println("# Set UART1 baud rate to 115200.");
  delay(100);
  SerGPS.begin(GPS_BAUD);
  delay(100);

  // Set navigation rate to 1 Hz
  if (!myGNSS.setNavigationFrequency(1))
  {
    SerLog.println("# Failed to set navigation frequency");
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
    SerLog.println("# Failed to configure time pulse!");
  }
  else
  {
    SerLog.println("# Time pulse configured.");
  }

  // Save settings
  if (!myGNSS.saveConfiguration())
  {
    SerLog.println("# Failed to save configuration!");
    return false;
  }
  else
  {
    SerLog.println("# GPS configuration saved.");
  }

  SerLog.println("# Setup complete. GPS will output RMC/ZDA/GGA at 115200 baud over UART1.");
  return true;
}


/**
 * @brief Board bring-up and system initialization.
 *
 * Initializes the debug UART, (optionally) the watchdog, configures GPIO for
 * LEDs / chip-selects / interrupts, starts SPI and I2C, then initializes
 * peripherals (ADS131M04, ICM-20948, BME680, u-blox GNSS). Runs a brief
 * startup LED sequence, optionally blocks until a valid GPS fix/time is
 * acquired, attaches PPS and ADC DRDY interrupts, and enters LOGGING state.
 *
 * @note This routine is intentionally blocking and includes delays
 *       (e.g., LED sequence, GNSS setup, optional GPS wait).
 * @note On any init failure, sets STATE_ERROR and enters a blinking loop.
 *
 * @pre Pin definitions (PIN_*), setLED(), updateLED(), and init*() must exist.
 * @post PPS/DRDY ISRs armed; system ready to stream/log sensor data.
 */
void setup() {

  SerLog.begin(LOG_BAUD);
  delay(500);

  SerLog.println("# Configuring pins...");
  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_BLUE,   OUTPUT);
  pinMode(PIN_ADC_CS,     OUTPUT);
  pinMode(PIN_ADC_DRDY,   INPUT);
  pinMode(PIN_GPS_PPS,    INPUT);
  pinMode(PIN_ADC_RESET, OUTPUT);
  pinMode(PIN_OPENLOG_DET, INPUT);

  digitalWrite(PIN_ADC_CS, HIGH); // ensure ADC CS is high

  delay(50);
  SerLog.println("# Initializing SPI...");
  SPI.begin();
  delay(50);
  SerLog.println("# Initializing I2C...");
  Wire.begin();
  Wire.setClock(400000);
  delay(300);

  bool ok = true;

  SerLog.print("# Initializing ADC... ");
  if (!initADC()) {
    SerLog.println("# failed");
    ok = false;
  } else SerLog.println("# ok");

  delay(10);

  SerLog.print("# Initializing IMU... ");
  if (!initIMU()) {
    SerLog.println("# failed");
    ok = false;
  } else SerLog.println("# ok");

  delay(10);

  SerLog.print("# Initializing BME688... ");
  if (!initBME()) {
    SerLog.println("# failed");
    ok = false;
  } else SerLog.println("# ok");

  delay(10);

  SerLog.print("# Initializing GPS... ");
   if (!initGPS()) {
      SerLog.println("# failed");
      ok = false;
  } else SerLog.println("# ok");

  delay(10);

  SerLog.println("# Checking openlog status...");
  if (digitalRead(PIN_OPENLOG_DET) == LOW) {
    SerLog.println("# OpenLog error!");
    ok = false;
  } else {
    SerLog.println("# OpenLog ok.");
  }
  
  startupLedSequence();
  if (!ok) {
    SerLog.println("# Setup failed — entering ERROR state");
    // show red LED
    runtime.currentState = RuntimeStatus::STATE_ERROR;
    updateLED();
    while (1) {setLED(255, 0, 0); delay(100); setLED(0, 0, 0); delay(100);} // blink red
  }

  #ifdef WAIT_ON_GPS_FIX
  SerLog.println("# Waiting for GPS lock...");
  runtime.currentState = RuntimeStatus::STATE_ACQUIRING_GPS;
  updateLED();
  while (!isGPSLocked()) {
    pollGPS();
    delay(100);
    #ifdef WATCHDOG_ENABLE
      IWatchdog.reload();
    #endif
  }
  #endif

  SerLog.println("# Setting up GPS PPS Interrupt...");
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ppsISR, RISING);

  delay(10);

  SerLog.println("# Setting up ADC Interrupt...");
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY), adcISR, FALLING);

  runtime.currentState = RuntimeStatus::STATE_LOGGING;
  updateLED();
  SerLog.println("# GPS lock acquired... setup complete");

  #ifdef WATCHDOG_ENABLE
    SerLog.println("# Starting watchdog...");
    IWatchdog.begin(26000000);    // max ~26 208 000 µs
    SerLog.println("# Watchdog running");
  #endif

  SerLog.println("# type,hhmmss00,msOff_from_pps,processor_ms,efield_raw,analog_acc_x_raw,analog_acc_y_raw,analog_acc_z_raw,ax_mg,ay_mg,az_mg,gx_dps10,gy_dps10,gz_dps10,mx_nT,my_nT,mz_nT  # type=1 (HF)");
  SerLog.println("# type,YYYYMMDDThhmmss00,msOff_ms,processor_ms,lat_e5,lon_e5,alt_m,temp_c_x10,press_hPa_x10,hum_%_x10 # type=2 (LF)");
}


/**
 * @brief Format a LogEntry as a CSV line into @p out.
 *
 * HF format:
 *   1,ppsUTC,msOff,ms,ch0,ch1,ch2,ch3,ax,ay,az,gx,gy,gz,mx,my,mz
 * LF format:
 *   2,ppsUTC,msOff,ms,latE5,lonE5,altM,temp_x10,pres_hPa_x10,hum_x10
 *
 * @param e    Log entry to serialize (REC_HF or REC_LF).
 * @param out  Destination buffer for the CSV string.
 * @param cap  Size of @p out in bytes.
 * @return int Number of characters that would be written (snprintf-style).
 *             If the return value >= cap, the output was truncated.
 *
 * @note IMU floats are scaled to compact int fields before printing.
 */
static int formatLine(const LogEntry &e, char *out, size_t cap) {
  if (e.type == REC_HF) {
    const HFRecord &r = e.data.hf;
    // Convert IMU floats to compact ints for lightweight printing
    int32_t ax = acc_to_mg(r.imu[0]), ay = acc_to_mg(r.imu[1]), az = acc_to_mg(r.imu[2]);
    int32_t gx = gyr_to_dps10(r.imu[3]), gy = gyr_to_dps10(r.imu[4]), gz = gyr_to_dps10(r.imu[5]);
    int32_t mx = mag_to_nT(r.imu[6]),  my = mag_to_nT(r.imu[7]),  mz = mag_to_nT(r.imu[8]);

    return snprintf(out, cap,
      "1,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
      (unsigned long)r.pps_utc,
      (unsigned long)r.ms_offset,
      (unsigned long)r.processor_ms,
      (long)r.adc[0], (long)r.adc[1], (long)r.adc[2], (long)r.adc[3],
      (long)ax, (long)ay, (long)az, (long)gx, (long)gy, (long)gz, (long)mx, (long)my, (long)mz
    );

  } else {
    const LFRecord &r = e.data.lf;
    // CSV: 2,ppsUTC,msOff,ms, latE5,lonE5, altM, temp_x10, pres_hPa_x10, hum_x10
    return snprintf(out, cap,
      "2,%08luT%08lu,%lu,%lu,%ld,%ld,%u,%d,%d,%d\n",
      (unsigned long)r.pps_date,
      (unsigned long)r.pps_utc,
      (unsigned long)r.ms_offset,
      (unsigned long)r.processor_ms,
      (long)r.lat, (long)r.lon, (unsigned)r.alt, (int)r.temp, (int)r.pressure, (int)r.humidity
    );
  }
}


/**
 * @brief Non-blocking drain of queued LogEntry records to the debug UART.
 *
 * Pops up to MAX_PER_CALL entries from logBuf, formats each to CSV with
 * formatLine(), and writes as many bytes as SerLog’s TX buffer can accept.
 * Uses a static “pending” buffer to handle partial line writes across calls.
 *
 * Call this frequently from loop(); it returns quickly if the UART has no space.
 *
 * @pre SerLog initialized; formatLine() and logBuf available.
 * @note One line may be held in the pending buffer while the UART drains.
 * @return void
 */
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
    int room = SerLog.availableForWrite();   // may be small (even 1–2 bytes)
    if (room <= 0) break;
    int chunk = pendingLen - pendingOff;
    if (chunk > room) chunk = room;

    SerLog.write((uint8_t*)line + pendingOff, chunk);
    pendingOff += chunk;

    // Finished the line?
    if (pendingOff >= pendingLen) {
      hasPending = false;
      ++sent;
    }
  }
}


/**
 * @brief Acquire one high-frequency sample (ADC + IMU) and queue it.
 *
 * Reads a decimated ADS131M04 sample, optionally reads ICM-20948 AGMT if the
 * IMU is OK, stamps PPS-aligned time fields, and pushes the populated HFRecord
 * into the log ring buffer for later serialization.
 *
 * Fields set:
 *  - pps_utc      : seconds (hhmmss00) advanced at PPS
 *  - ms_offset    : ms since last PPS (0..999)
 *  - processor_ms : millis() at acquisition
 *  - adc[0..3]    : ADS131M04 channels
 *  - imu[0..8]    : accX/Y/Z, gyrX/Y/Z, magX/Y/Z (0.0 if IMU unavailable)
 *
 * @note Intended to be called from the main loop when the DRDY flag is set,
 *       not from the ISR itself.
 * @return void
 */
void readHFSensors()
{
  adcOutput tmp = adc.readADC();  // one SPI read per decimated sample

  HFRecord rec{};
  rec.pps_utc = runtime.utc_hhmmss00;
  rec.ms_offset = adc_pps_offset;
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


/**
 * @brief Acquire one low-frequency sample (GPS + BME680) and queue it.
 *
 * Stamps PPS-aligned time fields, reads TinyGPS++ position/altitude, fetches
 * the previous asynchronous BME680 conversion via endReading(), then starts
 * the next conversion with beginReading(). Packs results into an LFRecord and
 * pushes it to the ring buffer for later serialization.
 *
 * Fields / units:
 *  - pps_utc      : hhmmss00 (advanced at PPS)
 *  - ms_offset    : ms since last PPS (0..999)
 *  - processor_ms : millis() at acquisition
 *  - lat, lon     : degrees × 1e5 (int32)
 *  - alt          : metres (uint16)
 *  - temp         : °C × 10 (int16)
 *  - pressure     : hPa × 10 (int16)  [from Pa/100]
 *  - humidity     : %RH × 10 (int16)
 *
 * @note If BME endReading() fails, temp/pressure/humidity are set to INT16_MIN.
 * @note Intended to run once per PPS (when the LF flag is set).
 * @return void
 */
void readLFSensors()
{
  // 1) Build a fresh LFRecord
  LFRecord rec{};
  rec.pps_utc = runtime.utc_hhmmss00;
  rec.pps_date   = runtime.utc_yyyymmdd;
  rec.ms_offset = (millis() - runtime.ppsMillis + 1000) % 1000;
  rec.processor_ms = millis();

  // 2) GPS position & altitude (TinyGPS++)
  debugPrintln("readLFSensors: Getting GPS position and altitude");
  double lat_d = gps.location.lat();
  double lon_d = gps.location.lng();
  rec.lat      = int32_t(lat_d * 1e5);
  rec.lon      = int32_t(lon_d * 1e5);
  rec.alt      = uint16_t(gps.altitude.meters() + 0.5);

  debugPrintln("readLFSensors: Getting BME readings");

  if (bme.endReading()) {
    // Temperature (degC) ×10
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
  debugPrintln("readLFSensors: LF record queued");
}


/**
 * @brief Main run loop: service HF/LF sampling, GPS, and output.
 *
 * - If DRDY/PPS ISRs have set flags, read high/low-frequency sensors and queue records.
 * - Poll GNSS serial to keep TinyGPS++ fed and maintain time/lock.
 * - Drain queued CSV records to the debug UART without blocking.
 * - Update status LED and (optionally) pet the watchdog.
 *
 * @note Keep this loop fast; heavy work is done outside ISRs via the flags.
 * @return void
 */
void loop() {

  if (flag_read_hf)
  {
    flag_read_hf = false;
    readHFSensors();
  }

  if (flag_read_lf)
  {
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
