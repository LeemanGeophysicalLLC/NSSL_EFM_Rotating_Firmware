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

// Uncomment to enable debug prints
#define DEBUG_PRINT_ENABLE
//#define WATCHDOG_ENABLE

#define DEBUG_BAUD 115200
#define GPS_BAUD_START   9600
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

volatile uint8_t adcSampleCnt = 0;

// Buffer template
template<typename T, size_t N>
struct CircularBuffer {
  T      buf[N];
  size_t head = 0, tail = 0;
  bool   isEmpty() const { return head == tail; }
  bool   isFull()  const { return ((head+1)%N) == tail; }
  void   push(const T &v) {
    buf[head] = v;
    head = (head+1)%N;
    if (head == tail) tail = (tail+1)%N;  // overwrite oldest
  }
  bool   pop(T &out) {
    if (isEmpty()) return false;
    out = buf[tail];
    tail = (tail+1)%N;
    return true;
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
static CircularBuffer<LogEntry, 10> logBuf;

// Raw ADC storage
static uint32_t rawAdc[4];

// Timing
static uint32_t lastLfLogMs = 0;

// Wait for a UBX-ACK-ACK for the given class/id, up to timeoutMs milliseconds.
// Returns true if ACK received, false otherwise.
bool waitForAck(uint8_t ackClass, uint8_t ackId, uint16_t timeoutMs) {
  enum AckState { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PL1, PL2, CK_A, CK_B };
  AckState state = SYNC1;
  uint8_t ckA = 0, ckB = 0;
  uint8_t lenL = 0, lenH = 0;
  uint8_t payloadClass = 0, payloadId = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    if (!SerGPS.available()) continue;
    uint8_t b = SerGPS.read();

    switch (state) {
      case SYNC1:
        if (b == 0xB5) state = SYNC2;
        break;
      case SYNC2:
        if (b == 0x62) state = CLASS;
        else state = SYNC1;
        break;
      case CLASS:
        if (b == 0x05) { ckA = b; ckB = ckA; state = ID; }
        else state = SYNC1;
        break;
      case ID:
        if (b == 0x01) { ckA += b; ckB += ckA; state = LEN1; }
        else state = SYNC1;
        break;
      case LEN1:
        lenL = b;
        if (lenL == 2) { ckA += b; ckB += ckA; state = LEN2; }
        else state = SYNC1;
        break;
      case LEN2:
        lenH = b;
        if (lenH == 0) { ckA += b; ckB += ckA; state = PL1; }
        else state = SYNC1;
        break;
      case PL1:
        payloadClass = b;
        ckA += b; ckB += ckA;
        state = PL2;
        break;
      case PL2:
        payloadId = b;
        ckA += b; ckB += ckA;
        state = CK_A;
        break;
      case CK_A:
        if (b == ckA) state = CK_B;
        else state = SYNC1;
        break;
      case CK_B:
        if (b == ckB) {
          // got a complete ACK-ACK; check that it's for our message
          if (payloadClass == ackClass && payloadId == ackId) {
            return true;
          }
        }
        state = SYNC1;
        break;
    }
  }
  return false;
}

// ------------------------------------------------------
// Low-frequency sampling (1 Hz): GPS + BME → LFRecord
// ------------------------------------------------------
void logLowFreqFields() {
  // 1) Build a fresh LFRecord
  LFRecord rec{};
  rec.pps_utc   = runtime.lastUTC;
  rec.ms_offset = millis() - runtime.ppsMillis;
  rec.processor_ms = millis();

  // 2) GPS position & altitude (TinyGPS++)
  double lat_d = gps.location.lat();
  double lon_d = gps.location.lng();
  rec.lat      = int32_t(lat_d * 1e5);              // ×1e5 fixed-point
  rec.lon      = int32_t(lon_d * 1e5);
  rec.alt      = uint16_t(gps.altitude.meters() + 0.5);  // metres

  // 3) BME280 readings (Adafruit_BME280)
  bme.performReading();
  rec.temp     = int16_t(bme.temperature * 10);            // ×10 fixed-point
  float p_hPa = bme.readPressure() / 100.0f;
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

void sendUBX(const uint8_t *msg, uint16_t len) {
  uint8_t ckA = 0, ckB = 0;
  // checksum over class, id, length and payload (bytes 2..len-1)
  for (uint16_t i = 2; i < len; i++) {
    ckA += msg[i];
    ckB += ckA;
  }
  SerGPS.write(msg, len);
  SerGPS.write(ckA);
  SerGPS.write(ckB);
}

 
/**
 * @brief Waits for a UBX-ACK-ACK for the given message (by class & id).
 *
 * @param MSG  Pointer to the original UBX message header (at least 4 bytes: [sync1, sync2, class, id, ...])
 * @return true  if a matching ACK-ACK is received within the timeout
 * @return false on timeout or mismatch
 */
bool getUBX_ACK(const uint8_t *MSG) {
  // Build the expected 10-byte ACK packet:
  // [0]  = 0xB5, [1] = 0x62, [2]=0x05, [3]=0x01, [4]=0x02, [5]=0x00,
  // [6] = MSG[2] (class), [7] = MSG[3] (id), [8]=CK_A, [9]=CK_B
  uint8_t ack[10];
  ack[0] = 0xB5;
  ack[1] = 0x62;
  ack[2] = 0x05;   // ACK class
  ack[3] = 0x01;   // ACK id
  ack[4] = 0x02;   // payload length LSB
  ack[5] = 0x00;   // payload length MSB
  ack[6] = MSG[2];
  ack[7] = MSG[3];

  // Calculate checksum over ack[2..7]
  ack[8] = 0;
  ack[9] = 0;
  for (int i = 2; i < 8; i++) {
    ack[8] += ack[i];
    ack[9] += ack[8];
  }

  // Now scan incoming bytes for an exact 10-byte match
  uint8_t idx = 0;
  uint32_t start = millis();
  const uint32_t timeout = 1000;  // ms

  while (millis() - start < timeout) {
    if (!SerGPS.available()) continue;
    uint8_t b = SerGPS.read();

    if (b == ack[idx]) {
      idx++;
      if (idx == 10) {
        // got full match
        return true;
      }
    } else {
      // mismatch: if this byte is the first sync, restart idx=1, else idx=0
      idx = (b == ack[0]) ? 1 : 0;
    }
  }

  // timed out
  return false;
}


bool setDynamicMode6()
{
  /*
   * Sets up the GPS in high altitude dynamic mode 6.
   */
  static byte gps_set_sucess = 0 ;
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,
                      0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,
                      0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  uint32_t start_millis = millis();
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
    // Timeout after 10 seconds
    if ((millis() - start_millis) > 10000)
    {
      return false;
    }
  }
  gps_set_sucess=0;
  return gps_set_sucess;
}

// Poll the NAV5 config and return true if we got it.
// On success dynModel is set (0..7).
bool readNav5(uint8_t &dynModel) {
  // 1) Build & send the read-request (no payload)
  static const uint8_t pollNav5[] = {
    0xB5, 0x62,       // UBX header
    0x06, 0x24,       // class=CFG, id=NAV5
    0x00, 0x00        // length = 0  → read request
  };
  sendUBX(pollNav5, sizeof(pollNav5));

  // 2) Scan for the response header
  // We'll look for: B5 62 06 24 lenL lenH ... checksum
  uint32_t deadline = millis() + 1000;
  enum { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2 } state = SYNC1;
  uint16_t payloadLen = 0;
  while (millis() < deadline) {
    if (!SerGPS.available()) continue;
    uint8_t b = SerGPS.read();
    switch (state) {
      case SYNC1: if (b==0xB5) state=SYNC2; break;
      case SYNC2: if (b==0x62) state=CLASS; else state=SYNC1; break;
      case CLASS: if (b==0x06) state=ID; else state=SYNC1; break;
      case ID:    if (b==0x24) state=LEN1; else state=SYNC1; break;
      case LEN1:  payloadLen = b; state=LEN2; break;
      case LEN2:  payloadLen |= (uint16_t(b) << 8); goto HAVE_LEN;
    }
  }
  return false;  
HAVE_LEN:;
  if (payloadLen != 36) return false;  // NAV5 payload is always 36 bytes

  // 3) Read the 36-byte payload
  uint8_t buf[36];
  for (int i = 0; i < 36; i++) {
    while (!SerGPS.available() && millis() < deadline);
    if (millis() >= deadline) return false;
    buf[i] = SerGPS.read();
  }

  // 4) Consume checksum (2 bytes)
  while (SerGPS.available() < 2 && millis() < deadline);
  SerGPS.read(); SerGPS.read();

  // 5) Byte 6 of the payload is dynModel
  dynModel = buf[2]; // was 6
  return true;
}

bool initGPS() {
  debugPrintln("GPS: Initializing...");

  // 1) Start serial at default 9600 baud
  SerGPS.begin(9600);
  delay(1000);
  // 2) UBX-CFG-PRT → set UART1 to 115200 baud, 8N1, NMEA+UBX in/out
  static const uint8_t ubx_cfg_prt[] = {
    0xB5, 0x62,             // UBX sync
    0x06, 0x00,             // CFG-PRT
    0x14, 0x00,             // length = 20
    0x01,                   // portID = UART1
    0x00,                   // reserved
    0x00, 0x00,             // txReady = 0 (no flow control)
    0xD0, 0x08, 0x00, 0x00, // mode = 0x000008D0 → 8N1
    0x00, 0xC2, 0x01, 0x00, // baudRate = 115200
    0x03, 0x00,             // inProtoMask = UBX+NMEA
    0x03, 0x00,             // outProtoMask = UBX+NMEA
    0x00, 0x00,             // flags
    0x00, 0x00              // reserved
  };
  sendUBX(ubx_cfg_prt, sizeof(ubx_cfg_prt));
  delay(1000);    // give the module a moment to respond
  debugPrintln("GPS: Port set to 115200");
  // reopen serial at new baud
  SerGPS.begin(115200);
  delay(100);

  /*
   * Turn off unused messages
   */

  // ------------------------------------------------------
// Trim NMEA output on UART1 down to only GGA, RMC, ZDA
// ------------------------------------------------------
{
  // 1) Disable everything we don’t parse: GLL(0x01), GSA(0x02), GSV(0x03),
  //    VTG(0x05), GST(0x07), GNS(0x0D)
  static const uint8_t nmeaOff[] = { 0x01, 0x02, 0x03, 0x05, 0x07, 0x0D };
  for (uint8_t msgId : nmeaOff) {
    // UBX-CFG-MSG (class=0x06,id=0x01,len=3) payload = [0xF0,msgId,0]
    uint8_t cfg[] = {
      0xB5, 0x62,             // header
      0x06, 0x01,             // CFG-MSG
      0x03, 0x00,             // length = 3
      0xF0, msgId, 0x00       // NMEA class(0xF0), message ID, rate=0 (off)
    };
    sendUBX(cfg, sizeof(cfg));
    delay(10);
  }

  // 2) Enable only GGA(0x00), RMC(0x04), ZDA(0x08) on UART1
  static const uint8_t nmeaOn[]  = { 0x00, 0x04, 0x08 };
  for (uint8_t msgId : nmeaOn) {
    uint8_t cfg[] = {
      0xB5, 0x62,
      0x06, 0x01,
      0x03, 0x00,
      0xF0, msgId, 0x01       // rate=1 → enabled
    };
    sendUBX(cfg, sizeof(cfg));
    delay(10);
  }
}

  /*
   * Set dynamic model
   */

  // drain garbage
while (SerGPS.available()) SerGPS.read();

static const uint8_t fullNav5[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00,  // header + length=36
  0xFF, 0xFF,                          // mask = 0xFFFF
  0x06,                                // dynModel = airborne <1g
  0x03,                                // fixMode = auto 2D/3D
  // exactly 32 more bytes of zero/default config
  0x00,0x00,0x00,0x00,
  0x10,0x27,0x00,0x00,
  0x05,0x00,0xFA,0x00,
  0xFA,0x00,0x64,0x00,
  0x2C,0x01,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00
};

sendUBX(fullNav5, sizeof(fullNav5));
delay(200);
uint8_t actualDyn;
if (readNav5(actualDyn)) {
  debugPrint("After full-mask NAV5, dynModel = ");
  debugPrintln(actualDyn);
} else {
  debugPrintln("readNav5() failed");
}

  // 4) Flush any stray NMEA/UBX bytes
  while (SerGPS.available()) SerGPS.read();

  debugPrintln("GPS: initGPS() complete");
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
  SerGPS.begin(GPS_BAUD_START);

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


  debugPrintln("Initializing SPI...");
  SPI.begin();
  debugPrintln("Initializing I2C...");
  Wire.begin();
  Wire.setClock(400000);

  bool ok = true;
  
  debugPrint("Initializing ADC... ");
  if (!initADC()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  debugPrint("Initializing IMU... ");
  if (!initIMU()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  debugPrint("Initializing BME688... ");
  if (!initBME()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  debugPrint("Initializing GPS... ");
  if (!initGPS()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");

  debugPrint("Initializing SD card... ");
  if (!initSD()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  debugPrint("Testing SD write... ");
  if (!testSDwrite()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");
  
  debugPrintln("Setting up GPS PPS Interrupt...");
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), ppsISR, RISING);

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

  debugPrintln("Waiting for GPS lock...");
  runtime.currentState = RuntimeStatus::STATE_ACQUIRING_GPS;
  updateLED();
  while (!isGPSLocked()) {
    pollGPS();
    #ifdef WATCHDOG_ENABLE
      IWatchdog.reload();
    #endif
  }
  
  runtime.currentState = RuntimeStatus::STATE_LOGGING;
  updateLED();
  debugPrintln("GPS lock acquired... setup complete");
  lastLfLogMs = millis();
}

void loop() {

  pollGPS();
  //debugPrintln("loop: pollgps returned");
  
  if (millis() - lastLfLogMs >= 1000) {
    logLowFreqFields();
    lastLfLogMs = millis();
    //debugPrintln("loop: LF read returned");
  }

  rolloverFileIfNeeded();
  //debugPrintln("loop: rolloverfile returned");
  flushLog();
  //debugPrintln("loop: flushLog returned");
  updateLED();
  //debugPrintln("loop: updateLED returned");

  #ifdef WATCHDOG_ENABLE
    IWatchdog.reload();
  #endif
}
