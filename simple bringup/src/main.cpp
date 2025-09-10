#include <Arduino.h>
#include <SPI.h>
#include "pins.h"
#include "ADS131M04.h"


HardwareSerial    SerDebug(PIN_RADIO_TX, PIN_RADIO_RX);

// ==================== Debug Macros ====================
#ifdef DEBUG_PRINT_ENABLE
  #define debugPrint(x)    SerDebug.print(x)
  #define debugPrintln(x)  SerDebug.println(x)
#else
  #define debugPrint(x)
  #define debugPrintln(x)
#endif  // DEBUG_PRINT_ENABLE

// ==================== Globals & Interfaces ====================
ADS131M04 adc;

volatile bool flag_sample_adc = false;
volatile uint32_t adc_ms = 0;
volatile uint8_t drdy_count = 0;

// ------------------------------------------------------
// ADC DRDY ISR: sample ADS131M04 at 250 Hz, keep 1/5 → 50 Hz
// ------------------------------------------------------
void adcISR()
{
  drdy_count++;
  if (drdy_count >= 5)
  {
    adc_ms = millis();
    drdy_count = 0;
    flag_sample_adc = true;
  }
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
  //uint8_t id = adc.readRegister(0x00);
  //debugPrint("ADS131M04 ID = ");
  //debugPrintln(id);

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



// ------------------------------------------------------
// Top-level Arduino setup()
// ------------------------------------------------------
void setup() {
  SerDebug.begin(115200);
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


  bool ok = true;
  
  debugPrint("Initializing ADC... ");
  if (!initADC()) {
    debugPrintln("failed");
    ok = false;
  } else debugPrintln("ok");

  delay(10);


  debugPrintln("Setting up ADC Interrupt...");
  attachInterrupt(digitalPinToInterrupt(PIN_ADC_DRDY), adcISR, FALLING);

}

void loop() {

  if(flag_sample_adc)
  {
    flag_sample_adc = false;
    adcOutput res = adc.readADC();  // one SPI read per decimated sample
    SerDebug.print("1,");
    SerDebug.print(adc_ms);
    SerDebug.print(",");
    SerDebug.print(res.ch0);
    SerDebug.print(",");
    SerDebug.print(res.ch1);
    SerDebug.print(",");
    SerDebug.print(res.ch2);
    SerDebug.print(",");
    SerDebug.println(res.ch3);
  }
}
