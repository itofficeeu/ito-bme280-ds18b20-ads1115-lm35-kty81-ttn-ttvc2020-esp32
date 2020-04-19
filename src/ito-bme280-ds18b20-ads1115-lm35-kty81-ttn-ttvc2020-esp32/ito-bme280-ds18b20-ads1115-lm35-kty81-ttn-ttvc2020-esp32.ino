/*
 * This program is written by Andreas C. Dyhrberg, www.itoffice.eu.
 * 
 * The mentioning of "Minimal-X Extension Board" refers to PCB hardware
 * produced by www.itoffice.eu.
 * 
 */

/* ====================== START of DEFINITIONS ====================== */

#include <strings.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include "SSD1306.h"
#include <AsyncDelay.h>
#include <SoftWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_ADS1015.h>
#include <lmic.h>
#include <hal/hal.h>
#include <CayenneLPP.h>

#define DEBUG_MONITOR 1

#if DEBUG_MONITOR
uint8_t  prepend = 0;
#endif

uint16_t loop_count = 0;

/* ==================== START of LORA DEFINITIONS =================== */

static osjob_t sendjob;

/* LoRaWAN NwkSKey, AppSKey and end-device address (DevAddr) for 
 * the application called '___________________'
 * and the device called '___________________'. */

/* LoRaWAN NwkSKey, network session key */
static const PROGMEM u1_t NWKSKEY[16] = 
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* LoRaWAN AppSKey, application session key */
static const u1_t PROGMEM APPSKEY[16] = 
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* LoRaWAN end-device address (DevAddr) */
static const u4_t DEVADDR = { 0x00000000 };
/* Application: "", device: "" */

/* These callbacks are only used in over-the-air activation, so they are 
 * left empty here (we cannot leave them out completely unless 
 * DISABLE_JOIN is set in config.h, otherwise the linker will complain). */
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/* Schedule data trasmission in every this many seconds. (Might become 
 * longer due to duty cycle limitations). Fair Use policy of TTN 
 * requires update interval of at least several min. Note that frequency 
 * hopping might influence how often single channel gateways receive data.
 * We set update interval here of 10 sec. for testing.
 */
const unsigned TX_INTERVAL = 10;

/* Maximum size of the string/buffer/package to be transmitted is in 
 * this case now defined to be 51 bytes. */
CayenneLPP lpp(51);

/* ===================== END of LORA DEFINITIONS ==================== */

/* ============== START of MICROCONTROLLER DEFINITIONS ============== */

/* Lora related - Pin mapping according to Cytron LoRa Shield RFM. 
 * These settings also works for Heltec WiFi LoRa 32 and TTGO LoRa 32 */
const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 34, 35},
};

/* Define wich pin will be used to power the mosfet feeding the board 
 * and sensors with ground. The microcontroller is of course excluded 
 * from this feature. */
#define POWER_PIN 12

/* I2C sensors can per default not use pins 21 and 22 on Heltec WiFi Lora 32 
 * when the display is used with SSD1306.h. One solution can be to let 
 * the physical wiring of the I2C sensors use pin 4 and 15. */
#define I2C_PIN_SDA 4
#define I2C_PIN_SCL 15

SoftWire sw(I2C_PIN_SDA, I2C_PIN_SCL);

/* Set 'DISPLAY_USE 0' to turn off the display. */
#define DISPLAY_USE 1
#define DISPLAY_PIN_CONTROL 16 /* For resetting the display */
SSD1306 display(0x3C, I2C_PIN_SDA, I2C_PIN_SCL);

/* Conversion factor for micro seconds to seconds: */
#define ESP32_SLEEP_uS_TO_S_FACTOR 1000000

#define ESP32_SLEEP_TIME_SEC 300
#define ESP32_SLEEP_ENABLED 1

/* Definitions for the microcontroller */
//#define MC_ANALOG_READ_LIMIT 4095 /* Esp32: 4095, Arduino: 1023 */
//const float MC_ANALOG_VOLT_LIMIT = 3.3;  /* Volt */

/* The circuit on the Minimal-X Extension Board where the sensor is inserted. */
//const float VOLT_DIVIDER_R1 = 26850; /* Ohm */
//const float VOLT_DIVIDER_R2 = 26850; /* Ohm */
//const float VOLTAGE_RATIO = VOLT_DIVIDER_R2 / (VOLT_DIVIDER_R1+VOLT_DIVIDER_R2);

/* Implications of the combination of the microcontroller and the extension board. */
//const float MC_VOLTAGE_MEASURING_LIMIT = MC_ANALOG_VOLT_LIMIT / VOLTAGE_RATIO;

/* Setting the I2C-address of the ADS1115 to 0x48 */
Adafruit_ADS1115 ads(0x48);

#define  ADS_CHANNEL_0 0
#define  ADS_CHANNEL_1 1
#define  ADS_CHANNEL_2 2
#define  ADS_CHANNEL_3 3

/* Specifications of the ADS1115. - WARNING HAZARD NOTE: You can maximal 
 * apply the Vcc to the analog input pins! */
const float ADS_VOLT_LIMIT  = 6.144; /* Volt - Theoretical only! Read the datasheet! */
const float ADS_READ_LIMIT  = 32767; /* Esp32: 0-4095, Arduino: 0-1023, ADS1115: +/-32767 */

/* The circuits/ports/connectors on the Minimal-X Extension Board where 
 * the sensors are inserted, and implications of the ADS1115 and the 
 * extension board combined. */
const float ADS_CH0_R1_OHM = 0.0000001; /* Approximation of the value zero */
const float ADS_CH0_R2_OHM = 100000000; /* Approximation of the value indefinite high */
const float ADS_CH0_RATIO = ADS_CH0_R2_OHM / (ADS_CH0_R1_OHM +ADS_CH0_R2_OHM);
const float ADS_CH0_VOLT_LIMIT = ADS_VOLT_LIMIT / ADS_CH0_RATIO;

const float ADS_CH1_R1_OHM = 38600;
const float ADS_CH1_R2_OHM = 38200;
const float ADS_CH1_RATIO = ADS_CH1_R2_OHM / (ADS_CH1_R1_OHM +ADS_CH1_R2_OHM);
const float ADS_CH1_VOLT_LIMIT = ADS_VOLT_LIMIT / ADS_CH1_RATIO;

const float ADS_CH2_R1_OHM = 21800;
const float ADS_CH2_R2_OHM =  3020;
const float ADS_CH2_RATIO = ADS_CH2_R2_OHM / (ADS_CH2_R1_OHM +ADS_CH2_R2_OHM);
const float ADS_CH2_VOLT_LIMIT = ADS_VOLT_LIMIT / ADS_CH2_RATIO;

const float ADS_CH3_R1_OHM = 14840;
const float ADS_CH3_R2_OHM =   993;
const float ADS_CH3_RATIO = ADS_CH3_R2_OHM / (ADS_CH3_R1_OHM +ADS_CH3_R2_OHM);
const float ADS_CH3_VOLT_LIMIT = ADS_VOLT_LIMIT / ADS_CH3_RATIO;

/* The sensor module "Sensor Adapter Switchboard", 
 * alias the circuit around the KTY81. Setup as a voltage divider like this:
 * 
 *                R1          R2
 * 
 *  (Vcc 5+)--+-->3k--+-->kty81-210---->(GND)
 *                    |
 *                    +-----> ADC (Analog Port of the microcontroller or ADS1115)
 *                    
 * The 3k Ohm can be between 2.7 K and 5.4 K Ohm.
 */
const float kty81210_module_vdd = 5.2; /* Volt */
const float kty81210_module_r1 = 3000; /* Ohm - Can be between 2.7 and 5.4 K Ohm. */
const float kty81210_module_kty81210_rating_at_25_degrees_celcius = 2000; /* Ohm */
const float kty81210_module_calibration_value = 60;

float kty81_210_value = 0;

float lm35_value = 0;

/* Defining the pin for 1-wire to be pin 17 */
#define ONE_WIRE_BUS 17
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature oneWireSensors(&oneWire);

DeviceAddress ds18s20_nr_1_id = { 0x10, 0x0E, 0x8C, 0x41, 0x03, 0x08, 0x00, 0xFF };
float ds18s20_nr_1_celcius = 0;
//DeviceAddress sensor2 = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//float ds18s20_nr_2_celcius = 0;
//DeviceAddress sensor3 = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
//float ds18s20_nr_3_celcius = 0;

/* Check the Adafruit library of the BME280 to check and set the correct address. */
Adafruit_BME280 bme;
float bmeTemp = 0;
float bmeHumi = 0;
float bmePres = 0;
float bmeAlti = 0;
float bmePresAtStart = 0;

/* =============== END of MICROCONTROLLER DEFINITIONS =============== */

/* ======================= END of DEFINITIONS ======================= */

/* ======================= START of FUNCTIONS ======================= */

#if ESP32_SLEEP_ENABLED
void Esp32WakeupReasonPrint()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.print("Wakeup was caused by: ");
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("External signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("External signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("ULP program"); break;
    default : Serial.printf("Something else than deep sleep. Wakeup code = %d\n",wakeup_reason); break;
  }
}
#endif

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      /* Schedule next transmission */
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), lora_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      /* Data received in ping slot. */
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void lora_send(osjob_t* j)
{
#if DEBUG_MONITOR
  //Serial.println(F("lora_send(osjob_t* j)"));
#endif

  /* Check if there is not a current TX/RX job running */
  if (LMIC.opmode & OP_TXRXPEND) 
  {
#if DEBUG_MONITOR
    Serial.printf("OP_TXRXPEND going on. => Will not send anything.\r\n");
#endif
  }
  else if (!(LMIC.opmode & OP_TXRXPEND)) 
  {
    FillDataToBeSendViaLora();

    /* Prepare upstream data transmission at the next possible time. */
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    loop_count++;

#if DEBUG_MONITOR
    //Serial.printf("Packet queued\r\n");
#endif
  }
  /* Next TX is scheduled after TX_COMPLETE event. */
}

/* ======================== START of SETUP() ======================== */
/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void setup() 
{
#if DEBUG_MONITOR
  Serial.begin(115200);
  Serial.printf("Starting...\r\n");
#endif

  /* Set the initial configuration for the power pin */
  pinMode(POWER_PIN, OUTPUT);
  BoardPowerOn();
  delay(1000); /* Let the sensors get stable */

  oneWireSensors.begin();
  
#if DEBUG_MONITOR
  oneWireScan();
#endif

#if ESP32_SLEEP_ENABLED

#if DEBUG_MONITOR
  Esp32WakeupReasonPrint();
#endif

  /* We set our ESP32 to wake up every ESP32_SLEEP_TIME_SEC seconds */
  esp_sleep_enable_timer_wakeup(ESP32_SLEEP_TIME_SEC * ESP32_SLEEP_uS_TO_S_FACTOR);
  
#if DEBUG_MONITOR
  float sleep_minutes = ESP32_SLEEP_TIME_SEC / 60;
  Serial.println("");
  Serial.println("Setting up the ESP32 to sleep for " 
                 + String(ESP32_SLEEP_TIME_SEC) 
                 +" seconds, which equals " 
                 + String(sleep_minutes) 
                 +" minutes.");
#endif

#endif /* #if ESP32_SLEEP_ENABLED */

#if DISPLAY_USE
  DisplayInitialize(); /* Must be before bme.begin() */
#endif

  ads.begin();
  /* If some of the channels repeatedly return the same number, it can 
   * be you have to rise the conversion delay. Standard is 8 ms: 
   * '#define ADS1115_CONVERSIONDELAY (8)'. Try out 16, 32 or maybe even 
   * 64 ms: '#define ADS1115_CONVERSIONDELAY (16)'. You will find the 
   * setting in the header file of the library: 'Adafruit_ADS1015.h'. */

  bool bme_status;
  bme_status = bme.begin();
  if (!bme_status) {
#if DEBUG_MONITOR
    Serial.println("Could not find a valid BME280 sensor. Please check wiring.");
#endif
    //while (1);
  }
  
  /* LMIC init */
  os_init();

  /* Reset the MAC state. Session and pending data transfers will be discarded. */
  LMIC_reset();

  /* Set static session parameters. Instead of dynamically establishing 
   * a session by joining the network, precomputed session parameters 
   * are be provided. */
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  /* Select frequencies range */
  //LMIC_selectSubBand(1); /* This is by purpose commented out. */
  
  /* Disable link check validation */
  LMIC_setLinkCheckMode(0);
  
  /* TTN uses SF9 for its RX2 window. */
  LMIC.dn2Dr = DR_SF9;
  
  /* Set data rate and transmit power for uplink 
   * (note: txpow seems to be ignored by the library). */
  LMIC_setDrTxpow(DR_SF7,14);
  
#if DEBUG_MONITOR
  Serial.println("");
  Serial.printf("LMIC setup done!\r\n");
#endif

  ValuesRead();

#if DEBUG_MONITOR
  Serial.println("");
  HeaderPrint();
  //ValuesPrint();
#endif

#if DISPLAY_USE
  display.init();
  DisplayPrint();
#endif

  /* Start the Lora job. */
  lora_send(&sendjob);
}
/* ========================= END of SETUP() ========================= */

/* ========================= START of LOOP() ======================== */
/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void loop() 
{
  ValuesRead();
#if DEBUG_MONITOR
  if (loop_count%10==0) {
    HeaderPrint();
  }
  ValuesPrint();
#endif
#if DISPLAY_USE
  DisplayPrint();
#endif
  
  /* Make sure LMIC is ran too. */
  os_runloop_once();

#if ESP32_SLEEP_ENABLED
  Esp32DeepSleepActivate();
  Serial.println("This line will never be printed if deep sleep is activated and working.");
#endif

  loop_count++;
  delay(2000);
}
/* ========================== END of LOOP() ========================= */

/* ================== START of HELPER FUNCTIONS() =================== */

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void FillDataToBeSendViaLora()
{
  lpp.reset();
  lpp.addTemperature(7, bmeTemp);
  lpp.addRelativeHumidity(8, bmeHumi);
  lpp.addBarometricPressure(9, bmePres);
  lpp.addAnalogInput(22, kty81_210_value);
  lpp.addAnalogInput(23, lm35_value);
  lpp.addAnalogInput(24, ds18s20_nr_1_celcius);
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void ValuesRead()
{
  bmeTemp = bme.readTemperature();
  bmePres = bme.readPressure() / 100.0F;
  if(bmePresAtStart == 0 ) {
    bmePresAtStart = bmePres;
  }
  bmeHumi = bme.readHumidity();
  //bmeAlti = bme.readAltitude(bmePresAtStart);

  delay(200);
  oneWireSensors.requestTemperatures();
  delay(300);
  ds18s20_nr_1_celcius = oneWireSensors.getTempC(ds18s20_nr_1_id);

  lm35_value = Lm35Read(ADS_CHANNEL_0, ADS_CH0_VOLT_LIMIT);
  
  kty81_210_value = Kty81210Read(ADS_CHANNEL_1, ADS_CH1_VOLT_LIMIT);
}

/* Source: https://www.eevblog.com/forum/beginners/kty81-220-temp-sensor-meh!/msg1371075/?PHPSESSID=jm8heeqc4easvca9f7gbur4ai7#msg1371075 */
struct resistance_row {
  int temp;
  int r2;
};

/* Source: Page 7 of 15 of the datasheet on 
 * https://www.nxp.com/docs/en/data-sheet/KTY81_SER.pdf
 * No, not the "smartest" way in the sense of "cool", but the smartest 
 * in the sense of the simplest and most adaptable way, as the datasheet 
 * only delivers data in this way. This way makes it convinient modifiable.
 * More on: https://en.wikipedia.org/wiki/KISS_principle
 */
struct resistance_row kty81210_resistance_table[] = {
  {0, 1630},
  {10, 1772},
  {20, 1922},
  {25, 2000},
  {30, 2080},
  {40, 2245},
  {50, 2417},
  {60, 2597},
  {70, 2785},
  {80, 2980},
  {90, 3182},
  {100, 3392},
  {110, 3607},
  {120, 3817},
  {130, 3915},
  {-1,-1}
};

/* Source: https://www.eevblog.com/forum/beginners/kty81-220-temp-sensor-meh!/msg1371075/?PHPSESSID=jm8heeqc4easvca9f7gbur4ai7#msg1371075 */
float temperatureForResistance( float resistance ) {
  int index = -1;
  boolean notFound=true;
  while(notFound)  {
    index+=1;
    if( kty81210_resistance_table[index].temp == -1 ) {
      return -1.0; // ERROR
    }
    if( kty81210_resistance_table[index].r2 > resistance ) {
      notFound = false;     
    }
  }
  if( index < 1 ) {
    return -1.0; // ERROR
  }
  int lowerBandT = kty81210_resistance_table[index-1].temp;
  int upperBandT = kty81210_resistance_table[index].temp;
  int lowerBandR = kty81210_resistance_table[index-1].r2;
  int upperBandR = kty81210_resistance_table[index].r2;
  int bandwidthT = upperBandT -lowerBandT;
  int bandwidthR = upperBandR -lowerBandR;
  int rDelta = resistance -lowerBandR;
  float temp = lowerBandT +(bandwidthT/(float)bandwidthR) *rDelta;
  return temp;
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
float Kty81210Read(int adsChannel, float voltLimit)
{
  float analog_read_volt = AdsVoltRead(adsChannel, voltLimit);

  /*Calculate the resistance of the KTY81 based on the voltage. - Sensor Adapter Switchboard */
  float sensor_r2_kty81210_resistance = ( analog_read_volt *(kty81210_module_r1 +kty81210_module_kty81210_rating_at_25_degrees_celcius) ) / kty81210_module_vdd;

  /* Correct for the KTY81 being off ideal values. - Sensor Adapter Switchboard */
  float sensor_r2_kty81210_resistance_calibrated = sensor_r2_kty81210_resistance + kty81210_module_calibration_value;

  /* Calculate the temperature in Celcius based on the resistance of the KTY81. */
  /* - Celcuis: */
  float kty81210_temperature_celcius = temperatureForResistance(sensor_r2_kty81210_resistance_calibrated);

  return kty81210_temperature_celcius;
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
float Lm35Read(int adsChannel, float voltLimit)
{
  float analog_read_volt = AdsVoltRead(adsChannel, voltLimit);
  return analog_read_volt *100;
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
float AdsVoltRead(int adsChannel, float adsChannelVoltLimit)
{
  int16_t analog_read_value = ads.readADC_SingleEnded(adsChannel);
  float analog_read_volt = (analog_read_value / ADS_READ_LIMIT ) * adsChannelVoltLimit;
  return analog_read_volt;
}

int AnalogReadAverage(int pinToRead, byte numberOfSamples)
{
  unsigned int runningValue = 0; 
  for(int x = 0 ; x < numberOfSamples ; x++) {
    runningValue = runningValue + analogRead(pinToRead);
  }
  runningValue = runningValue / numberOfSamples;
  return(runningValue);  
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x -in_min) *(out_max -out_min) /(in_max -in_min) +out_min;
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void BoardPowerOn()
{
  digitalWrite(POWER_PIN, HIGH);
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void BoardPowerOff()
{
  digitalWrite(POWER_PIN, LOW);
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void BoardPowerStatus()
{
  digitalRead(POWER_PIN);
}

#if DEBUG_MONITOR
/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void ColumnAdd(int i, float value) {
  if (value>=0) {
    Serial.print(" ");
  }
  while(i) {
    if ( value< (pow(10,i)) ) {
      Serial.print(" ");
    }
    i--;
  }
  Serial.print(value);
  Serial.print(";");
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void HeaderPrint()
{
  Serial.print(" Air Temp.;");
  Serial.print(" Air Humi.;");
  Serial.print(" Air Pres.;");
  Serial.print(" KTY81-210;");
  Serial.print("      LM35;");
  Serial.print("   DS18S20;");
  Serial.println("");

  Serial.print("      / °C;");
  Serial.print("       / %;");
  Serial.print("     / hPa;");
  Serial.print(" Temp./ °C;");
  Serial.print(" Temp./ °C;");
  Serial.print(" Temp./ °C;");
  Serial.println("");
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void ValuesPrint()
{
  ColumnAdd(5, bmeTemp);
  ColumnAdd(5, bmeHumi);
  ColumnAdd(5, bmePres);
  ColumnAdd(5, kty81_210_value);
  ColumnAdd(5, lm35_value);
  ColumnAdd(5, ds18s20_nr_1_celcius);
  Serial.println("");
}

void oneWireScan()
{
  Serial.println("");
  Serial.println("Starting the search for 1-wire addresses.");
  byte i;
  byte addr[8];

  if ( !oneWire.search(addr)) {
    oneWire.reset_search();
    delay(1000);
    return;
  }
  if (OneWire::crc8( addr, 7) != addr[7]) { 
      Serial.print("CRC is not valid!\n");
      return; 
  }

  Serial.print("Address found:");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    if ( addr[i]<16) {
      Serial.print("0");
    }
    Serial.print(addr[i], HEX);
  }
  Serial.println("");
  Serial.println("Ended search for 1-wire addresses.");
  Serial.println();
}
#endif /* #if DEBUG_MONITOR */

#if DISPLAY_USE
/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void DisplayInitialize()
{
    pinMode(DISPLAY_PIN_CONTROL, OUTPUT);
    /* Set DISPLAY_PIN_CONTROL low to reset OLED: */
    digitalWrite(DISPLAY_PIN_CONTROL, LOW);
    delay(50);
    digitalWrite(DISPLAY_PIN_CONTROL, HIGH);
    delay(50);
    display.init();
    display.setFont(ArialMT_Plain_10);
    delay(50);
    display.drawString( 0, 0, "Starting up ...");
    display.drawString( 0,20, "- and initializing...");
    display.display();
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void DisplayPrint()
{
  display.clear();

  prepend = DisplayPrePend(4, loop_count);
  display.drawString( 0,0, "Count:            : itoffice.eu");
  display.drawString((29+prepend),0, String(loop_count));

  prepend = DisplayPrePend(6, bmeTemp);
  display.drawString( 0,10, "TEMP:                          ");
  display.drawString((29+prepend),10, String(bmeTemp));

  prepend = DisplayPrePend(6, bmeHumi);
  display.drawString( 0,20, "HUMI:                          ");
  display.drawString((29+prepend),20, String(bmeHumi));

  display.display();
}

/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
int DisplayPrePend(int i, float value) {
  int display_prepend = 0;
  if (value>-0.0000001) {
    display_prepend = 6;
  }
  while(i) {
    if ( value< (pow(10,i)) ) {
      display_prepend += 6;
    }
    i--;
  }
  return display_prepend;
}
#endif /* #if DISPLAY_USE */

#if ESP32_SLEEP_ENABLED
/* Author: Andreas C. Dyhrberg, www.itoffice.eu */
void Esp32DeepSleepActivate()
{
  /*
  Next we decide what all peripherals to shut down/keep on. By default, 
  ESP32 will automatically power down the peripherals not needed by the 
  wakeup source, but if you want to be a power-user this is for you. 
  Read in detail at the API docs 
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented out as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep. */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /* In the case that no wake up sources were provided but deep sleep 
   * is started, it will sleep forever unless hardware reset occurs. */
#if DEBUG_MONITOR
  Serial.println("Going to sleep now");
  Serial.flush();
#endif
  esp_deep_sleep_start();

  /* If one or more of the following libraries are used, please add the 
  corresponding function: */
  //esp_bluedroid_disable();
  //esp_bt_controller_disable();
  //esp_wifi_stop();

  /* As an alternative to deep sleep, can you use light sleep. */
  //int sleep_error_code = esp_light_sleep_start();
  //Serial.print("sleep_error_code = ");
  //Serial.println(sleep_error_code);
  /* Definitions for error constants (above called 'sleep_error_code').
   * Added here to inform about it:
   * #define ESP_OK                    0 //< Value indicating success (no error)
   * #define ESP_ERR_INVALID_STATE 0x103 //< Invalid state */
}
#endif /* #if ESP32_SLEEP_ENABLED */
