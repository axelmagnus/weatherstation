#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <WebOTA.h>
#include "esp_deep_sleep.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHTesp.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#define DHTPIN 27     // what digital pin DHT is connected to
#define buckboostPIN 33     // to turn off the buck-booster

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

//#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex

/******************************* WIFI **************************************/

#define WLAN_SSID       "MY WIFI 2C50"//"D410"//"2.4 MegaPlupp"//"MY WIFI 2C50"//"TN_private_4A7E34"//"Axels Moto"//"Axels UMI"//
#define WLAN_PASS       "MYWIFI5780"//"Axel2018"//"MYWIFI5780"//"Axel2018"//"azazaz13"//"9JWEKJYMC3T7W"//"MYWIFI5780"//"azazaz13"//"9JWEKJYMC3T7W"//
const char* host     = "ESP-OTA"; // Used for MDNS resolution
//const char* ssid     = WLAN_SSID;
//const char* password = WLAN_PASS ;

/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "axelmagnus"
#define AIO_KEY         "1a20315d078d4304bee799ce4b2af0e7"

RTC_DATA_ATTR int RTCnoshield;//keep track of noshield errors
RTC_DATA_ATTR int RTCwifitry;//keep track of noshield errors
RTC_DATA_ATTR int RTCcurrent;//keep track of current pub errors
RTC_DATA_ATTR int RTClvolt;//keep track of lvolt pub errors
RTC_DATA_ATTR int RTCtemp1;//keep track of temp1 pub errors
RTC_DATA_ATTR int RTChum1;//keep track of hum1 pub errors
RTC_DATA_ATTR int RTChic1;//keep track of hic1 pub errors
RTC_DATA_ATTR int RTCrain1;//keep track of rain1 pub errors
static RTC_DATA_ATTR struct timeval sleep_enter_time;
RTC_DATA_ATTR int rainAcc;
RTC_DATA_ATTR long  time2sleep;

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);

// Setup feed for temp, hum, and heat index, hic for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>

Adafruit_MQTT_Publish current = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/current", MQTT_QOS_1);
Adafruit_MQTT_Publish current_ch = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/current_ch", MQTT_QOS_1);
Adafruit_MQTT_Publish lvolt = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lvolt1", MQTT_QOS_1);
Adafruit_MQTT_Publish temp1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp1", MQTT_QOS_1);
Adafruit_MQTT_Publish hum1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hum1", MQTT_QOS_1);
Adafruit_MQTT_Publish hic1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hic1", MQTT_QOS_1);
Adafruit_MQTT_Publish rain1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rain1", MQTT_QOS_1);

Adafruit_MQTT_Publish RTCcurrentp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RTCcurrent", MQTT_QOS_1);
Adafruit_MQTT_Publish RTClvoltp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RTClvolt", MQTT_QOS_1);
Adafruit_MQTT_Publish RTCtemp1p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RTCtemp1", MQTT_QOS_1);
Adafruit_MQTT_Publish RTChum1p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RTChum1", MQTT_QOS_1);
Adafruit_MQTT_Publish RTChic1p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RTChic1", MQTT_QOS_1);
Adafruit_MQTT_Publish RTCrain1p = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RTCrain1", MQTT_QOS_1);
Adafruit_MQTT_Publish RTCnoshieldp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RTCnoshield1 ", MQTT_QOS_1);
Adafruit_MQTT_Publish RTCwifitryp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/RTCwifitry", MQTT_QOS_1);
Adafruit_MQTT_Publish battLevelp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/intbattery", MQTT_QOS_1);

int pubState = 0; //to check that all data are sent

/*************************** Error Reporting *********************************/

Adafruit_MQTT_Subscribe errors = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/errors");
Adafruit_MQTT_Subscribe throttle = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/throttle");

Adafruit_INA219 ina219;
DHTesp dht;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float current_mA_ch = 0;
float loadvoltage = 0;
float battLevel;
int filterlen = 10;

const long wakeup_time_usec = 600 * 1000000; //200 seconds sleep time
const long DSoffset = 148 * 1000; //sligth extra time adjusted for DS

void setup(void)
{
  if (time2sleep == 0) time2sleep = wakeup_time_usec; //first time its not set

  Serial.begin(115200);

  Serial.println("");
  pinMode(13, OUTPUT);
  pinMode(buckboostPIN, OUTPUT);
  struct timeval now;
  gettimeofday(&now, NULL);
  //Below: calc the time it was asleep before woken up
  long sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;
  Serial.print("sleep_time_ms: ");
  Serial.println(sleep_time_ms);
  Serial.print("time2sleep: us");
  Serial.println(time2sleep);
  Serial.println("Enabling EXT0 wakeup on pins GPIO12");
  esp_deep_sleep_enable_ext0_wakeup(GPIO_NUM_12, 1); //1 = High, 0 = Low

  esp_deep_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_deep_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) { //rain gauge trigger; just increase rain accumulated and go to sleep again, adjusted time
    Serial.println("Wake up from GPIO; rain.\n It has rained another 0.2794 mm\n");//https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly..pdf?_ga=2.108192194.1741446430.1558188952-1566586438.1554723270
    rainAcc++;
    time2sleep = time2sleep - sleep_time_ms * 1000L; //Set new DS time, reduced by the time it was asleep before rain trigger
    digitalWrite(13, 1);
    Serial.print("Enabling timer wakeup: ");
    Serial.println(time2sleep - DSoffset);
    esp_deep_sleep_enable_timer_wakeup(abs(time2sleep - DSoffset));
    Serial.println("Entering deep sleep after rain trigger");
    gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
    digitalWrite(13, 0);
    esp_deep_sleep_start();
  }
  // Serial.setDebugOutput(true);
  time2sleep = wakeup_time_usec; //reset sleep time
  esp_deep_sleep_enable_timer_wakeup(time2sleep);//10 minutes between send readings wake ups
  //esp_deep_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1); //1 = High, 0 = Low

  uint32_t currentFrequency;

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
  for (int i = 0; i < filterlen; i++) {
    shuntvoltage += ina219.getShuntVoltage_mV();
    busvoltage += ina219.getBusVoltage_V();
    current_mA += ina219.getCurrent_mA();

    battLevel += (analogRead(A13) / 4096.0) * 2 * 3.3 * 1.1;
  }
  shuntvoltage /= filterlen;
  busvoltage /= filterlen;
  current_mA /= filterlen;
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  battLevel /= filterlen;

  if ((loadvoltage > 3.1 || battLevel < 3.2) && (battLevel < 4.15 || loadvoltage > battLevel)) {
    digitalWrite(buckboostPIN, 1); //turn on the buck-boost if we need to charge internal battery)
    delay(500);

    for (int i = 0; i < filterlen; i++) {
      current_mA_ch += ina219.getCurrent_mA();//How much we are now charging the int battery
    }
    current_mA_ch /= filterlen;
  }
  delay(200);
  dht.setup(DHTPIN, DHTesp::DHT22);

  Serial.print(F("Connecting to "));
  Serial.println(WLAN_SSID);
  int wifitry = 0;
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(13, 1);
    delay(200);
    wifitry++;
    Serial.print(WiFi.status());
    digitalWrite(13, 0);
    delay(200);
    if (WiFi.status() == 255 && wifitry > 4) { //didnt detect wifi board, sleep for 10 sec
      RTCnoshield++;
      digitalWrite(13, 0);
      delay(50);
      digitalWrite(13, 1);
      delay(50);
      digitalWrite(13, 0);
      delay(50);
      digitalWrite(13, 1);
      delay(50);
      digitalWrite(13, 0);
      delay(50);
      digitalWrite(13, 1);
      delay(50);
      esp_deep_sleep_enable_timer_wakeup(10 * 1000000);//10 sec for reconnecting wifi
      gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
      esp_deep_sleep_start();
    }
    if (wifitry > 10) {
      RTCwifitry++;
      esp_deep_sleep_enable_timer_wakeup(3 * 1000000);//3 sec for reconnecting wifi
      gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
      esp_deep_sleep_start();
    }
  }
  Serial.println("Wifi connected!");//
  init_mdns(host);
  Serial.println("Measuring voltage and current with INA219 ...");
  // Setup MQTT subscriptions for throttle & error messages
  mqtt.subscribe(&throttle);
  mqtt.subscribe(&errors);
}

void loop(void)
{
  webota.handle();
  MQTT_connect();

  float h = dht.getHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.getTemperature();
  float hict = dht.computeHeatIndex(t, h, false);
  Serial.print("Temp: "); Serial.print(t); Serial.print(" C \t");
  Serial.print("Hic: "); Serial.print(hict); Serial.print(" C \t");
  Serial.print("Hum:   "); Serial.print(h); Serial.print(" % \t");


  /*
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.print("V \t");
    Serial.print("Shunt Voltage:   "); Serial.print(shuntvoltage); Serial.print("mV \t");
  */
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.print("V \t");
  Serial.print("Internal Battery  "); Serial.print(battLevel); Serial.print("V \t");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Current charge:       "); Serial.print(current_mA_ch); Serial.println(" mA");
  Serial.print("RTC current:  "); Serial.print(RTCcurrent); Serial.print(" ");
  Serial.print("RTC lvolt:       "); Serial.print(RTClvolt); Serial.println(" ");
  Serial.print("RTC noshield:  "); Serial.print(RTCnoshield); Serial.print(" ");
  Serial.print("RTC WiFi try:       "); Serial.print(RTCwifitry); Serial.println(" ");


  if (! current.publish(current_mA, 1)) {
    Serial.println(F("Publish Current failed."));
    RTCcurrent++;
  } if (! current_ch.publish(current_mA_ch, 1)) {
    Serial.println(F("Publish Current failed."));
    RTCcurrent++;
  }
  if (! lvolt.publish(loadvoltage, 2)) {
    Serial.println(F("Publish volt Failed."));
    RTClvolt++;
  }
  if (! battLevelp.publish(battLevel, 2)) {
    Serial.println(F("Publish volt Failed."));
    RTClvolt++;
  }
  if (! temp1.publish(t, 1)) {
    Serial.println(F("Publish temp1 Failed."));
    RTCtemp1++;
  }
  if (! hum1.publish(h, 1)) {
    Serial.println(F("Publish hum1 Failed."));
    RTChum1++;
  }
  if (! hic1.publish(hict, 1)) {
    Serial.println(F("Publish hic1 Failed."));
    RTChic1++;
  }
  if (! rain1.publish(rainAcc * 0.2794, 1)) { //0.2794 mm per trigger //https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly..pdf?_ga=2.108192194.1741446430.1558188952-1566586438.1554723270
    Serial.println(F("Publish rain1 Failed."));
    RTCrain1++;
  } else {
    rainAcc = 0; //reset rain accumulator when published
  }
  if (! RTChic1p.publish(RTChic1, 0)) {
    Serial.println(F("Publish RTChic1 Failed."));
  }
  if (! RTChum1p.publish(RTChum1, 0)) {
    Serial.println(F("Publish RTChum1 Failed."));
  }
  if (! RTCtemp1p.publish(RTCtemp1, 0)) {
    Serial.println(F("Publish RTCtemp1 Failed."));
  }
  if (! RTCcurrentp.publish(RTCcurrent, 0)) {
    Serial.println(F("Publish RTChic1 Failed."));
  }
  if (! RTClvoltp.publish(RTClvolt, 0)) {
    Serial.println(F("Publish RTClvolt Failed."));
  }
  if (! RTCnoshieldp.publish(RTCnoshield, 0)) {
    Serial.println(F("Publish RTChic1 Failed."));
  }
  if (! RTCwifitryp.publish(RTCwifitry, 0)) {
    Serial.println(F("Publish RTClvolt Failed."));
  }

  Serial.println(F("Publish Success x 5!"));
  /*    No subs right now
         Adafruit_MQTT_Subscribe *subscription;
      while ((subscription = mqtt.readSubscription(3000))) {
        if (subscription == &errors) {
          Serial.print(F("ERROR: "));
          Serial.println((char *)errors.lastread);
        } else if (subscription == &throttle) {
          Serial.print(F("THROTTLE: "));
          Serial.println((char *)throttle.lastread);
        }
      }
  */
  //
  digitalWrite(13, 1);
  webota.delay(100);
  digitalWrite(13, 0);
  webota.delay(50);
  //delay(1000);
  digitalWrite(13, 1);
  webota.delay(300);
  digitalWrite(13, 0);
  if (loadvoltage > 3.2 && battLevel < 4.15) {
    Serial.println("Charging internal battery");
    digitalWrite(13, 1);
    webota.delay(300);
    digitalWrite(13, 0);
    webota.delay(4000);//to allow for charging via buck-booster
    digitalWrite(13, 1);
    webota.delay(300);
    digitalWrite(13, 0);
    webota.delay(4000);//to allow for charging via buck-booster

    if (loadvoltage > battLevel) {//Higher voltage on solar battery, charge some more
      Serial.println("Higher voltage on solar battery, charge some more");
      webota.delay(5000);//to allow for charging via buck-booster
      digitalWrite(13, 1);
      webota.delay(300);
      digitalWrite(13, 0);
      webota.delay(200);
      digitalWrite(13, 1);
      webota.delay(300);
      digitalWrite(13, 0);
      webota.delay(5000);//to allow for charging via buck-booster
    }
  }
  Serial.println("Setup ESP32 to sleep for every " + String(10) + " minutes");

  digitalWrite(buckboostPIN, 0); //turn off the buck-booster
  digitalWrite(13, 1);
  webota.delay(300);
  digitalWrite(13, 0);
  gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep

  esp_deep_sleep_start();
  //if published

  //delay(2000); // to avoid throttle on MQTT
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print(F("Connecting to MQTT... "));

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println(F("Retrying MQTT connection in 5 seconds..."));
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      gettimeofday(&sleep_enter_time, NULL);//store on RTC at what time you enter sleep
      esp_deep_sleep_start();
    }
  }
  Serial.println(F("MQTT Connected!"));
}
