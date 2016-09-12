/**
 * This sketch is done for the Temperature/humidity sensors DHT22 for the heating
 * v: 1470657545
 */

#define MY_NODE_ID 104

// #define MY_DEBUG
// #define VERBOSE

#define HEART_BEAT_CYCLE 6   // 5min * 6 = every 30min
#define BATTERY_REPORT_CYCLE 288 // 5min * 288 = every 24h
#define SENSOR_RETRY 5
#define RADIO_RETRY 5

#ifdef MY_DEBUG
  #define SLEEP_TIME 10000 // 10 * 1000 = 10000  sleep time between read (in milliseconds)
  #define HUMI_TRANSMIT_THRESHOLD 0  // THRESHOLD tells how much the value should have changed since last time it was transmitted.
  #define TEMP_TRANSMIT_THRESHOLD 0
  #define VERBOSE
#else
  #define SLEEP_TIME 300000 // 5 * 60 * 1000 = 30000  sleep time between read (in milliseconds)
  #define HUMI_TRANSMIT_THRESHOLD 3.0  // THRESHOLD tells how much the value should have changed since last time it was transmitted.
  #define TEMP_TRANSMIT_THRESHOLD 0.3
#endif

#define MY_RADIO_NRF24

#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_TRIP 2

#define HUMIDITY_SENSOR_DIGITAL_PIN 3

// Battery Calculation
// 1M, 470K divider across battery and using internal ADC ref of 1.1V
// Sense point is bypassed with 0.1 uF cap to reduce noise at that point
// ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
// 3.44/1023 = Volts per bit = 0.003363075
#define VBAT_PER_BITS 0.003363075
#define VMIN 1.9  // Battery monitor lower level. Vmin_radio=1.9V
#define VMAX 3.3  //  " " " high level. Vmin<Vmax<=3.44

int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point

int batteryReportCounter = BATTERY_REPORT_CYCLE ;
int heartBeatCounter = HEART_BEAT_CYCLE;

// DHT
DHT dht(HUMIDITY_SENSOR_DIGITAL_PIN, DHT22);
float lastTemp = -100;
int lastHum = -100;
int oldBatteryPcnt = 110;
int lastHeartBeat = true;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage alive(CHILD_ID_TRIP, V_VAR1);

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Humidity Battery", "1.5");

    // Register all sensors to gw (they will be created as child devices)
    present(CHILD_ID_HUM, S_HUM);
    wait(200);
    present(CHILD_ID_TEMP, S_TEMP);
    wait(200);
    present(CHILD_ID_TRIP, S_CUSTOM);
    wait(200);
}

void setup()
{
  // use the 1.1 V internal reference
  #if defined(__AVR_ATmega2560__)
     analogReference(INTERNAL1V1);
  #else
     analogReference(INTERNAL);
  #endif
  dht.begin();
}

void loop()
{

  if (heartBeatCounter++ >= (HEART_BEAT_CYCLE - 1)) {
    _send(alive.set(lastHeartBeat? false : true));
  }

  readSensor();

  // Check battery
  if (batteryReportCounter++ >= (BATTERY_REPORT_CYCLE - 1)) {
    readBattery();
  }

  sleep(SLEEP_TIME);
}

void readSensor()
{

  float temp = NAN;
  int hum = NAN;
  int sensorCounter = 0;
  while((isnan(temp) || isnan(hum)) && (sensorCounter++ < SENSOR_RETRY)) {
    #ifdef VERBOSE
      Serial.println("Reading...");
    #endif
    temp = dht.readTemperature();
    hum = dht.readHumidity();
    if (isnan(temp) || isnan(hum)) {
      Serial.println("Failed reading from DHT");
      if (sensorCounter < SENSOR_RETRY) {
        Serial.print("RETRY ");
        Serial.println(sensorCounter - 1);
      }
    }
    wait(200);
  }

  #ifdef VERBOSE
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println("°C");
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.println("%");
  #endif

  float diffTemp = abs(lastTemp - temp);
  int diffHum = abs(lastHum - hum);

  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) {
    _send(msgTemp.set(temp, 1));
  }

  if (diffHum > HUMI_TRANSMIT_THRESHOLD) {
    _send(msgHum.set(hum, 1));
  }
}

void readBattery() {
  // get the battery Voltage
  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  float batteryV  = sensorValue * VBAT_PER_BITS;
  int batteryPcnt = static_cast<int>(((batteryV-VMIN)/(VMAX-VMIN))*100);

  #ifdef VERBOSE
  Serial.print("Battery Voltage: ");
  Serial.print(batteryV);
  Serial.println(" V");

  Serial.print("Battery percent: ");
  Serial.print(batteryPcnt);
  Serial.println(" %");
  #endif

  if (batteryV < VMIN) {
    #ifdef VERBOSE
      Serial.println("Battery percent sent: 0%");
    #endif
    sendBatteryLevel(0);
    wait(100);

    batteryReportCounter = 0;
  } else {
    // Power up radio after sleep
    sendBatteryLevel(batteryPcnt);
    wait(100);
    oldBatteryPcnt = batteryPcnt;

    batteryReportCounter = 0;
  }
}

bool _send(MyMessage msg) {
  bool sent = false;
  int retry = 0;

  while(!(sent = send(msg) && (retry++ < RADIO_RETRY))) {
    #ifdef VERBOSE
      Serial.print("Sending ");
      Serial.print(retry);
      Serial.print(" on ");
      Serial.print(RADIO_RETRY);
      Serial.println("...")
    #endif
    if (isTransportReady()) {
      wait(200);
    } else {
      wait(5000);
    }
  }

  if (sent == true) {
    switch (msg.type) {
      case V_HUM:
        lastHum = msg.getFloat();
        #ifdef VERBOSE
          Serial.print(lastHum);
          Serial.println("% saved");
          Serial.println("Humidity sent");
        #endif
        break;
      case V_TEMP:
        lastTemp = msg.getFloat();
        #ifdef VERBOSE
          Serial.print(lastTemp);
          Serial.println("°C saved");
          Serial.println("Temperature sent");
        #endif
        break;
      case V_VAR1:
        lastHeartBeat = msg.getBool();
        heartBeatCounter = 0;
        #ifdef VERBOSE
          Serial.print(lastHeartBeat);
          Serial.println(" saved");
          Serial.println("HeartBeat sent");
        #endif
        break;
    }
  }

  return sent;
}
