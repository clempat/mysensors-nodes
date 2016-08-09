/**
 * This sketch is done for the Temperature/humidity sensors DHT22 for the heating
 * v: 1470657545
 */

#define MY_DEBUG
#define MY_NODE_ID 104
#define HEART_BEAT_CYCLE 6   // 5min * 6 = every 30min
#define BATTERY_REPORT_CYCLE 288 // 5min * 288 = every 24h
#define SENSOR_RETRY 5
#define SLEEP_TIME 300000 // 5 * 60 * 1000 = 30000  sleep time between read (in milliseconds)

#define MY_RADIO_NRF24

#include <SPI.h>
#include <MySensors.h>
#include <DHT.h>

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_TRIP 2

#define HUMIDITY_SENSOR_DIGITAL_PIN 3
#define HUMI_TRANSMIT_THRESHOLD 3.0  // THRESHOLD tells how much the value should have changed since last time it was transmitted.
#define TEMP_TRANSMIT_THRESHOLD 0.3

// Battery Calculation
// 1M, 470K divider across battery and using internal ADC ref of 1.1V
// Sense point is bypassed with 0.1 uF cap to reduce noise at that point
// ((1e6+470e3)/470e3)*1.1 = Vmax = 3.44 Volts
// 3.44/1023 = Volts per bit = 0.003363075
#define VBAT_PER_BITS 0.003363075
#define VMIN 1.9  // Battery monitor lower level. Vmin_radio=1.9V
#define VMAX 3.3  //  " " " high level. Vmin<Vmax<=3.44

int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point

int batteryReportCounter = BATTERY_REPORT_CYCLE - 1;
int heartBeatCounter= HEART_BEAT_CYCLE - 1;
int sensorCounter = 0;

// DHT
DHT dht(HUMIDITY_SENSOR_DIGITAL_PIN, DHT22);
float lastTemp = -100;
int lastHum = -100;
int oldBatteryPcnt = 110;
int oldTrippedValue = 0;

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage alive(CHILD_ID_TRIP, V_TRIPPED);

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

void presentation()
{
  // Send the sketch version information to the gateway and Controller
   sendSketchInfo("Humidity Battery", "1.4");

    // Register all sensors to gw (they will be created as child devices)
    present(CHILD_ID_HUM, S_HUM);
    present(CHILD_ID_TEMP, S_TEMP);
    present(CHILD_ID_TRIP, S_MOTION);
}

void loop()
{
  batteryReportCounter++;
  heartBeatCounter++;

  if (heartBeatCounter >= HEART_BEAT_CYCLE) {
    int trippedValue = abs(oldTrippedValue - 1);
    send(alive.set(trippedValue));
    oldTrippedValue = trippedValue;
    heartBeatCounter = 0;
  }

  readSensor();

  // Check battery
  if (batteryReportCounter >= BATTERY_REPORT_CYCLE) {
    // get the battery Voltage
    int sensorValue = analogRead(BATTERY_SENSE_PIN);
    float batteryV  = sensorValue * VBAT_PER_BITS;
    int batteryPcnt = static_cast<int>(((batteryV-VMIN)/(VMAX-VMIN))*100);

    #ifdef MY_DEBUG
    Serial.print("Battery Voltage: ");
    Serial.print(batteryV);
    Serial.println(" V");

    Serial.print("Battery percent: ");
    Serial.print(batteryPcnt);
    Serial.println(" %");
    #endif

    if (batteryV < VMIN) {
      #ifdef MY_DEBUG
        Serial.println("Battery percent sent: 0%");
      #endif
      sendBatteryLevel(0);

      batteryReportCounter = 0;
    } else if (oldBatteryPcnt > batteryPcnt) {
      // Power up radio after sleep
      sendBatteryLevel(batteryPcnt);
      oldBatteryPcnt = batteryPcnt;

      batteryReportCounter = 0;
    }
  }

  sleep(SLEEP_TIME);
}

void readSensor()
{
  sensorCounter++;
  wait(2000);
  float temp = dht.readTemperature();
  int hum = dht.readHumidity();

  if (isnan(temp) | isnan(hum)) {
    Serial.println("Failed reading from DHT");
    if (sensorCounter - 1 < SENSOR_RETRY) {
      Serial.print("RETRY ");
      Serial.println(sensorCounter - 1);
      readSensor();
    }
   return;
  }

  #ifdef MY_DEBUG
    Serial.print("T: ");
    Serial.println(temp);
  #endif

  #ifdef MY_DEBUG
    Serial.print("H: ");
    Serial.println(hum);
  #endif

  sendData(hum, temp);
}

void sendData(int hum, float temp)
{
  float diffTemp = abs(lastTemp - temp);
  if (diffTemp > TEMP_TRANSMIT_THRESHOLD) {
    send(msgTemp.set(temp, 1));
    lastTemp = temp;
    #ifdef MY_DEBUG
      Serial.println('Temperature sent');
    #endif
  }
  float diffHum = abs(lastHum - hum);
  if (diffTemp > HUMI_TRANSMIT_THRESHOLD) {
    send(msgHum.set(hum, 1));
    lastHum = hum;
    #ifdef MY_DEBUG
      Serial.println('Humidity sent');
    #endif
  }

}
