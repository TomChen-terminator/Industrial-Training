#include "MPU9250.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

const char* ssid = "AH CHEH-2.4GHz@unifi";
const char* password = "0126901246";
const char* mqtt_server = "192.168.0.144";

WiFiClient espClient;
PubSubClient client(espClient);

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

const float accelBias[3] = { 0.15, 0.1, 0.15 }; // Obtained through calibration scketch
const float accelFactor[3] = { 1.0, 1.0, 1.0 }; // Obtained through calibration scketch

const float MagBias[3] = { 19.055, 42.915, -12.6 }; // Obtained through calibration scketch
const float MagFactor[3] = { 1, 1, 1.133 };

float startMillis;  //some global variables available anywhere in the program
float currentMillis;

float timetaken;

float ax;
float ay;
float az;
float gx;
float gy;
float gz;
float mx;
float my;
float mz;
float t;

String combine;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
 while (!client.connect("nodemcu1")) {
   Serial.print(".");
 }
}

void setup() {
  // serial to display data
  Serial.begin(115200);
  setup_wifi();
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  
  IMU.setAccelCalX(accelBias[0], accelFactor[0]);
  IMU.setAccelCalY(accelBias[1], accelFactor[1]);
  IMU.setAccelCalZ(accelBias[2], accelFactor[2]);

  IMU.setMagCalX(MagBias[0], MagFactor[0]);
  IMU.setMagCalY(MagBias[1], MagFactor[1]);
  IMU.setMagCalZ(MagBias[2], MagFactor[2]);

  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  client.setServer(mqtt_server, 1883);
  
  startMillis = millis();
}

void loop() {
  // read the sensor
  currentMillis = millis();

  timetaken = (currentMillis - startMillis)*0.001;

  if (timetaken > 0.02){

  if (!client.connected()) {
    reconnect();
  }

  client.loop();
  
  IMU.readSensor();
  // display the data

  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();

  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();

  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();

  t = IMU.getTemperature_C();

  combine = "";
  combine.concat(ax);
  combine.concat(",");
  combine.concat(ay);
  combine.concat(",");
  combine.concat(az);
  combine.concat(",");
  combine.concat(gx);
  combine.concat(",");
  combine.concat(gy);
  combine.concat(",");
  combine.concat(gz);
  combine.concat(",");
  combine.concat(mx);
  combine.concat(",");
  combine.concat(my);
  combine.concat(",");
  combine.concat(mz);
  combine.concat(",");
  combine.concat(t);

  int str_len = combine.length() + 1; 

  char char_array[str_len];

  combine.toCharArray(char_array, str_len);

  client.publish("imu", char_array);

//  Serial.println(combine);
  
  startMillis = currentMillis;
  }
} 
