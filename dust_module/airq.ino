#include "arduino_secrets.h"
/*
//Arduino secrets for passwords
// do not forget to add it to gitignore
const char* ssid = "<wifi ssid>";       
const char* pass = "<wifi password>"; 
*/
#include <Wire.h>
#include <SPI.h>
#include <string.h>
#include <WiFiNINA.h>         
#include <WiFiUdp.h>
#include "Adafruit_SGP40.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "DFRobot_AirQualitySensor.h"        

/*Device Config*/
#define VERSION_STR F("v.1.1.0")
#define LED_WARNING 12

/* Safety constraints*/
#define MAX_CO2 2000
#define MAX_TVOC 400
#define MIN_HUM 25.0
#define MAX_PM1 350
#define MAX_PM2 150
#define MAX_PM10 355
#define MAX_AQI 200

#define AVG_CO2 1200
#define AVG_TVOC 221
#define AVG_HUM 30
#define AVG_PM1 50
#define AVG_PM2 50
#define AVG_PM10 50
#define AVG_AQI 50
#define AQI_LEVELS 6
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_SCK 5
#define BME_SDO 4
#define BME_SDI 3
#define BME_CS 2
#define DUST_ADDRESS    0x19
#define LOCAL_UDP_PORT 2389
#define RECONNECT_AFTER 5
#define WAIT_DELAY_MS 1000
#define MEASURE_LOOP_MS 2000

/* AQI Tables*/
float a_low[7] =      {  0.0, 51.0,  101.0, 151.0, 201.0, 301.0};
float a_high[7] =      {50.0, 100.0, 150.0, 200.0, 300.0, 500.0};
float pm2_c_low[7] =   { 0.0,  12.1, 35.5, 55.5,  150.5, 250.5};
float pm2_c_high[7] =   {12.0, 35.4, 55.4, 150.4, 250.4, 500.4};
float pm10_c_low[7] =  { 0.0,  55.0, 155.0, 255.0,  355.0, 425.0};
float pm10_c_high[7] =  {54.0, 154.0, 254.0, 354.0, 424.0, 604.0};

/* Network data and structs */     
const char* GATE_MESH = "192.168.4.1";
const int GATE_PORT = 2390;
int wifi_status = WL_IDLE_STATUS;
WiFiUDP Udp;
int cnt = 0;

/* Hardware structs*/
/* VOC sensor Init
VCC -> 3.3V
GND -> GND
SCL -> analog 5
CDA -> analog 4
*/
Adafruit_SGP40 sgp;
Adafruit_BME280 bme(BME_CS, BME_SDI, BME_SDO, BME_SCK);
DFRobot_AirQualitySensor dust_sensor(&Wire, DUST_ADDRESS);

/* AIRq data and buffers*/
struct air_data {
  char level;
  float temp;
  float pressure;
  float humidity;
  uint16_t aqi;
  uint16_t tvoc;
  uint16_t pm1;
  uint16_t pm2;
  uint16_t pm10;
};
air_data prev_value, data;
byte udp_buf[sizeof(air_data)];

/*
Output flag:
D - Dainger
A - Average
S - Safe
*/
char info_flag;
bool init_done = false;
char network_flag;

float aqi_level(float ih, float il, float ch, float cl, float value) {
  return ((ih-il) / (ch-cl)) * (value - cl) + il;
}

float calc_aqi_level(float value, float c_low[7], float c_high[7]) {
  for(int i = 0; i < AQI_LEVELS; i++){
    if(value <= c_high[i]){
      return aqi_level(a_high[i], a_low[i], c_high[i], c_low[i], value);      
    }
  }
  return 600.0;
}

/* AQI calculation entry point*/
uint16_t aqi(air_data data) {
    float pm25_aqi = uint16_t(calc_aqi_level(data.pm2, pm2_c_low, pm2_c_high));
    float pm10_aqi = uint16_t(calc_aqi_level(data.pm10, pm10_c_low, pm10_c_high));
    return uint16_t((pm25_aqi + pm10_aqi) / 2.0);
}

/* Sensor processing */
air_data read_sensors() {
  air_data result;
  result.temp = bme.readTemperature();
  result.humidity = bme.readHumidity();
  result.pressure = bme.readPressure() / 100.0F;
  result.tvoc = sgp.measureVocIndex(result.temp, result.humidity);
  result.pm1 = dust_sensor.gainParticleConcentration_ugm3(PARTICLE_PM1_0_STANDARD);
  result.pm2 = dust_sensor.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);
  result.pm10 = dust_sensor.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);
  result.aqi = aqi(result);
  
  // check safety levels
  safety_check(data);
  result.level = info_flag;
  return result;
}

/* Air quality checks */
void safety_check(air_data data) {
  bool warning = (data.tvoc > MAX_TVOC) || 
                 (data.humidity < MIN_HUM) ||
                 (data.pm1 > MAX_PM1) ||
                 (data.pm2 > MAX_PM2) ||
                 (data.pm10 > MAX_PM10 ) ||
                 (data.aqi > MAX_AQI);

  bool average = (data.tvoc >= AVG_TVOC && data.tvoc <= MAX_TVOC) ||
                 (data.humidity <= AVG_HUM && data.humidity >= MIN_HUM) ||
                 (data.pm1 >= AVG_PM1 && data.pm1 <= MAX_PM1) ||
                 (data.pm2 >= AVG_PM2 && data.pm2 <= MAX_PM2) ||
                 (data.pm10 >= AVG_PM10 && data.pm10 <= MAX_PM10) ||
                 (data.aqi >= AVG_AQI && data.pm10 <= MAX_AQI);

  digitalWrite(LED_WARNING, LOW);
  info_flag = 'S';

  if (warning) {
    info_flag = 'D';
    digitalWrite(LED_WARNING, HIGH);
  }
  else if (average){
    info_flag = 'A';
  }
}

/* Serial output for UI*/
char compare(int a, int b) {
  if (a == b) {
    return '-';
  }
  if (a > b) {
    return 92;
  }
  return '/';
}

/* Serial interface output */
void send_usb(air_data data) {
  /*
  To read usb data you can use minicom ubuntu tool
  # cat /dev/ttyACM0
  */
  Serial.print("Level       = ");
  Serial.println(info_flag);

  Serial.print("IoT status  = ");
  Serial.println(network_flag);

  char buf[32];
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "AQI         = %d %c", data.aqi, compare(prev_value.aqi, data.aqi));
  Serial.println(buf);

  memset(buf, 0, sizeof(buf));
  sprintf(buf, "Temperature = %d C %c", int(data.temp), compare(prev_value.temp, data.temp));
  Serial.println(buf);

  memset(buf, 0, sizeof(buf));
  sprintf(buf, "Humidity    = %d %% %c", int(data.humidity), compare(prev_value.humidity, data.humidity));
  Serial.println(buf);

  memset(buf, 0, sizeof(buf));
  sprintf(buf, "Pressure    = %d hPa", int(data.pressure));
  Serial.println(buf);

  memset(buf, 0, sizeof(buf));
  sprintf(buf, "PM1         = %d ug/m3 %c", data.pm1, compare(prev_value.pm1, data.pm1));
  Serial.println(buf);

  memset(buf, 0, sizeof(buf));
  sprintf(buf, "PM2.5       = %d ug/m3 %c", data.pm2, compare(prev_value.pm2, data.pm2));
  Serial.println(buf);

  memset(buf, 0, sizeof(buf));
  sprintf(buf, "PM10        = %d ug/m3 %c", data.pm10, compare(prev_value.pm10, data.pm10));
  Serial.println(buf);

  memset(buf, 0, sizeof(buf));
  sprintf(buf, "VOC         = %d ppm %c\n", data.tvoc, compare(prev_value.tvoc, data.tvoc));
  Serial.println(buf);
}

/* Sendors output transmission entry point */
void send(air_data data){
  send_usb(data);

  if(!airq_gate_connection()){
    return;
  }
  
  memset(udp_buf, 0, sizeof(udp_buf));

  byte * p = (byte*) &data;
  int sent = 0;
  for (sent = 0; sent < sizeof(data); sent++)
        udp_buf[sent] = *p++;

  Udp.beginPacket(GATE_MESH, GATE_PORT);
  Udp.write(udp_buf, sent);
  Udp.endPacket();
}

/* Hardware UI*/
void blink_led(int n){
  for(int i=0;i<n;i++){
    digitalWrite(LED_WARNING, HIGH);
    delay(200);
    digitalWrite(LED_WARNING, LOW);
    delay(200);
  }
}

/* Hardware init */
void setup() {
  pinMode(A7, OUTPUT);
  digitalWrite(A7, HIGH);
  network_flag = 'D';

  Serial.begin(9600);

  // LED init
  pinMode(LED_WARNING, OUTPUT);
  digitalWrite(LED_WARNING, LOW);
  blink_led(1);

  Wire.begin();

  // VOC sensor init
  if (!sgp.begin()){
    Serial.println("SGP40 sensor not found :(");
    while (1);
  }
  blink_led(1);

  // Temp/pressure init
  unsigned status = bme.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BME280 sensor"));
    while (1) delay(10);
  }
  blink_led(1);

  while(!dust_sensor.begin())
  {
    Serial.println(F("Dust sensor not found"));
    delay(WAIT_DELAY_MS);
  }
  init_done = true;

  delay(WAIT_DELAY_MS);
  // 3 blink - device is ready to use
  //airq_gate_transmission();
  Udp.begin(LOCAL_UDP_PORT);
  blink_led(3);
}

/*
void render() {
  switch (page_num) {
    case 1:
      air_print(data, "PM1", data.pm1, prev_value.pm1);
      break;
    case 2:
      air_print(data, "PM2", data.pm2, prev_value.pm2);
      break;
    case 3:
      air_print(data, "P10", data.pm10, prev_value.pm10);
      break;
    case 4:
      air_print(data, "VOC", data.tvoc, prev_value.tvoc);
      break;
    case 5:
      air_print(data, "CO2", data.co2, prev_value.co2);
      break;
    case 6:
      print_legend(F("AQI Levels (index)\nGood: < 50\nAVG: 50-200\nBAD: > 200"));
      break;
    case 7:
      print_legend(F("PM1 Levels (ug/m3)\nGood: < 50\nAVG: 50-350\nBAD: > 350"));
      break;
    case 8:
      print_legend(F("PM2 Levels (ug/m3)\nGood: < 50\nAVG: 50-150\nBAD: > 150"));
      break;
    case 9:
      print_legend(F("PM10 Levels (ug/m3)\nGood: < 50\nAVG: 50-355\nBAD: > 355"));
      break;
    case 10:
      print_legend(F("CO2 Levels (ppm)\nGood: < 800\nAVG: 800-1500\nBAD: > 1500"));
      break;
    case 11:
      print_legend(F("TVOC Levels (ppb)\nGood: < 220\nAVG: 221-660\nBAD: > 660"));
      break;
    case 12:
      print_legend(F("Humidity levels (%)\nGood: > 50\nAVG: 30-50\nBad: < 30"));
      break;
    default:
      air_print(data, "AQI", data.aqi, prev_value.aqi);
      break;
  }
}*/

/* Wifi auto reconnect */
bool airq_gate_connection() {
  if (wifi_status != WL_CONNECTED || cnt > RECONNECT_AFTER){
      wifi_status = WiFi.begin(ssid, pass);
      cnt = 0;
  }
  if (wifi_status == WL_CONNECTED){
    network_flag = 'U';
    cnt += 1;
  } else {
    network_flag = 'D';
  }
  return wifi_status = WL_CONNECTED;
}

/* Device event loop */
void loop() {
  // Init check
  if (!init_done) {
    delay(WAIT_DELAY_MS);
    return;
  }

  // Request sensors
  prev_value = data;
  data = read_sensors();

  // Prepare data and send
  send(data);

  // Next iteration
  delay(MEASURE_LOOP_MS);
}
