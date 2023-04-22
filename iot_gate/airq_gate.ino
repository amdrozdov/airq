#include "arduino_secrets.h"
/*
// Add adruino_secrets.h to gitignore
// Passwords data
const char* GATE_SSID= "<IoT network name>";
const char* GATE_PWD = "<IoT network password>";
const char* NET_SSID = "<External network name>";
const char* NET_PWD  = "<External network password>";
*/

#include "WiFi.h"
#include <math.h>
#include <Adafruit_GFX.h>   
#include <Adafruit_ST7789.h> 
#include <SPI.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include <ArduinoMqttClient.h>
#include "Adafruit_MAX1704X.h"
#include <Adafruit_NeoPixel.h>

// Constants
#define TFT_I2C_POWER  21
#define TFT_CS          7
#define TFT_RST        40
#define TFT_DC         39
#define TFT_BACKLITE   45
#define LED_BUILTIN    13
#define PIN_NEOPIXEL   33
#define NEOPIXEL_NUM    1 
#define NEOPIXEL_POWER 34   
#define DATA_PORT    2390
#define INFO_SIZE       2
#define AQI_GOOD       50
#define AQI_BAD       200
#define VOC_GOOD      120
#define VOC_BAD       660
#define RECONNECT_AFTER 5
#define WF_SOFT_DSC    15
#define WF_SCAN_MS   1000

const char* MQTT_HOST = "192.168.1.80";
const int   MQTT_PORT = 1883;
const char* TOPIC_TMP = "/airq/dust/temperature";
const char* TOPIC_PRE = "/airq/dust/pressure";
const char* TOPIC_HUM = "/airq/dust/humidity";
const char* TOPIC_AQI = "/airq/dust/aqi";
const char* TOPIC_VOC = "/airq/dust/voc";
const char* TOPIC_PM1 = "/airq/dust/pm1";
const char* TOPIC_PM2 = "/airq/dust/pm2";
const char* TOPIC_PM10 = "/airq/dust/pm10";

// Hardware structs
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_MAX17048 maxlipo;
Adafruit_NeoPixel strip(NEOPIXEL_NUM, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Network data
//Static IP(hardcoded for internal net AP)
IPAddress local_ip(192,168,4,1);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);
WiFiUDP Udp;
// Connected devices structs
wifi_sta_list_t wifi_sta_list;
tcpip_adapter_sta_list_t adapter_sta_list;
// Wifi and mqtt structs
WiFiClient espClient;
MqttClient mqttClient(espClient);
// Flags and counters
int ticks_without_transmission = 0;
int disconnect_cnt = 0;
int wifi_cnt = 0;
int mqtt_cnt = 0;
int wifi_status = WL_IDLE_STATUS;

/* AIRq data*/
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
byte udp_buf[sizeof(air_data)];

// UI data
char loader[4] = {'-', '\\', '|', '/'};
int loader_state = 0;
bool has_data = false;
int loader_cnt = 0;
char mqtt_flag = ' ';
char network_flag = 'D';

/* Device init */
void setup()
{
  Serial.begin(115200);
  
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
  strip.begin();
  strip.show();

  Serial.begin(9600);

  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);  

  if (!maxlipo.begin()) {
    Serial.println(F("Couldnt find Adafruit Battery"));
    while (1) delay(10);
  }

  // Wifi mode AP + Client (in order to have 2 networks)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(GATE_SSID, GATE_PWD);
  local_ip = WiFi.softAPIP();
  Udp.begin(DATA_PORT);

  // Show loader after hardware init
  show_animation();
  // Try to start in external mode
  if(!restore_connection()){
    Serial.println("Start in internal only mode");
  }
}

/* Print wrapper for LCD */
void print_text(char *text, int size, int color, bool n){
  tft.setTextColor(color);
  tft.setTextSize(size);
  if(n){
      tft.println(text);
  }
  else{
      tft.print(text);    
  }
}

/* Clear screen wrapper*/
void cls() {
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
}

/* IP Struct print for UI */
void print_ip(IPAddress addr, int size, int color){
    char out[4];
    for(int i = 0; i < 4; i++) {
      memset(out, 0, sizeof(out));
      if(i < 3){
          sprintf(out, "%d.", addr[i]);
      }
      else {
          sprintf(out, "%d", addr[i]);
      }
      print_text(out, size, color, false);
    }
}

/* Loader UI */
void show_animation(){
    cls();
    print_text("AIRq Gate", 4, ST77XX_BLUE, true);
    print_text("v.1.0.0", 2, ST77XX_BLUE, true);
    print_text("Loading", 2, ST77XX_RED, true);

    for (int i=0; i<10; i++){
      print_text("-", 2, ST77XX_RED, false);
      delay(100);
    }
}

/*void info_header(int i, int total){
    char buf[64];
    int power = int(maxlipo.cellPercent());
    strip.clear();
    strip.setBrightness(5);
    if(power > 80){
      strip.setPixelColor(0, 0, 255, 0);
    }
    else if(power > 30){
      strip.setPixelColor(0, 255, 255, 0);
    } else {
      strip.setPixelColor(0, 255, 0, 0);
    }
    strip.show();

    memset(buf, 0, sizeof(buf));
    sprintf(buf, "Net %2d/%2d:     B%3d%%", i+1, total, power);
    print_text(buf, 2, ST77XX_BLUE, true);
}*/

/* Top UI bar with device information */
void info_header(bool status){
    char buf[64];
    int p = get_bat_power();
    int header_color;

    memset(buf, 0, sizeof(buf));
    if(status){
        sprintf(buf, "Online         B%3d%%", p);
        buf[7] = loader[loader_state++];
        buf[9] = mqtt_flag;
        buf[10] = 'D'; // for dust module
        loader_state = loader_state % sizeof(loader);
        header_color = ST77XX_MAGENTA;
    } else {
        sprintf(buf, "Offline        B%3d%%", p);
        buf[8] = mqtt_flag;
        header_color = ST77XX_YELLOW;
    }

    print_text(buf, 2, header_color, true);
    print_text(  "--------------------", 2, ST77XX_CYAN, true);

}

/* Returns battery charge in % from hardware */
int get_bat_power(){
  int power = int(maxlipo.cellPercent());
  strip.clear();
  strip.setBrightness(5);
  if(power > 80){
    strip.setPixelColor(0, 0, 255, 0);
  }
  else if(power > 30){
    strip.setPixelColor(0, 255, 255, 0);
  } else {
    strip.setPixelColor(0, 255, 0, 0);
  }
  strip.show();
  return power;
}

/* UDP decoder for airq data*/
air_data decode_air_buffer(){
    air_data result;

    int len = Udp.read(udp_buf, sizeof(air_data));
    if(len != sizeof(air_data)){
      Serial.println("Incorrect air_data protocol");
      result.humidity = -1.0;
      return result;
    }

    byte *ptr = (byte*)&result;
    for(int i=0;i<sizeof(air_data);i++){
      *ptr++ = udp_buf[i];
    }
    memset(udp_buf, 0, sizeof(udp_buf));
    return result;
}

/* Metrics color highlight for UI*/
int highlight(int value, int good, int bad){
    if(value > good && value < bad){
      return ST77XX_YELLOW;
    }
    if (value >= bad){
      return ST77XX_RED;
    }
    return ST77XX_GREEN;
}

/* UI render */
void render(air_data data){
    cls();
    info_header(has_data);
    print_text("Status:  ", 2, ST77XX_CYAN, false);
    if(data.level == 'S'){
        print_text("Safe", 2, ST77XX_GREEN, true);
    } else if (data.level == 'A'){
        print_text("Average", 2, ST77XX_YELLOW, true);
    } else {
        print_text("Danger!", 2, ST77XX_RED, true);
    }
    
    print_text("AQI = ", 3, ST77XX_CYAN, false);
    tft.setTextColor(highlight(data.aqi, AQI_GOOD, AQI_BAD));
    tft.println(data.aqi);

    print_text("VOC = ", 3, ST77XX_CYAN, false);
    int voc_color = highlight(data.tvoc, VOC_GOOD, VOC_BAD);
    tft.setTextColor(voc_color);
    tft.print(data.tvoc);
    print_text(" ppm", 3, voc_color, true);
    print_text(  "--------------------", 2, ST77XX_CYAN, true);

    print_text("Temp     = ", 1, ST77XX_WHITE, false);
    tft.print(int(data.temp));
    print_text("C,  PM1   = ", 1, ST77XX_WHITE, false);
    tft.print(data.pm1);
    print_text(" ug/m3", 1, ST77XX_WHITE, true);

    print_text("Hum      = ", 1, ST77XX_WHITE, false);
    tft.print(int(data.humidity));
    print_text("%,  PM2.5 = ", 1, ST77XX_WHITE, false);
    tft.print(data.pm2);
    print_text(" ug/m3", 1, ST77XX_WHITE, true);
    
    print_text("Pressure = ", 1, ST77XX_WHITE, false);
    tft.print(int(data.pressure));
    print_text(", PM10  = ", 1, ST77XX_WHITE, false);
    tft.print(data.pm10);
    print_text(" ug/m3", 1, ST77XX_WHITE, true);
}

/* Device scanner UI*/
void searching_ui(){
    cls();
    info_header(has_data);
    char lbuf[9] = {'S', 'c', 'a', 'n', '.', '.', ' ', ' ',0};
    lbuf[7] = loader[loader_cnt++];
    loader_cnt = loader_cnt % 4;
    print_text(lbuf, 4, ST77XX_CYAN, true);

    print_text("NET:  ", 2, ST77XX_GREEN, false);
    tft.println(GATE_SSID);
    print_text("IP:   ", 2, ST77XX_GREEN, false);
    print_ip(local_ip, 2, ST77XX_GREEN);
    tft.println();
    print_text("PORT: ", 2, ST77XX_GREEN, false);
    tft.println(DATA_PORT);
}

/* Serial logging for UDP*/
void trace_packet(int packetSize){
    Serial.print("Received ");
    Serial.print(packetSize);
    Serial.print(" bytes from ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(" on port ");
    Serial.println(Udp.remotePort());
}

/* Wifi autoconnect (works in the IO loop)*/
bool wifi_connection() {
  if (wifi_status != WL_CONNECTED || wifi_cnt > RECONNECT_AFTER){
      wifi_status = WiFi.begin(NET_SSID, NET_PWD);
      wifi_cnt = 0;
  }
  if (wifi_status == WL_CONNECTED){
    network_flag = 'U';
    wifi_cnt += 1;
  } else {
    network_flag = 'D';
  }
  return wifi_status = WL_CONNECTED;
}

/* Network entrypoint (for IO loop)
   Returns true if we are connected to Wifi and MQTT
*/
bool restore_connection(){
  /* allow only for 2 net transmission mode*/
  
  bool has_wifi = wifi_connection();
  if(!has_wifi){
    return false;
  }
  int mqtt_error = 0;
  bool has_mqtt = mqttClient.connected();
  if(has_mqtt){
    mqtt_flag = '+';
  } else {
    mqtt_flag == ' ';    
  }
  if(!has_mqtt && mqtt_cnt >= RECONNECT_AFTER){
      mqtt_error = mqttClient.connect(MQTT_HOST, MQTT_PORT);
      mqtt_cnt = 0;
  }
  mqtt_cnt++;
  if(mqtt_error != 0) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
  }
  return mqttClient.connected() && (!mqtt_error);
}

/* Sends sensors data to external network/broker */
void mqtt_broadcast(air_data data){
    if(!restore_connection()){
      return;
    }
    mqttClient.beginMessage(TOPIC_TMP);
    mqttClient.print(data.temp);
    mqttClient.endMessage();

    mqttClient.beginMessage(TOPIC_PRE);
    mqttClient.print(data.pressure);
    mqttClient.endMessage();

    mqttClient.beginMessage(TOPIC_HUM);
    mqttClient.print(data.humidity);
    mqttClient.endMessage();

    mqttClient.beginMessage(TOPIC_AQI);
    mqttClient.print(data.aqi+1);
    mqttClient.endMessage();

    mqttClient.beginMessage(TOPIC_VOC);
    mqttClient.print(data.tvoc+1);
    mqttClient.endMessage();

    mqttClient.beginMessage(TOPIC_PM1);
    mqttClient.print(data.pm1+1);
    mqttClient.endMessage();

    mqttClient.beginMessage(TOPIC_PM2);
    mqttClient.print(data.pm2+1);
    mqttClient.endMessage();
    
    mqttClient.beginMessage(TOPIC_PM10);
    mqttClient.print(data.pm10+1);
    mqttClient.endMessage();

    Serial.print("Sent ");
    Serial.print(sizeof(air_data));
    Serial.println(" bytes to external network");
}

/* Device event loop */
void loop()
{
    mqttClient.poll();

    // Hard check for disconnection
    disconnect_cnt++;
    if(disconnect_cnt == RECONNECT_AFTER){
      // Check device disconnection (timeout ~300sec)
      esp_wifi_ap_get_sta_list(&wifi_sta_list);
      tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
      if(has_data && !adapter_sta_list.num){
        Serial.println("Disconnect!");
        has_data = false;
      }
      disconnect_cnt = 0;
    }

    // Listen for UDP data
    int packetSize = Udp.parsePacket();

    // Soft check for disconnection (to detect in early then 2 minutes)
    if (!packetSize) {
      // Check for disconnects in advance
      ticks_without_transmission++;
      if(ticks_without_transmission > WF_SOFT_DSC){
        has_data = false;
      }
      if (!has_data){
        searching_ui();
      }
      delay(WF_SCAN_MS);
      return;
    }

    // We are online and have some data
    ticks_without_transmission = 0; 
    trace_packet(packetSize);    
    air_data data = decode_air_buffer();
    if(data.humidity < 0){
      // Decoding error - return
      Serial.println("AIRq protocol is incorrect - got wrong packet");
      return;
    }

    // Try send data to the external network
    mqtt_broadcast(data);

    // Set UI flag and render
    has_data = true;
    render(data);
}