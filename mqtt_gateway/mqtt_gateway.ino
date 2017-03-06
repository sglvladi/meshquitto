#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <SimpleList.h>
#include <Crc16.h>

#define TX D1
#define RX D2
#define TX_IRQ D4
#define RX_IRQ D5

#define TX_FSM_ABORT 0
#define TX_FSM_READY 1
#define TX_FSM_ACK   2

SoftwareSerial swSer(RX, TX, false, 255);
// Update these with values suitable for your network.

const char* ssid     = "some-SSID";
const char* password = "some-PSK";
const char* mqtt_server = "192.168.1.3";
WiFiClient wificlient;
WiFiClientSecure espClient;
PubSubClient client(espClient);
SimpleList<String> meshMessageBuffer;
SimpleList<String> wifiMessageBuffer;
long lastMsg = 0;
char msg[50];
int value = 0;
bool _sending = false;
bool _receiving = false;
bool _mqtt_update = false;
String _topic;
int mqtt_sent=0;
String _swMessage;

bool _empty_wifi_buffer_irq = false;

String TX_FSM_STATE = TX_FSM_READY;

//Ticker RX_Interrupt_Ticker;
/************************************************************************/
/* Turns returned mac address into a readable string                    */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String macToStr(const uint8_t* mac)                                    
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
{
  String result;
  String atom;
  for (int i = 0; i < 6; ++i) {
    atom = String(mac[i], 16);
    atom.toUpperCase();
    if ( atom.length() < 2)
    {
      result += String ("0") + atom;
    }
    else
    {
      result += atom;
    }
    if (i < 5)
      result += '-';
  }
  return result;
}
/************************************************************************/


/************************************************************************/
/* Reads from DS18B20 sensor interfase given one-wire                   */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String getMac()                                    
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
{
  unsigned char mac[6];
  WiFi.macAddress(mac);
  return macToStr(mac);
}
/************************************************************************/

void printHeap(){
  Serial.print("Free Heap: "); Serial.println(ESP.getFreeHeap());
}

void printQueueSizes(){
  Serial.print("WIFI queue: "); Serial.println(wifiMessageBuffer.size());
  Serial.print("Mesh queue: "); Serial.println(meshMessageBuffer.size());
}

String getCRCString(String data){
  Crc16 crc;
  byte * data_buf = (unsigned char*)data.c_str();
  crc.clearCrc();
  unsigned short value = crc.XModemCrc(data_buf,0,data.length());
  String crc_value = String(value, HEX);
  while(crc_value.length()<4){
    crc_value = "0" + crc_value;
  }
  data += crc_value;
  return data;
}

bool checkCRC(String dataPlusCRC ){
  Crc16 crc;
  crc.clearCrc();
  byte data[dataPlusCRC.length()-4];
  for(int i=0; i<dataPlusCRC.length()-4; i++){
    data[i] = (byte) dataPlusCRC[i];
  }
  unsigned short crc_1 = strtoul(dataPlusCRC.substring(dataPlusCRC.length()-4).c_str(), NULL, 16);
  unsigned short crc_check = crc.XModemCrc(data,0,dataPlusCRC.length()-4);
  if(crc_1==crc_check){
    return true; 
  }
  return false;
}

String jsonMqttMessage(String topic, String payload){
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& rootFS2 = jsonBuffer.createObject();
  rootFS2["topic"] = topic;
  rootFS2["payload"] = payload;
  String json_msg;
  rootFS2.printTo(json_msg);
  return json_msg;
}

void sendToMesh(String topic, String payload){
    _sending = true;
    StaticJsonBuffer<500> jsonBufferFS;
    JsonObject& rootFS2 = jsonBufferFS.createObject();
    rootFS2["topic"] = topic;
    rootFS2["payload"] = payload;
    String json_msg;
    rootFS2.printTo(json_msg);
    Serial.print("Forwarding message to Mesh GW");
    for (int i = 0; i < json_msg.length(); i++) {
      Serial.print(".");
      swSer.write(json_msg[i]);
    }
    Serial.println("DONE");

    // Pulse TX_IRQ to tell Mesh GW a message has been written to serial buffer
    digitalWrite(TX_IRQ, LOW);
    delay(10);
    digitalWrite(TX_IRQ, HIGH);
    _sending = false;
}

void sendToMesh(String json_msg){
    _sending = true;
    Serial.print("Forwarding message to Mesh GW: "); Serial.println(json_msg);
    printQueueSizes();
    for (int i = 0; i < json_msg.length(); i++) {
      Serial.print(".");
      swSer.write(json_msg[i]);
    }
    Serial.println("DONE");
    
    // Pulse TX_IRQ to tell Mesh GW a message has been written to serial buffer
    digitalWrite(TX_IRQ, LOW);
    delay(10);
    digitalWrite(TX_IRQ, HIGH);
    _sending = false;
}

void RX_check( void ){
  if(digitalRead(RX_IRQ)==LOW){
    //RX_Interrupt_Ticker.detach();
    receiveFromMesh();
  }
}

void receiveFromMesh( void ){
  if(wifiMessageBuffer.size()>=30){
    _empty_wifi_buffer_irq = true;
  }
  if(!_empty_wifi_buffer_irq){
    _receiving = true;
    Serial.println("Mesh GW interrupt detected");
    printQueueSizes();
    bool received = false;
    String swMessage;
    //while(digitalRead(RX_IRQ)==LOW){
      while (swSer.available() > 0) {
        char swChar = swSer.read();
        Serial.print(swChar);
        if((char)swChar == '\\'){
          received = true;
          Serial.println("\n==================================> ");
          Serial.println("Received message from Mesh gateway: ");
          Serial.println("==================================> ");
          Serial.println(swMessage);
          if(!checkCRC(swMessage)){
            Serial.println("\n====================================>");
            Serial.println("CORRUPTED MESSAGE!!!!!!");
            Serial.println("====================================>");
            swMessage = "";
            break;
          }
          else{
            swMessage = swMessage.substring(0, swMessage.length()-4);
            _mqtt_update = true;
            wifiMessageBuffer.push_back(swMessage);
            printQueueSizes();
            swMessage = "";
            continue;
          }
        }
        if((char)swChar!='ÿ'){ // Exclude character ÿ
          swMessage += swChar;
        }
    // }
    }
    if(!received){
      Serial.println("Failed!");
    }
    _receiving = false;
  }
  else{
    Serial.println("Wifi buffer full!! Dropping data while it's emptying....");
  }
//  RX_Interrupt_Ticker.attach(0.1, RX_check);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String value;
  for (int i = 0; i < length; i++) {
    value += (char)payload[i];
  }
  Serial.println(value);
  String topic_str = String(topic);
  int topic_length = topic_str.length();
  int subtopicNo = 0;
  for (int i=0; i<topic_length; i++){
    if (String(topic_str.charAt(i)) == "/"){
      subtopicNo++;
    }
  }
  //Serial.print("subotpicNo: "); Serial.println(subtopicNo);
  String subtopics[subtopicNo];
  bool topicParsed = false;
  int parserPos = 0;
  int i = 0;
  while(!topicParsed){
    int startPos = parserPos+1;
    //Serial.print("indexOf: "); Serial.println(topic_str.indexOf("/", startPos));
    int endPos = topic_str.indexOf("/", startPos);
    if (endPos==-1){
      endPos = topic_str.length();
      topicParsed = true;
    }
    subtopics[i] = topic_str.substring(startPos, endPos);
    parserPos = endPos;
    i++;
  }
  //for(int i=0; i<subtopicNo; i++){
    //Serial.println(subtopics[i]);
  //}
  // 0:user_id, 1:"device", 2:GW_id (optional) 3:device_id, 4:"sensors", 5:sensor_id, 6: "input"/"output", 7: "value"/"min"/"max" etc.
  String mesh_topic;
  for(int i=3; i<subtopicNo; i++){
    mesh_topic+="/";
    mesh_topic+=subtopics[i];
  }
  meshMessageBuffer.push_back(jsonMqttMessage(mesh_topic, value));
  //sendToMesh(mesh_topic, value);
  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266MeshGateway1", "some-MQTT-account", "some-MQTT-password", String("/1/gateways/"+getMac()+"/disconnected").c_str(), 1, true, String("1").c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(String("/1/gateways/"+getMac()+"/disconnected").c_str(), String("0").c_str());
      // ... and resubscribe
      client.subscribe(String("/1/gateways/"+getMac()+"/+/sensors/+/input/value").c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(TX_IRQ, OUTPUT);
  digitalWrite(TX_IRQ, HIGH);
  pinMode(RX_IRQ, INPUT);
  attachInterrupt(RX_IRQ, receiveFromMesh, FALLING);
  swSer.begin(38400);
  setup_wifi();
  client.setServer(mqtt_server, 8883);
  client.setCallback(callback);
  Serial.println("\nSoftware serial test started");

//  for (char ch = ' '; ch <= 'z'; ch++) {
//    swSer.write(ch);
//  }
//  swSer.println("");

}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //Serial.println("Here");
  if(_mqtt_update&&!_receiving){
    //Serial.println(meshMessageBuffer.size());
    while(wifiMessageBuffer.size()>0){
      printHeap();
      String swMessage = wifiMessageBuffer[0];
      //Serial.println(swMessage);
      wifiMessageBuffer.pop_front();
      char contentBuffer[500];
      swMessage.toCharArray(contentBuffer,500);
      StaticJsonBuffer<500> jsonBuffer;
      JsonObject& rootFS2 = jsonBuffer.parseObject(contentBuffer);
      if(rootFS2["topic"].as<String>()!="" && rootFS2["payload"].as<String>()!=""){
        String topic = "/1/gateways/"+getMac()+"/"+rootFS2["topic"].as<String>();
        String payload = rootFS2["payload"].as<String>();
        Serial.print("TOPIC: "); Serial.println(topic);
        Serial.print("VALUE: "); Serial.println(payload);
        Serial.println("==================================> ");
        client.publish((topic).c_str(), payload.c_str());
        printHeap();
      }
    }
    _empty_wifi_buffer_irq = false;
    _mqtt_update=false;
  }
  if(!_receiving){
     while(meshMessageBuffer.size()>0){
        String msg = meshMessageBuffer[0];
        Serial.println("Sending message to GW: ");
        Serial.println(msg);
        meshMessageBuffer.pop_front();
        sendToMesh(msg);
        printHeap();
     }
     
  }
  //while (swSer.available() > 0) {
  //  Serial.write(swSer.read());
  //}
 
  //while (Serial.available() > 0) {
  //  swSer.write(Serial.read());
  //}
}
