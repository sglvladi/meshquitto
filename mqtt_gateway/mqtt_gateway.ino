/*=================================================================================== */
/* meshquitto/mqtt_gateway.ino                                                        */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Example implementation of a meshquitto mqtt gateway.                               */
/*                                                                                    */
/* Created by Lyudmil Vladimirov, Feb 2017                                            */
/* More info: https://github.com/sglvladi/meshquitto                                  */
/* ================================================================================== */

#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <SimpleList.h>
#include <Crc16.h>

// Message buffer size
#define BUFFER_SIZE   30

// Definition of TX/RX and TX/RX_IRQ (interrupt) pins
#define TX            D1
#define RX            D2
#define TX_IRQ        D4
#define RX_IRQ        D5

//#define TX_FSM_ABORT 0
//#define TX_FSM_READY 1
//#define TX_FSM_ACK   2

// Software Serial instantiation
SoftwareSerial swSer(RX, TX, false, 255);

// Network settings
const char* ssid              = "some-SSID";
const char* password          = "some-PSK";

// MQTT connect settings
const char* mqtt_server       = "some-MQTT-server";
const char* mqtt_username     = "some-MQTT-username"; 
const char* mqtt_password     = "some-MQTT-password";
const char* mqtt_client_id    = "Mesquitto Gateway "+ ESP.getChipId();
//const char* mqtt_will_topic   = ("/1/gateways/"+getMac()+"/disconnected").c_str();
const char* mqtt_will_payload = "1";
const int   mqtt_will_qos     = 1;
const bool  mqtt_will_retain  = true; 

// WiFi and MQTT client instantiation
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Buffers to store messages
SimpleList<String> mqttMessageBuffer;         // Stores list of all messages queued to be sent to MQTT broker
SimpleList<String> meshMessageBuffer;         // Stores list of all messages queued to be forwarded to Mesh gateway

// Timestamp to store last time a message was sent to Mesh
long lastMsg = 0;

// Global flags used for control
bool _sending                 = false;                // Flag set when sending to Mesh
bool _receiving               = false;              // Flag set when receiving from mesh
bool _empty_mqtt_buffer_irq   = false;  // Flag set when 

//String TX_FSM_STATE = TX_FSM_READY;
//Ticker RX_Interrupt_Ticker;
//void RX_check( void ){
// if(digitalRead(RX_IRQ)==LOW){
//   RX_Interrupt_Ticker.detach();
//    receiveFromMesh();
//  }
//}

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


/************************************************************************/
/* Prints available heap memory to Serial                               */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void printHeap(){                                                       
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  Serial.print("Free Heap: "); Serial.println(ESP.getFreeHeap());
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/************************************************************************/
/* Prints the number of queued messages to Serial                       */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void printQueueSizes(){
  Serial.print("MQTT queue: "); Serial.println(mqttMessageBuffer.size());
  Serial.print("Mesh queue: "); Serial.println(meshMessageBuffer.size());
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/************************************************************************/
/* Computes CRC16 of given data, then appends it to data and returns    */
/* the resulting string                                                 */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/************************************************************************/
/* Reads a string of data with it's CRC16 appended to the end, then     */
/* computes a new CRC16 based on the raw data and returns true if       */
/* the two CRC16 values match.                                          */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Reads topic and payload of MQTT message and formats it as JSON string */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String jsonMqttMessage(String topic, String payload){
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& rootFS2 = jsonBuffer.createObject();
  rootFS2["topic"] = topic;
  rootFS2["payload"] = payload;
  String json_msg;
  rootFS2.printTo(json_msg);
  return json_msg;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Reads topic and payload of MQTT message, formats it as JSON String    */
/* and forwards it to Mesh gateway                                       */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Reads a JSON message and forwards it to Mesh gateway                  */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Function called when Mesh Interrupt (RX_IRQ) is triggered.            */
/* Checks to see if max buffer limit is reached. If not, the received    */
/* message is read and pushed to the MQTT buffer. Otherwise, it signals  */
/* that buffer needs to be emptied and drops any messages until done.    */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void receiveFromMesh( void ){
  if(mqttMessageBuffer.size()>=30){
    _empty_mqtt_buffer_irq = true;
  }
  if(!_empty_mqtt_buffer_irq){
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
            mqttMessageBuffer.push_back(swMessage);
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
    Serial.println("MQTT buffer full!! Dropping data while it's emptying....");
  }
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Connects to Wifi network with the given credentials                   */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
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
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* MQTT callback function: Receives MQTT messages from the broker, then  */
/* formats them as JSON Strings and pushes them onto the Mesh buffer     */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void callback(char* topic, byte* payload, unsigned int length) {
  // Print some debugging
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String value;
  for (int i = 0; i < length; i++) {
    value += (char)payload[i];
  }
  Serial.println(value);

  // Extract all subtopics
  String topic_str = String(topic);
  int topic_length = topic_str.length();
  int subtopicNo = 0;
  for (int i=0; i<topic_length; i++){
    if (String(topic_str.charAt(i)) == "/"){
      subtopicNo++;
    }
  }
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

  // Format and push message to buffer
  String mesh_topic;
  for(int i=3; i<subtopicNo; i++){
    mesh_topic+="/";
    mesh_topic+=subtopics[i];
  }
  meshMessageBuffer.push_back(jsonMqttMessage(mesh_topic, value));
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* MQTT reconnect funtion: Get called when a connection is lost or first */
/* started.                                                              */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password, ("/1/gateways/"+getMac()+"/disconnected").c_str(), mqtt_will_qos, mqtt_will_retain, mqtt_will_payload)) {
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
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


void setup() {

  // Start HW and SW Serials
  Serial.begin(115200);
  swSer.begin(38400);

  // Print initial debugging info
  Serial.println("\nMeshquitto MQTT Gateway started!");

  // Setup WiFi and MQTT connections
  setup_wifi();
  client.setServer(mqtt_server, 8883);
  client.setCallback(callback);
  reconnect();
  client.loop();

  // Set up TX and RX pins and interrupts
  pinMode(TX_IRQ, OUTPUT);
  digitalWrite(TX_IRQ, HIGH);
  pinMode(RX_IRQ, INPUT);
  attachInterrupt(RX_IRQ, receiveFromMesh, FALLING);
}


void loop() {

  // MQTT_loop
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Publish any available messages received from Mesh to MQTT
  if(mqttMessageBuffer.size()>0&&!_receiving){
    while(mqttMessageBuffer.size()>0){
      printHeap();
      String swMessage = mqttMessageBuffer[0];
      mqttMessageBuffer.pop_front();
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
    _empty_mqtt_buffer_irq = false;
  }

  // Forward any available MQTT messages to Mesh network
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
}
