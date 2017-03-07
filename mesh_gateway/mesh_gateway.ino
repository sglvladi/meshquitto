/*=================================================================================== */
/* meshquitto/mesh_gateway.ino                                                        */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Example implementation of a meshquitto mesh gateway.                               */
/*                                                                                    */
/* Created by Lyudmil Vladimirov, Feb 2017                                            */
/* More info: https://github.com/sglvladi/meshquitto                                  */
/* ================================================================================== */

#include <SoftwareSerial.h>
#include <painlessMesh.h>
#include <ArduinoJson.h>
#include <HashMap.h> 
#include <Ticker.h>
#include <AES.h>
#include <Crc16.h>

// Mesh network details
#define   MESH_SSID       "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_ENCRYPT    "sampleEncryptKey"
#define   MESH_PORT       5555
#define   MESH_KEEP_ALIVE 5000  // (millis)

// Message buffer size
#define   BUFFER_SIZE     30

// Definition of TX/RX and TX/RX_IRQ (interrupt) pins
#define TX                D1
#define RX                D2
#define TX_IRQ            D4
#define RX_IRQ            D5

//#define TX_FSM_ABORT 0
//#define TX_FSM_READY 1
//#define TX_FSM_ACK   2

// Mesh intantiation
painlessMesh  mesh;

// Storage containers and buffers
SimpleList<uint32_t> nodes;                   // Stores list of all nodes
SimpleList<uint32_t> lostConnections;         // Stores list of all lost connections
SimpleList<unsigned long> lostConnTimeouts;   // Stores list of all lost connection timeouts (keep alive = 5 sec)
SimpleList<String> mqttMessageBuffer;         // Stores list of all messages queued to be forwarded to MQTT gateway
SimpleList<String> meshMessageBuffer;         // Stores list of all messages queued to be sent to Mesh network

// {NodeId => MAC} Map 
CreateHashMap(macAddressMap, int, String, 30);

// AES Encryption parameters
const String                  AES_KEY   = "0123456789010123";
const unsigned long long int  AES_IV    = 36753562;
const int                     AES_BITS  = 256;

// Timestamp to store last time a message was sent to Mesh
unsigned long lastMsg = millis();

SoftwareSerial swSer(RX, TX, false, 255);

// Global flags used for control
bool _sending   = false;
bool _receiving = false;

//Ticker RX_Interrupt_Ticker;
//void RX_check( void ){
//  if(digitalRead(RX_IRQ)==LOW){
    //RX_Interrupt_Ticker.detach();
//    receiveFromWiFi();
//  }
//}

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
/* Performs AES encryption on a given text. Key *must* be 16 bytes.      */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String AES_encrypt(String plain, String key) {
  /* Local AES instance (uncomment to recreate problem) */
  AES aes ; 
  aes.set_IV(AES_IV);
  byte * plain_buf = (unsigned char*)plain.c_str();
  byte * key_buf = (unsigned char*)key.c_str();
  // add padding where appropriate
  int cipher_length = (plain.length()+1 < 16) ? 16 : (plain.length()+1) + (16 - (plain.length()+1) % 16);
  byte cipher_buf[cipher_length];
  aes.do_aes_encrypt(plain_buf, plain.length() + 1, cipher_buf, key_buf, AES_BITS);
  String cipher = aes.printToHEXString(cipher_buf, cipher_length);
  //aes.printArray(cipher_buf, false);
  uint16_t plain_size = plain.length();
  char lo = plain_size & 0xFF;
  char hi = plain_size >> 8;
  String size_pad;
  for(uint8_t i=0; i<(4-String(plain.length()).length());i++){
    size_pad +="0";
  }
  size_pad+=String(plain.length());
  cipher+=size_pad;
  //Serial.println(sizeof(cipher_buf)/sizeof(cipher_buf[0]));
  //Serial.println(cipher.length());
  
  plain_size = cipher.substring(cipher.length()-4).toInt();
  //Serial.println(cipher);
  return cipher;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Performs AES decryption on a given cipher. Key *must* be 16 bytes.    */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String AES_decrypt(String cipher, String key) {
  //printHeap();
  /* Local AES instance (uncomment to recreate problem) */
  AES aes ; 
  //Serial.println(cipher);
  aes.set_IV(AES_IV);
  byte cipher_buf[cipher.length()/2-2];
  int j = 0;
  for(int i=0;i<=cipher.length()-6;i+=2){
    //Serial.print(cipher.substring(i,i+2));
    cipher_buf[j] = char(strtoul(cipher.substring(i,i+2).c_str(), NULL, 16));
    //Serial.print(char(cipher_buf[j]));
    j++;
  }
  //printHeap();
  //aes.printArray(cipher_buf, false);
  int plain_size = cipher.substring(cipher.length()-4).toInt();
  //Serial.println(plain_size);
 // byte * cipher_buf = (unsigned char*)cipher.c_str();
  byte * key_buf = (unsigned char*)key.c_str();
  int plain_length = cipher.length();
  byte plain_buf[plain_length];
  aes.do_aes_decrypt(cipher_buf, cipher.length()/2-2, plain_buf, key_buf, AES_BITS);
  String plain = aes.printToString(plain_buf, plain_size);
  //Serial.println(sizeof(plain_buf)/sizeof(plain_buf[0]));
  //Serial.println(plain.length());
  //printHeap();
  return plain;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*****************************************************************************/
/* Computes and returns the set difference between two SimpleList<uint32_t>  */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
SimpleList<uint32_t> getDifference(SimpleList<uint32_t> arr1, SimpleList<uint32_t> arr2){
  if(arr1.size()==0){
    return arr2;
  }
  if(arr2.size()==0){
    return arr1;
  }
  int m = arr1.size();
  int n = arr2.size();
  SimpleList<uint32_t> diff;
  int i = 0, j = 0;
  while (i < m && j < n)
  {
    if (arr1[i] < arr2[j])
      diff.push_back(arr1[i++]);
    else if (arr2[j] < arr1[i])
      diff.push_back(arr2[j++]);
    else /* if arr1[i] == arr2[j] */
    {
      j++;//Serial.printf(" %d ", arr2[j++]);
      i++;
    }
  }
  while(i<m){
    diff.push_back(arr1[i++]);
  }
  while(j<n){
    diff.push_back(arr2[j++]);
  }
  
  return diff;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*******************************************************************************/
/* Computes and returns the set intersection between two SimpleList<uint32_t>  */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
SimpleList<uint32_t> getIntersection(SimpleList<uint32_t> arr1, SimpleList<uint32_t> arr2){
  
  int m = arr1.size();
  int n = arr2.size();
  SimpleList<uint32_t> intersect;
  int i = 0, j = 0;
  while (i < m && j < n)
  {
    if (arr1[i] < arr2[j])
      i++;
    else if (arr2[j] < arr1[i])
      j++;
    else /* if arr1[i] == arr2[j] */
    {
      intersect.push_back(arr2[j++]);
      i++;
    }
  }
  return intersect;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/**************************************************************************/
/* Given two lists, before and after a new scan, computes and returns all */
/* nodes that have disconnected since the last scan.                      */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
SimpleList<uint32_t> getLostConnections(SimpleList<uint32_t> nodes, SimpleList<uint32_t> old_nodes){
  if(nodes.size()==0){
    return old_nodes;
  }
  SimpleList<uint32_t> diff = getDifference(old_nodes, nodes);
  //Serial.println(diff.size());
  Serial.print("Diff: "); 
  SimpleList<uint32_t>::iterator node = diff.begin();
    while (node != diff.end()) {
        Serial.printf(" %u", *node);
        node++;
    }
  Serial.println();
  SimpleList<uint32_t> lostCons = getIntersection(diff, old_nodes);
  return lostCons;
  
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/**************************************************************************/
/* Given two lists, before and after a new scan, computes and returns all */
/* newly connected nodes.                                                 */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
SimpleList<uint32_t> getNewConnections(SimpleList<uint32_t> nodes, SimpleList<uint32_t> old_nodes){
  
  SimpleList<uint32_t> diff = getDifference(old_nodes, nodes);
  SimpleList<uint32_t> newCons = getIntersection(diff, nodes);
  return newCons;
  
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/***********************************************************************/
/* Checks for any connections which have been lost for longer than the */
/* keep alive interval, and forwards a "willTopic" to MQTT gateway     */
/* where applicable.                                                   */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void updateLostConnections(){
  if(lostConnections.size()>0){
    //Serial.println("Updating Lost Connections");
    SimpleList<uint32_t>::iterator node = lostConnections.begin();
   // Serial.println(lostConnections.size());
    int i;
    while (node != lostConnections.end()) 
    {
        //Serial.println(node-nodes.begin());
        if(millis()-lostConnTimeouts[i]>MESH_KEEP_ALIVE){
          String topic = String(String(macAddressMap[lostConnections[i]])+"/disconnect");
          String payload = "1";
          String json_msg =jsonMqttMessage(topic, payload);
          if(mqttMessageBuffer.size()<BUFFER_SIZE){
            mqttMessageBuffer.push_back(json_msg);
          }
          else{
            Serial.print("Dropping messages!!!");
          }
          sendToMQTT(topic, payload);
          macAddressMap.remove(lostConnections[i]);
          lostConnections.remove(i);
          lostConnTimeouts.remove(i);
          break;
        }
        i++;
        node++;
    } 
    //printHeap();
  }
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/************************************************************************/
/* Mesh received callback: Receives a message and pushes it to buffer.  */
/* Also checks to see if the sender's mac is stored in the macAddresMap */
/* If not, it means this is the first message received from this device */
/* (after losing connection) and any "willTopic" for that device is     */ 
/* cleared, by pushing another message to the buffer.                   */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %d msg=%s\n", from, msg.c_str());
  msg = AES_decrypt(msg, AES_KEY);
  char contentBuffer[500];
  msg.toCharArray(contentBuffer,500);
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& rootFS2 = jsonBuffer.parseObject(contentBuffer);
  String topic = rootFS2["topic"].as<String>();
  String payload = rootFS2["payload"].as<String>();
  int endPos = topic.indexOf("/");
  String macAddress = topic.substring(0, endPos);

  // Check if macAddress is already stored else clear
  if(macAddressMap.indexOf(from)==-1){
    macAddressMap[from] = macAddress;
    String topic = String(String(macAddress+"/disconnect"));
    String payload = "0";
    String json_msg =jsonMqttMessage(topic, payload);
    // Push "willTopic" clear meassage to buffer
    if(mqttMessageBuffer.size()<BUFFER_SIZE){
      mqttMessageBuffer.push_back(json_msg);
    }
    else{
      Serial.print("Dropping messages!!!");
    }
  }

  // Push received message to buffer.
  String json_msg =jsonMqttMessage(topic, payload);
  if(mqttMessageBuffer.size()<BUFFER_SIZE){
    mqttMessageBuffer.push_back(json_msg);
  }
  else{
    Serial.print("Dropping messages!!!");
  }
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/************************************************************************/
/* Mesh changed connection Callback: Detection and processing of lost   */
/* and new connections is done here.
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void changedConnectionCallback() {
 
    Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());
    SimpleList<uint32_t> old_nodes = nodes;
    
    // Print old available connections to nodes
    Serial.printf("Old Connection list:");
    SimpleList<uint32_t>::iterator node = old_nodes.begin();
    while (node != old_nodes.end()) {
        Serial.printf(" %u", *node);
        node++;
    }
    Serial.println();
    nodes = mesh.getNodeList();
    nodes.sort();
    Serial.printf("Num nodes: %d\n", nodes.size());

    // Print all available connections to nodes
    Serial.printf("Connection list:");
    node = nodes.begin();
    while (node != nodes.end()) {
        Serial.printf(" %u", *node);
        node++;
    }
    Serial.println();

    // Get and print lost connections to nodes
    SimpleList<uint32_t> lostConns = getLostConnections(nodes, old_nodes);
    Serial.printf("Lost Connections: ");
    node = lostConns.begin();
    while (node != lostConns.end()) {
        Serial.printf(" %u", *node);
        node++;
    }
    Serial.println();
    if(lostConns.size()>0){
      Serial.println(lostConns.size());
      node = lostConns.begin();
      while (node != lostConns.end()) {
        Serial.printf(" %u", *node);
        int index = lostConnections.indexOf(*node);
        if(index==-1){
          lostConnections.push_back(*node);
          lostConnTimeouts.push_back(millis());
        }
        node++;
      }
    }
    Serial.println();
    // Free up the memory
    //lostConns.~SimpleList<uint32_t>();
    printHeap();
    
    // Get and print new lost connections to nodes
    SimpleList<uint32_t> newConns = getNewConnections(nodes, old_nodes);
    Serial.printf("New Connections: ");
    node = newConns.begin();
    while (node != newConns.end()) {
        Serial.printf(" %u", *node);
        node++;
    }
    Serial.println();
    if(newConns.size()>0){
      Serial.println(newConns.size());
      node = newConns.begin();
      while (node != newConns.end()) {
        Serial.printf(" %u", *node);
        int index = lostConnections.indexOf(*node);
        if(index!=-1){
          Serial.print("Removing index: "); Serial.println(index);
          Serial.print("Removing: "); Serial.println(*node);
          lostConnections.remove(index);
          lostConnTimeouts.remove(index);
          Serial.print("Remaining: ");
          SimpleList<uint32_t>::iterator lost_node = lostConnections.begin();
          while (lost_node != lostConnections.end()) {
            Serial.printf(" %u", *lost_node);
            lost_node++;
          }
          Serial.println();
          Serial.print("        : ");
          SimpleList<unsigned long>::iterator lost_timeout = lostConnTimeouts.begin();
          while (lost_timeout != lostConnTimeouts.end()) {
            Serial.printf(" %u", millis()-*lost_timeout);
            lost_timeout++;
          }
          Serial.println();
        }
        node++;
      }
    }
    Serial.println();
    // Free up the memory
    //newConns.~SimpleList<uint32_t>();
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/************************************************************************/
/* Currently unused painlessMesh callbacks                              */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}
void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}
void delayReceivedCallback(uint32_t from, int32_t delay) {
    Serial.printf("Delay to node %u is %d us\n", from, delay);
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Reads topic and payload of MQTT message, formats it as JSON String    */
/* and forwards it to MQTT.                                              */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void sendToMQTT(String topic, String payload){
    _sending = true;
    Serial.print("Forwarding message to WiFi GW");
    DynamicJsonBuffer jsonBufferFS;
    JsonObject& rootFS2 = jsonBufferFS.createObject();
    rootFS2["topic"] = topic;
    rootFS2["payload"] = payload;
    String json_msg;
    rootFS2.printTo(json_msg);
    for (int i = 0; i < json_msg.length(); i++) {
      Serial.print(".");
      swSer.write(json_msg[i]);
    }
    swSer.write("\\");
    swSer.flush();
    Serial.println("DONE");
    Serial.print("TOPIC: "); Serial.println(topic);
    Serial.print("VALUE: "); Serial.println(payload);
    printQueueSizes();
    Serial.println("==================================> ");

    // Pulse RX_IRQ to tell MQTT GW a message has been written to serial buffer
    digitalWrite(TX_IRQ, LOW);
    delay(10);
    digitalWrite(TX_IRQ, HIGH);
    
    _sending = false;
    printHeap();
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Reads a JSON message and forwards it to MQTT gateway                  */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void sendToMQTT(String json_msg){
    //_sending = true;
    String crc_msg = getCRCString(json_msg);
    for (int i = 0; i < crc_msg.length(); i++) {
      Serial.print(".");
      swSer.write(crc_msg[i]);
    }
    swSer.write("\\");
    Serial.println("DONE");
    Serial.print("MESSAGE: "); Serial.println(crc_msg);
    printQueueSizes();
    //Serial.print("TOPIC: "); Serial.println(topic);
    //Serial.print("VALUE: "); Serial.println(payload);
    Serial.println("==================================> ");

    // Pulse RX_IRQ to tell MQTT GW a message has been written to serial buffer
    digitalWrite(TX_IRQ, LOW);
    delay(10);
    digitalWrite(TX_IRQ, HIGH);
    _sending = false;
    printHeap();
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*************************************************************************/
/* Function called when MQTT Interrupt (RX_IRQ) is triggered.            */
/* Checks to see if max buffer limit is reached. If not, the received    */
/* message is read and pushed to the Mesh buffer. Otherwise, it signals  */
/* that buffer needs to be emptied and drops any messages until done.    */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void receiveFromWiFi( void ){
  if(mqttMessageBuffer.size()>=30){
    _empty_mqtt_buffer_irq = true;
  }
  if(!_empty_mqtt_buffer_irq){
    _receiving = true;
    Serial.println("WiFi GW interrupt detected");
    bool received = false;
    String swMessage;
    while (swSer.available() > 0) {
      char swChar = swSer.read();
      if(swChar!='ÿ'){ // Exclude character ÿ
        swMessage += swChar;
      }
      received = true;
    }
    if(received && swMessage!=""){
      Serial.print("Received message from WiFi gateway: ");
      swMessage = swMessage.substring(swMessage.indexOf("{"));//replace("ÿ","");
      Serial.println(swMessage);
      char contentBuffer[500];
      swMessage.toCharArray(contentBuffer,500);
      StaticJsonBuffer<500> jsonBuffer;
      JsonObject& rootFS2 = jsonBuffer.parseObject(contentBuffer);
      String topic = rootFS2["topic"].as<String>();
      int endPos = topic.indexOf("/",2);
      String macAddress = topic.substring(1, endPos);
      if(macAddressMap.indexOfValue(macAddress)!=-1){
        uint32_t nodeId = macAddressMap.keyAt(macAddressMap.indexOfValue(macAddress));
        String encrypted_msg = AES_encrypt(swMessage, AES_KEY);
        Serial.print("Sending to: "); Serial.println(nodeId);
        mesh.sendSingle(nodeId, encrypted_msg);
      }
      else{
        Serial.println("NOPE...............................................");
      }
    }
    else{
      Serial.println("Failed!");
    }
    _receiving = false;
    //RX_Interrupt_Ticker.attach(0.1, RX_check);
  }
  else{
    Serial.println("Mesh buffer full!! Dropping data while it's emptying....");
  }
}

void setup() {
  
  // Start HW and SW Serials
  Serial.begin(115200);
  swSer.begin(38400);

  // Print initial debugging info
  Serial.println("\nMeshquitto Mesh Gateway started!");
  Serial.print("Chip ID: "); Serial.println(ESP.getChipId());

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP| CONNECTION );  // set before init() so that you can see startup messages
  mesh.init(MESH_SSID, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  // Set up TX and RX pins and interrupts
  pinMode(TX_IRQ, OUTPUT);
  digitalWrite(TX_IRQ, HIGH);
  pinMode(RX_IRQ, INPUT);
  //RX_Interrupt_Ticker.attach(0.1, RX_check);
  attachInterrupt(RX_IRQ, receiveFromWiFi, FALLING);
}

void loop() {
  mesh.update();
  updateLostConnections();
  
  if(!_receiving&&(mqttMessageBuffer.size()>0)){
    // Ensure we only send up to 10 messages a second
    // This avoids overloading of the MQTT server.
    if(millis()-lastMsg>100){
      //delay(100);
      String msg = mqttMessageBuffer[0];
      sendToMQTT(msg);
      mqttMessageBuffer.pop_front();
      lastMsg = millis();
  //    switch(TX_FSM_STATE){
  //      case(TX_FSM_READY):{
  //        delay(50);
  //        String msg = mqttMessageBuffer[0];
  //        TX_FSM_STATE = TX_FSM_ACK;
  //        sendToMQTT(msg);
  //        break;
  //      }
  //      case(TX_FSM_ACK):{
  //        if(digitalRead(RTX_INFO)==HIGH){
  //          mqttMessageBuffer.pop_front();
  //        }
  //        TX_FSM_STATE = TX_FSM_READY;
  //        break;
  //      }
  //    }
    }
  }
}
