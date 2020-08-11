/*=================================================================================== */
/* meshquitto_node/meshquitto_node.ino                                                */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Example implementation of a meshquitto node.                                       */
/*                                                                                    */
/* This example assumes that a DS18B20 temp sensor is plugged in on port D4 and a LED */
/* on port D1. For exact wiring refer to mesquitto_node.png                           */
/*                                                                                    */
/* Created by Lyudmil Vladimirov, Feb 2017 (Last update: Aug 2020)                                              */
/* More info: https://github.com/sglvladi/meshquitto                                  */
/* ================================================================================== */

#include <DallasTemperature.h>
#include<ESP8266WiFi.h>
#include <painlessMesh.h>
#include <AES.h>
#include "./libraries/HashMap/HashMap.h" 
#include "./libraries/CustomList/CustomList.h"

// Define GPIO pins
#define   LED_PIN             D1      // GPIO pin of connected LED
#define   LED_TOPIC           "/LED1"
#define   TEMP_PIN            D4      // GPIO pin of connected DS18B20 temp sensor
#define   TEMP_TOPIC          "/TEMP1"

// Mesh network details
#define   MESH_SSID       "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_ENCRYPT    "sampleEncryptKey"
#define   MESH_PORT       5555
#define   GW_ID           2143537872

// How often the node should publish the temperature
#define SEND_INTERVAL 1000

// Mesh intantiation
painlessMesh  mesh;

// Storage containers and buffers
bool calc_delay = false;
CustomList<uint32_t> nodes;
uint32_t sendMessageTime = 0;

// AES Encryption parameters
const String                  AES_KEY   = "0123456789010123";
const unsigned long long int  AES_IV    = 36753562;
const int                     AES_BITS  = 256;

long lastMsg = millis();

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
/* Reads and returns the device MAC adress as a String                  */
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
/* Checks if SEND_INTERVAL has passed and if true, reads from DS18B20   */ 
/* sensor and publishes temperature                                     */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int readTemp(int port)                                    
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
{
  if((millis()-lastMsg>=SEND_INTERVAL)){
    lastMsg = millis();
    // Setup a oneWire instance to communicate with any OneWire devices 
    // (not just Maxim/Dallas temperature ICs)
    OneWire oneWire(port);
    // Pass our oneWire reference to Dallas Temperature.
    DallasTemperature sensors(&oneWire);
    // Start up the library
    sensors.begin();
    Serial.print(" Requesting temperatures...");
    sensors.requestTemperaturesByIndex(0); 
    Serial.print("Temperature: ");
    double temperature = sensors.getTempCByIndex(0);
    Serial.print(temperature); Serial.println(" *C"); 
    char temp[] = "";
    dtostrf(temperature,1, 2, temp);
    DynamicJsonDocument jsonDoc(1024);
    jsonDoc["topic"] = TEMP_TOPIC;
    jsonDoc["payload"] = String(temperature);
    String json_msg;
    serializeJson(jsonDoc, json_msg);
    Serial.println("Sending.....");
    uint32_t dest = GW_ID;
    String encrypted_msg = AES_encrypt(json_msg, AES_KEY);
    mesh.sendSingle( dest, encrypted_msg );

    return sensors.getDeviceCount();
  }
  return 0;
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


/*************************************************************************/
/* Performs AES encryption on a given text. Key *must* be 16 bytes.      */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
String AES_encrypt(String plain, String key)
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
{
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
String AES_decrypt(String cipher, String key)
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
{
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


/************************************************************************/
/* Mesh received callback: Receives a message and if the topic of the   */ 
/* received message matches LED_TOPIC, then sets the LED ON, if payload */
/* is equal to 1, and OFF, if payload is equal to 0                     */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void receivedCallback( uint32_t from, String &msg ) {
  String decrypted_msg = AES_decrypt(msg, AES_KEY);
  Serial.printf("startHere: Received from %d msg=",from); Serial.println(decrypted_msg);
  String response = "Message received! ID: ";
  StaticJsonDocument<500> jsonDoc;
  deserializeJson(jsonDoc, decrypted_msg);
  String topic = jsonDoc["topic"];
  String payload = jsonDoc["payload"];
  Serial.println("Message arrived...");
  Serial.println("Topic: "+topic);
  Serial.println("Payload: "+payload);

  // If topic matches LED_TOPIC
  if(topic == LED_TOPIC){
    // Switch on the LED if an 1 was received as first character
    int requested_state = int((char)payload[0]=='1'? 1:0);
    Serial.println("Turning LED "+String(((requested_state==1) ? "ON":"OFF")));
    digitalWrite(LED_PIN, requested_state);
  }
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*****************************************************************************/
/* Computes and returns the set difference between two CustomList<uint32_t>  */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
CustomList<uint32_t> getDifference(CustomList<uint32_t> arr1, CustomList<uint32_t> arr2){
  
  int m = arr1.size();
  int n = arr2.size();
  CustomList<uint32_t> diff;
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
  return diff;
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/*******************************************************************************/
/* Computes and returns the set intersection between two CustomList<uint32_t>  */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
CustomList<uint32_t> getIntersection(CustomList<uint32_t> arr1, CustomList<uint32_t> arr2){
  
  int m = arr1.size();
  int n = arr2.size();
  CustomList<uint32_t> intersect;
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
CustomList<uint32_t> getLostConnections(CustomList<uint32_t> nodes, CustomList<uint32_t> old_nodes){
  
  CustomList<uint32_t> diff = getDifference(old_nodes, nodes);
  CustomList<uint32_t> lostCons = getIntersection(diff, old_nodes);
  return lostCons;
  
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/**************************************************************************/
/* Given two lists, before and after a new scan, computes and returns all */
/* newly connected nodes.                                                 */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
CustomList<uint32_t> getNewConnections(CustomList<uint32_t> nodes, CustomList<uint32_t> old_nodes){
  
  CustomList<uint32_t> diff = getDifference(old_nodes, nodes);
  CustomList<uint32_t> newCons = getIntersection(diff, nodes);
  return newCons;
  
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

void changedConnectionCallback() {
 
    Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());
    CustomList<uint32_t> old_nodes = nodes;
    nodes = mesh.getNodeList();
    nodes.sort();
    Serial.printf("Num nodes: %d\n", nodes.size());

    // Print all available connections to nodes
    Serial.printf("Connection list:");
    CustomList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
        Serial.printf(" %u", *node);
        node++;
    }
    Serial.println();

    // Get and print lost connections to nodes
    CustomList<uint32_t> lostCons = getLostConnections(nodes, old_nodes);
    if(lostCons.size()>0){
      Serial.printf("Lost Connections: ");
      node = lostCons.begin();
      while (node != lostCons.end()) {
          Serial.printf(" %u", *node);
          node++;
      }
      Serial.println();
    }
    // ========================>
    //    DO STUFF .....
    // ========================>
    // Free up the memory
    lostCons.~CustomList<uint32_t>();
    
    calc_delay = true;
    old_nodes = nodes;
}


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


void setup() {
  Serial.begin(115200);
  Serial.print("Chip ID: "); Serial.println(ESP.getChipId());
  pinMode( LED_PIN, OUTPUT );
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  
  mesh.init(MESH_SSID, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);

  randomSeed( analogRead( A0 ) );  
}

void loop() {
  mesh.update();
  readTemp(D4);
}

