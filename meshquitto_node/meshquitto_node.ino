/*=================================================================================== */
/* meshquito_node.ino                                                                 */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* Example implementation of a meshquitto node.                                       */
/*                                                                                    */
/* # Listens to:                                                                      */  
/*    (<Mqtt_gateway_subtopics>)+"/$MAC address$/sensors/R1/input/value" topic        */
/*   and turns LED (D1) ON if 1 is received and OFF otherwise.                        */
/* # Publishes every SEND_INTERVAL to:                                                */
/*    (<Mqtt_gateway_subtopics>)+"/$MAC address$/sensors/T4/output/value" topic       */
/*   the temperature read from DS18B20 (D4).                                          */
/*                                                                                    */
/* Created by Lyudmil Vladimirov, Feb 2017                                            */
/* More info: https://github.com/sglvladi/meshquitto                                  */
/* ================================================================================== */

#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <painlessMesh.h>
#include <AES.h>
#include <HashMap.h> 


// Define LED pin
#define   LED             D1

// Define send/publish interval
#define SEND_INTERVAL 2000

// Mesh network details
#define   MESH_SSID       "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_ENCRYPT    "sampleEncryptKey"
#define   MESH_PORT       5555
#define   GW_ID           2143537872


// Mesh intantiation
painlessMesh  mesh;

// AES Encryption parameters
const String                  AES_KEY   = "0123456789010123";
const unsigned long long int  AES_IV    = 36753562;
const int                     AES_BITS  = 256;

// Store last time a message was sent
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
/* Prints available heap memory to Serial                               */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void printHeap(){                                                       
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  Serial.print("Free Heap: "); Serial.println(ESP.getFreeHeap());
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


/************************************************************************/
/* Reads from DS18B20 sensor and publishes it.                          */
/* Example topic: /5C-CF-7F-13-91-A8/sensors/T4/output/value            */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int readTemp(int port, int sensor_id)                                    
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
{
  // If SEND_INTERVAL has been surpassed
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
    Serial.print("Temperature for sensor T");
    Serial.print(sensor_id);
    Serial.print(" is: ");
    double temperature = sensors.getTempCByIndex(0);
    Serial.print(temperature); Serial.println(" *C"); 
    char temp[] = "";
    dtostrf(temperature,1, 2, temp);
    DynamicJsonBuffer jsonBufferFS;
    JsonObject& rootFS2 = jsonBufferFS.createObject();
    rootFS2["topic"] = String(getMac()+"/sensors/T"+sensor_id+"/output/value");
    rootFS2["payload"] = String(temperature);
    String json_msg;
    rootFS2.printTo(json_msg);
    Serial.println("Sending.....");
    uint32_t dest = GW_ID;
    String encrypted_msg = AES_encrypt(json_msg, AES_KEY);
    mesh.sendSingle( dest, encrypted_msg );

    return sensors.getDeviceCount();
  }
  return 0;
}
/************************************************************************/


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


/************************************************************************/
/* Mesh received callback: Mqtt messages received by the Mesh gateway   */ 
/* whose topic starts with the MAC address of this device, will be sent */
/* to this device. This is were handling and processing of all messages */
/* is performed. Below is a simple routine which parses all subtopics,  */
/* such that different action can be performed when a different topic   */
/* is received.                                                         */
/* NOTE: Everything below can be easily changed to suit your needs.     */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void receivedCallback( uint32_t from, String &msg ) {

  // Decrypt the received message
  String decrypted_msg = AES_decrypt(msg, AES_KEY);
  Serial.printf("startHere: Received from %d msg=",from); Serial.println(decrypted_msg);
  String response = "Message received! ID: ";
  
  // Decode json message
  char contentBuffer[500];
  decrypted_msg .toCharArray(contentBuffer,500);
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& rootFS2 = jsonBuffer.parseObject(contentBuffer);
  String topic = rootFS2["topic"];
  String payload = rootFS2["payload"];
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  // Parse topic and extract a list of subtopics
  SimpleList<String> subtopics;
  bool topicParsed = false;
  int parserPos = 0;
  while(!topicParsed){
    int startPos = parserPos+1;
    int endPos = topic.indexOf("/", startPos);
    if (endPos==-1){
      endPos = topic.length();
      topicParsed = true;
    }
    subtopics.push_back(topic.substring(startPos, endPos));
    parserPos = endPos;
  }
  
  // Example structure of incoming message topic:
  // [(<Mqtt_gateway_subtopics>)* + /$MAC_Address$/sensors/$sensor_id$/input/value]
  // * (<Mqtt_gateway_subtopics>): Not visible within the mesh. This layer is "peeled" off and on by the mqtt_gateway.

  // Example: /5C-CF-7F-13-91-A8/sensors/R1/input/value
  String sensor_id = String(subtopics[2]); 
  String sensor_type = sensor_id.substring(0,1);
  int channel_id = (sensor_id.substring(1)).toInt();
  Serial.print("Channel_id: "); Serial.println(channel_id);
  Serial.println();

  // If sensor_type=="R", it means it is a meesage for a Relay, except we use an LED here.
  // (The reason is it can plug straight in-to our MQTT server/client application. You can change it to your needs)
  if (sensor_type == "R"){
    
    // Switch on the LED if a 1 was received as first character
    Serial.print("Channel id: ");
    Serial.println(channel_id);
    Serial.println("==================>");
    Serial.println("Relay message detected");
    int requested_state = int((char)payload[0]=='0'? 0:1);
    Serial.println("Requested state: "+String(((requested_state==1) ? "OFF":"ON")));
    //if(int(requested_state)==readRelay(channel_id)){
    Serial.print("..."); Serial.print(requested_state); Serial.println("...");
    switch(requested_state){
      case(0):{
        Serial.println("Turning LED OFF");
        digitalWrite(LED, LOW);
        break;
      }
      case(1):{
        Serial.println("Turning LED ON");
        digitalWrite(LED, HIGH);
        break;
      }
      default:{
        Serial.println("No matching action");
        break;
      }
    }

    // Now publish the new state of the Relay (LED) to /5C-CF-7F-13-91-A8/sensors/R1/output/value 
    // to let the user know that the Relay (LED) has been turned ON.
    String topic_back = "/" + getMac()+ "/sensors/"+sensor_id+"/output/value";
    Serial.println("Sending.....");
    JsonObject& rootFS2 = jsonBuffer.createObject();
    rootFS2["topic"] = topic_back;
    rootFS2["payload"] = digitalRead(LED);
    String json_msg;
    rootFS2.printTo(json_msg);
    Serial.println("Sending.....");
    uint32_t dest = GW_ID;
    String encrypted_msg = AES_encrypt(json_msg, AES_KEY);
    mesh.sendSingle( dest, encrypted_msg );
  } 
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


/************************************************************************/
/* Currently unused painlessMesh callbacks                              */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}
void changedConnectionCallback() {
    Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());
    SimpleList<uint32_t> nodes = mesh.getNodeList();
    nodes.sort();
    Serial.printf("Num nodes: %d\n", nodes.size());

    // Print all available connections to nodes
    Serial.printf("Connection list:");
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
        Serial.printf(" %u", *node);
        node++;
    }
    Serial.println();
}
void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}
void delayReceivedCallback(uint32_t from, int32_t delay) {
    Serial.printf("Delay to node %u is %d us\n", from, delay);
}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


void setup() {
  // Start Serial
  Serial.begin(115200);
  
   // Print initial debugging info
  Serial.println("\nMeshquitto Node started!");
  Serial.print("Chip ID: "); Serial.println(ESP.getChipId());
  
  pinMode( LED, OUTPUT );
  
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  mesh.init(MESH_SSID, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback); 
}

void loop() {
  mesh.update();
  readTemp(D4, 4);
}


