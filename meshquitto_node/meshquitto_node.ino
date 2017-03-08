#include <DallasTemperature.h>
#include<ESP8266WiFi.h>

//************************************************************
// this is a simple example that uses the easyMesh library
//
// 1. blinks led once for every node on the mesh
// 2. blink cycle repeats every BLINK_PERIOD
// 3. sends a silly message to every node on the mesh at a random time betweew 1 and 5 seconds
// 4. prints anything it recieves to Serial.print
// 
//
//************************************************************
#include <painlessMesh.h>
#include <AES.h>
#include <HashMap.h> 


// some gpio pin that is connected to an LED... 
// on my rig, this is 5, change to the right number of your LED.
#define   LED             D1      // GPIO number of connected LED

#define   BLINK_PERIOD    1000000 // microseconds until cycle repeat
#define   BLINK_DURATION  100000  // microseconds LED is on for

#define   MESH_SSID       "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_ENCRYPT    "sampleEncryptKey"
#define   MESH_PORT       5555
#define   GW_ID           2143537872

#define NC_ID 0x00
#define WIND_ID 0x01
#define VOLT_ID 0x02
#define CURR_ID 0x03
#define HUM_ID 0x04
#define TEMP_ID 0x05
#define DOOR_ID 0x06
#define RELAY_ID 0x07
#define LIGHT_ID 0x08

#define SEND_INTERVAL 100

painlessMesh  mesh;
bool calc_delay = false;
SimpleList<uint32_t> nodes;
uint32_t sendMessageTime = 0;

String key = "0123456789010123";
int bits = 256;
unsigned long long int my_iv = 36753562;

long lastMsg[17] = {millis(), millis(), millis(), millis(),
                    millis(), millis(), millis(), millis(),
                    millis(), millis(), millis(), millis(),
                    millis(), millis(), millis(), millis(), millis()};

// {Identifier => Sensor type} Map 
CreateHashMap(sensTypeMap, String, int, 8);

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
/* Reads from DS18B20 sensor interfase given one-wire                   */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
int readTemp(int port, int sensor_id)                                    
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
{
  if((millis()-lastMsg[sensor_id]>=SEND_INTERVAL)){
    lastMsg[sensor_id] = millis();
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
    String encrypted_msg = AES_encrypt(json_msg, key);
    mesh.sendSingle( dest, encrypted_msg );

    return sensors.getDeviceCount();
  }
  return 0;
}
/************************************************************************/

void printHeap(){
  Serial.print("Free Heap: "); Serial.println(ESP.getFreeHeap());
}

String AES_encrypt(String plain, String key) {
  /* Local AES instance (uncomment to recreate problem) */
  AES aes ; 
  aes.set_IV(my_iv);
  byte * plain_buf = (unsigned char*)plain.c_str();
  byte * key_buf = (unsigned char*)key.c_str();
  // add padding where appropriate
  int cipher_length = (plain.length()+1 < 16) ? 16 : (plain.length()+1) + (16 - (plain.length()+1) % 16);
  byte cipher_buf[cipher_length];
  aes.do_aes_encrypt(plain_buf, plain.length() + 1, cipher_buf, key_buf, bits);
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

String AES_decrypt(String cipher, String key) {
  printHeap();
  /* Local AES instance (uncomment to recreate problem) */
  AES aes ; 
  //Serial.println(cipher);
  aes.set_IV(my_iv);
  byte cipher_buf[cipher.length()/2-2];
  int j = 0;
  for(int i=0;i<=cipher.length()-6;i+=2){
    //Serial.print(cipher.substring(i,i+2));
    cipher_buf[j] = char(strtoul(cipher.substring(i,i+2).c_str(), NULL, 16));
    //Serial.print(char(cipher_buf[j]));
    j++;
  }
  printHeap();
  //aes.printArray(cipher_buf, false);
  int plain_size = cipher.substring(cipher.length()-4).toInt();
  //Serial.println(plain_size);
 // byte * cipher_buf = (unsigned char*)cipher.c_str();
  byte * key_buf = (unsigned char*)key.c_str();
  int plain_length = cipher.length();
  byte plain_buf[plain_length];
  aes.do_aes_decrypt(cipher_buf, cipher.length()/2-2, plain_buf, key_buf, bits);
  String plain = aes.printToString(plain_buf, plain_size);
  //Serial.println(sizeof(plain_buf)/sizeof(plain_buf[0]));
  //Serial.println(plain.length());
  printHeap();
  return plain;
}



void receivedCallback( uint32_t from, String &msg ) {
  String decrypted_msg = AES_decrypt(msg, key);
  Serial.printf("startHere: Received from %d msg=",from); Serial.println(decrypted_msg);
  String response = "Message received! ID: ";
  char contentBuffer[500];
  decrypted_msg .toCharArray(contentBuffer,500);
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& rootFS2 = jsonBuffer.parseObject(contentBuffer);
  String topic = rootFS2["topic"];
  String payload = rootFS2["payload"];
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  int topic_length = topic.length();
  int subtopicNo = 0;
  for (int i=0; i<topic_length; i++){
    if (String(topic.charAt(i)) == "/"){
      subtopicNo++;
    }
  }
  //Serial.print("subotpicNo: "); Serial.println(subtopicNo);
  String subtopics[subtopicNo];
  bool topicParsed = false;
  int parserPos = 0;
  int i = 0;
  while(!topicParsed){
    //yield();
    int startPos = parserPos+1;
    //Serial.print("indexOf: "); Serial.println(topic_str.indexOf("/", startPos));
    int endPos = topic.indexOf("/", startPos);
    if (endPos==-1){
      endPos = topic.length();
      topicParsed = true;
    }
    subtopics[i] = topic.substring(startPos, endPos);
    parserPos = endPos;
    i++;
  }
  // 0:device_id, 1:"sensors", 2:sensor_id, 3: "input"/"output", 4: "value"/"min"/"max" etc.
  String sensor_id = String(subtopics[2]); 
  //String(topic).substring(String(topic).lastIndexOf("/")+1);
  int sensor_type = sensTypeMap[sensor_id.substring(0,1)];
  int channel_id = (sensor_id.substring(1)).toInt();
  Serial.print("Channel_id: "); Serial.println(channel_id);
  Serial.println();
  if (sensor_type == RELAY_ID){
    // Switch on the LED if an 1 was received as first character
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
        Serial.println("Turning relay ON");
        pinMode(D1,OUTPUT);
        digitalWrite(LED, LOW);
        //delay(500);
        break;
        
      }
      case(1):{
        Serial.println("Turning relay OFF");
        pinMode(D1,OUTPUT);
        digitalWrite(LED, HIGH);
        break;
      }
      default:{
        Serial.println("No matching action");
        break;
      }
    }
    String topic_back = "/" + getMac()+ "/sensors/"+sensor_id+"/output/value";
    Serial.println("Sending.....");
    JsonObject& rootFS2 = jsonBuffer.createObject();
    rootFS2["topic"] = topic_back;
    rootFS2["payload"] = digitalRead(LED);
    String json_msg;
    rootFS2.printTo(json_msg);
    Serial.println("Sending.....");
    uint32_t dest = GW_ID;
    String encrypted_msg = AES_encrypt(json_msg, key);
    mesh.sendSingle( dest, encrypted_msg );
  } 
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

SimpleList<uint32_t> getDifference(SimpleList<uint32_t> arr1, SimpleList<uint32_t> arr2){
  
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
  return diff;
}

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

SimpleList<uint32_t> getLostConnections(SimpleList<uint32_t> nodes, SimpleList<uint32_t> old_nodes){
  
  SimpleList<uint32_t> diff = getDifference(old_nodes, nodes);
  SimpleList<uint32_t> lostCons = getIntersection(diff, old_nodes);
  return lostCons;
  
}

SimpleList<uint32_t> getNewConnections(SimpleList<uint32_t> nodes, SimpleList<uint32_t> old_nodes){
  
  SimpleList<uint32_t> diff = getDifference(old_nodes, nodes);
  SimpleList<uint32_t> newCons = getIntersection(diff, nodes);
  return newCons;
  
}

void changedConnectionCallback() {
 
    Serial.printf("Changed connections %s\n", mesh.subConnectionJson().c_str());
    SimpleList<uint32_t> old_nodes = nodes;
    nodes = mesh.getNodeList();
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

    // Get and print lost connections to nodes
    SimpleList<uint32_t> lostCons = getLostConnections(nodes, old_nodes);
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
    lostCons.~SimpleList<uint32_t>();
    
    calc_delay = true;
    old_nodes = nodes;
}
 
void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
    Serial.printf("Delay to node %u is %d us\n", from, delay);
}

void setup() {
  Serial.begin(115200);
  // Set up sensor type hashMap
  sensTypeMap["W"] = 1; sensTypeMap["V"] = 2; sensTypeMap["C"] = 3; sensTypeMap["H"] = 4;
  sensTypeMap["T"] = 5; sensTypeMap["D"] = 6; sensTypeMap["R"] = 7; sensTypeMap["L"] = 8;
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

  randomSeed( analogRead( A0 ) );  
}

void loop() {
  mesh.update();
  readTemp(D4, 4);
  //Serial.println("\n Intersection:");
  //printIntersection(arr1, arr2, m, n);
  //Serial.println("\n Difference:");
  //printDifference(arr1, arr2, m, n);
  //Serial.print("Free heap: "); Serial.println(ESP.getFreeHeap());
  // run the blinky
//  bool  onFlag = false;
//  uint32_t cycleTime = mesh.getNodeTime() % BLINK_PERIOD;
//  for ( uint8_t i = 0; i < ( mesh.connectionCount() + 1); i++ ) {
//    uint32_t onTime = BLINK_DURATION * i * 2;    
//
//    if ( cycleTime > onTime && cycleTime < onTime + BLINK_DURATION )
//      onFlag = true;
//  }
//  digitalWrite( LED, onFlag );
//
//  // get next random time for send message
//  if ( sendMessageTime == 0 ) {
//    sendMessageTime = mesh.getNodeTime() + random( 1000000, 5000000 );
//  }
//
//  // if the time is ripe, send everyone a message!
//  if ( sendMessageTime != 0 && sendMessageTime < mesh.getNodeTime() ){
//    String msg = "Hello from node ";
//    msg += mesh.getNodeId();
//    //mesh.sendBroadcast( msg );
//    sendMessageTime = 0;
//  }
}


