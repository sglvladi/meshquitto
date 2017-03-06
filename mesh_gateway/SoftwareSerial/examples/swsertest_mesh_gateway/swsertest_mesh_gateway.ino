
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
#include <easyMesh.h>
#include <SoftwareSerial.h>
// some gpio pin that is connected to an LED... 
// on my rig, this is 5, change to the right number of your LED.
#define   LED             D1      // GPIO number of connected LED

#define   BLINK_PERIOD    1000000 // microseconds until cycle repeat
#define   BLINK_DURATION  100000  // microseconds LED is on for

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneeky"
#define   MESH_PORT       5555


#define WIFI_IRQ D3
#define MESH_IRQ D4

easyMesh  mesh;

uint32_t sendMessageTime = 0;

SoftwareSerial swSer(D2, D1, false, 1024);


bool _sending = false;
bool _receiving = false;

void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %d msg=%s\n", from, msg.c_str());
  sendToWiFi(msg, msg.length());
}

void newConnectionCallback( bool adopt ) {
  Serial.printf("startHere: New Connection, adopt=%d\n", adopt);
}

void sendToWiFi(String payload, unsigned int length){
    _sending = true;
    Serial.print("Forwarding message to WiFi GW");
    for (int i = 0; i < length; i++) {
      Serial.print(".");
      swSer.write((char)payload[i]);
    }
    Serial.println("DONE");

    // Pulse WIFI_IRQ to tell Mesh GW a message has been written to serial buffer
    digitalWrite(MESH_IRQ, LOW);
    delay(200);
    digitalWrite(MESH_IRQ, HIGH);
    _sending = false;
}

void receiveFromWiFi( void ){
  _receiving = true;
  Serial.println("WiFi GW interrupt detected");
  bool received = false;
  String swMessage;
  while (swSer.available() > 0) {
    char swChar = swSer.read();
    swMessage += swChar;
    received = true;
  }
  if(received){
    Serial.print("Received message from WiFi gateway: ");
    Serial.println(swMessage);
    mesh.sendBroadcast(swMessage);
  }
  else{
    Serial.println("Failed!");
  _receiving = false;
}

void setup() {
  Serial.begin(115200);
  swSer.begin(38400);
  Serial.println("\nSoftware serial test started");
  pinMode(MESH_IRQ, OUTPUT);
  digitalWrite(MESH_IRQ, HIGH);
  pinMode(WIFI_IRQ, INPUT);
  attachInterrupt(WIFI_IRQ, receiveFromWiFi, FALLING);
  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");

  pinMode( LED, OUTPUT );

//mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT );
  mesh.setReceiveCallback( &receivedCallback );
  mesh.setNewConnectionCallback( &newConnectionCallback );

  randomSeed( analogRead( A0 ) );  

}

void loop() {
  mesh.update();
  
  while (Serial.available() > 0) {
    swSer.write(Serial.read());
  }

}
