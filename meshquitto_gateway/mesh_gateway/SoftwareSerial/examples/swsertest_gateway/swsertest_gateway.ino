#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

SoftwareSerial swSer(D1, D2, false, 256);
// Update these with values suitable for your network.

const char* ssid     = "VM146928-2G";
const char* password = "gbwnhajc";
const char* mqtt_server = "192.168.1.3";

WiFiClientSecure espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

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
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    swSer.write((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266MeshGateway", "device", "didmsiskf0wdo")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("test", "hello world");
      // ... and resubscribe
      client.subscribe("test");
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
  swSer.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 8883);
  client.setCallback(callback);
  Serial.println("\nSoftware serial test started");

  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");

}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  while (swSer.available() > 0) {
    Serial.write(swSer.read());
  }

}
