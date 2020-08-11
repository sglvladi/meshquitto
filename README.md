# meshquitto
A simple Arduino project, which aims to provide a gateway between a mesh network of ESP8266's and a remote MQTT broker.

## How it works
<p align="center">
  <img src="https://github.com/sglvladi/meshquitto/blob/master/mesquitto_topology.jpg"/>
</p>

* The mesh network is created by making use of the amazing painlessMesh library (see [here](https://gitlab.com/BlackEdder/painlessMesh/wikis/home)).
* The **Meshquitto gateway** is composed of 2 ESP8266 devices (tested on D1-mini).
  * The first of the two devices (**_MQTT gateway_**) connects to an available WiFi network and consequently establishes a connection to a MQTT broker. Any messages received from the MQTT broker are forwarded to the Mesh gateway and vice versa.
  * The second (**_Mesh gateway_**) connects to the mesh network. Any messages received from the MQTT gateway are forwarded to the Mesh network and vice versa.
  * Since Software serial does not provide parity check functionality, CRC16 (see [here](https://github.com/vinmenn/Crc16) for library) is used to detect transmission errors. (**NOTE**: Retransmition is not currently implemented, thus corrupted messages are simply dropped).
* The topic structure of messages is shown in the image above:
  * \<MQTT_gateway_subtopics\> form the first (few) subtopic(s) of the MQTT topic, which are used solely for subscription and publishing identification of the gateway with the MQTT broker. 
  * Each of the mesh nodes (including the Mesh gateway) then are abstracted from the  \<MQTT_gateway_subtopics\> and simply "publish" and "subscribe" to the \<Mesh_network_subtopics\>.
  * Removal and concatenation of the \<MQTT_gateway_subtopics\> is handled by the MQTT gateway.
* AES encryption (see relevant library [here](https://github.com/sglvladi/AES)) has been imported in order to add an extra security layer to messages passed between nodes in the mesh.
