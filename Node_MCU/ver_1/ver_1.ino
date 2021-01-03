
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Connect to the WiFi
const char* ssid = "HR";                           //!!!!!!!!!!!!!!!!!!!!!
const char* password = "123123123";                //!!!!!!!!!!!!!!!!!!!!!
const char* mqtt_server = "192.168.43.241";                 //!!!!!!!!!!!!!!!!!!!!!

String str = "";
WiFiClient AGV_FireBird;
PubSubClient client(AGV_FireBird);

void setup()
{
  Serial.begin(9600);
  delay(10);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("connecting WiFi ...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

 client.setServer(mqtt_server, 1883);
 client.setCallback(callback);

 str.reserve(10);

}
 

 
void callback(char* topic, byte* payload, unsigned int length) {
 
 Serial.print("Message arrived [");
 Serial.print(topic);
 Serial.print("] ");
 Serial.println();
 Serial.print("Sending Path to AGV........");
 delay(100);
 for (int i=0;i<length;i++) 
  {
  char receivedChar = (char)payload[i];
  Serial.write(receivedChar);
  }
  delay(2000);
  send_feedback();
  Serial.println();
}
 
 
void reconnect() {
 while (!client.connected()) {

 if (client.connect("ESP8266 Client1")) {
  client.subscribe("path_tx");
 } else {
  
  Serial.print("failed, rc=");
  Serial.print(client.state());
  Serial.println(" try again in 5 seconds");
  //Wait 5 seconds before retrying
  delay(5000);
  }
 }
}

char dum[10];



void loop()
{
  
 if (!client.connected()) {
   reconnect();
 }
 
 client.loop();
 
 while(Serial.available()>0)
 {
  char c= (char)Serial.read();
  str += c;
  int len;
  if(str.length()>=4){
    str.toCharArray(dum,10);
    client.publish("feedback",dum);
    str = "";
  }

 }

}



void send_feedback()
{
    client.publish("feedback","31N");
}
