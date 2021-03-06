#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <SPI.h>
#define CS_PIN 15 
#define HIT 3
#define NOHIT 0

uint8_t hit = 3;
uint8_t sendbuffer[256];
uint8_t tester[5] = { 3,3,3,3,3 };
uint8_t tester2[127];
uint16_t geophone = 0;
//const char* ssid     = "MySpectrumWiFi60-2G";
//const char* password = "betterocean560";
const char* ssid     = "UHWireless";

//const char* ip = "192.168.1.15";
const char* ip = "172.25.146.248";
uint16_t state = NOHIT;
uint16_t current_time = 0;

//WiFiClient client;
WiFiUDP Client;

void setup() {
  pinMode(16, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  digitalWrite(16, HIGH);
  Serial.begin(115200);

  
  
  //WiFi.begin(ssid, password);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".....");
  }
  Serial.println("i'm connected");
  digitalWrite(16, LOW);
  delay(1000);
  
  /*
  const int httpPort = 55555;
  while(!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    Serial.println(client.connect(host, httpPort));
  }
  */

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setFrequency(10000); // 1 MHz -- remove line when running on Arduino UNO
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  digitalWrite(16, LOW);
}

void loop() {



  /*
  Client.beginPacket(ip,6789);
  Client.write(3);
  Client.endPacket();
  delay(2);
  */
      
  /*
  for(uint8_t i = 0; i < 127; i++){
    geophone = adcRead(0);
    if(geophone > 700){
      //sendbuffer[i] = hit;
      tester2[i] = hit;
      digitalWrite(2, LOW);
    }else{
      //sendbuffer[i] = 0;
      tester2[i] = 0;
    }
  }
  */
  
  geophone = adcRead(0);
  if(geophone > 500){
      Client.beginPacket(ip,6789);
      Client.write(3);
      Client.endPacket();
      delay(2);
  }
  

  /*
  Client.beginPacket(ip,6789);
  Client.write(tester2, 127);
  //Client.write(tester, 5);
  Client.endPacket();
  yield();
  */


  /*
  geophone = adcRead(0);
  if(geophone > 620){
    udpsend(hit);
    digitalWrite(2, LOW);
  }
  */
  
  /*
  while(state == NOHIT){
    geophone = adcRead(0);
    digitalWrite(2, HIGH);
    yield();
    if(geophone > 700){
      digitalWrite(2, LOW);
      state = HIT;
      //current_time = millis();
    }
    //current_time = millis();
    yield();
    
  }
  current_time = millis();
  
  while(state == HIT){
    
    for(uint8_t i = 0; i< 20; i++){
      udpsend(hit);
    }
    
    
    udpsend(hit);
    //state = NOHIT;
    if(millis() > current_time + 700){
      state = NOHIT;
    }
    
    yield();
  }
  */
  
  
  
  

}

/*
void udpsend_buffer(uint8_t data[]){
  
  Client.beginPacket(ip, 6789);
  Client.write(data, 512);
  Client.endPacket();
  yield();

}
*/
void udpsend(uint8_t data){
  Client.beginPacket(ip,6789);
  Client.write(data);
  Client.endPacket();
  yield();
}


int adcRead(int channel) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x01);
  uint8_t r1 = SPI.transfer((channel + 8) << 4);
  uint8_t r2 = SPI.transfer(0);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  return ((r1 & 3) << 8) + r2;
}








