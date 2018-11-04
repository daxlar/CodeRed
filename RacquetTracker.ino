#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#define FOREHAND 1
#define BACKHAND 2

const uint8_t MPU_addr=0x68;
//const char* ssid     = "MySpectrumWiFi60-2G";
//const char* password = "betterocean560";
//const char* host = "192.168.1.15";
//const char* ip = "192.168.1.15";
const char* ip = "172.25.146.248";
const char* ssid     = "UHWireless";
/*
const float MPU_GYRO_250_SCALE = 131.0;
const float MPU_GYRO_500_SCALE = 65.5;
const float MPU_GYRO_1000_SCALE = 32.8;
*/
const float MPU_GYRO_2000_SCALE = 16.4;
/*
const float MPU_ACCL_2_SCALE = 16384.0;
const float MPU_ACCL_4_SCALE = 8192.0;
const float MPU_ACCL_8_SCALE = 4096.0;
*/
const float MPU_ACCL_16_SCALE = 2048.0;


int16_t accl_x,accl_y,accl_z,gyro_x,gyro_y,gyro_z, temperature;
int16_t offset_accl_x,offset_accl_y,offset_accl_z,offset_gyro_x,offset_gyro_y,offset_gyro_z;
int16_t STATE = 0;



struct data{
  int16_t accl_x;
  int16_t accl_y;
  int16_t accl_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

//struct data* offset = (struct data*)malloc(sizeof(data));
//WiFiClient client;
WiFiUDP Client;

void setup() {
  
  pinMode(2, OUTPUT);     //set up on board led
  pinMode(16, OUTPUT);    //set up on board led #2
  digitalWrite(2, HIGH);      //led is negative logic
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
  /*
  delay(1000);
  const int httpPort = 55555;
  while(!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    Serial.println(client.connect(host, httpPort));
  }
  */
  
  Wire.begin();
  setup_mpu_6050_registers();
  calc_mpu_6050_offs();
  digitalWrite(2, LOW);
  
}

void loop() {
  read_mpu_6050_data();
  scof_mpu_6050_data();
  
  float acc_total_vector = sqrt((accl_x*accl_x)+(accl_y*accl_y)+(accl_z*accl_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  float angle_pitch_acc = asin((float)(accl_y)/acc_total_vector)* 57.296;       //Calculate the pitch angle
  float angle_roll_acc = asin((float)(accl_x)/acc_total_vector)* -57.296;       //Calculate the roll angle

 
  /*
  Serial.print("pitch angle: ");
  Serial.print(angle_pitch_acc);
  Serial.print("roll angle: ");
  Serial.print(angle_roll_acc);
  Serial.print("z axis");
  Serial.println( accl_z);
  */
  
  
  if(STATE == 0){
    if(((angle_pitch_acc > -30) && (angle_pitch_acc < -5)) && (( angle_roll_acc < 80) && ( angle_roll_acc > -5))){
      if(accl_z < 0){
        STATE = BACKHAND;
      }
    }
    if(((angle_pitch_acc > -40) && (angle_pitch_acc < 0)) && (( angle_roll_acc > 10) && ( angle_roll_acc < 80))){
      if(accl_z > 0){
         STATE = FOREHAND;
      }
    }
  }
  if(STATE == FOREHAND){
    if(((angle_pitch_acc > -30) && (angle_pitch_acc < -5)) && (( angle_roll_acc < 80) && ( angle_roll_acc > -5))){
      if(accl_z < 0){
        STATE = BACKHAND;
      }
    }
    Client.beginPacket(ip,6789);
    Client.write(1);
    Client.endPacket();
    delay(2);
    //client.write(FOREHAND);
    //Serial.println("forehand");
  }
  
  if(STATE == BACKHAND){
    if(((angle_pitch_acc > -40) && (angle_pitch_acc < 0)) && (( angle_roll_acc > 10) && ( angle_roll_acc < 80))){
      if(accl_z > 0){
         STATE = FOREHAND;
      }
    }
    Client.beginPacket(ip,6789);
    Client.write(2);
    Client.endPacket();
    delay(2);
    //client.write(BACKHAND);
    //Serial.println("backhand");
  }
  
}



void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-16g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x18);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (2000dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x18);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050

  
  while(Wire.available() < 14);                                       //Wait until all the bytes are received
  accl_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  accl_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  accl_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}

void calc_mpu_6050_offs(){                                                                // calculating the offsets of the gyro and accelerometer
  for(int16_t i = 0; i < 2000; i++){
    read_mpu_6050_data();
    offset_accl_x += accl_x;
    offset_accl_y += accl_y;
    offset_accl_z += accl_z;
    offset_gyro_x += gyro_x;
    offset_gyro_y += gyro_y;
    offset_gyro_z += gyro_z;
    yield();
  }
  offset_accl_x /= 2000;
  offset_accl_y /= 2000;
  offset_accl_z /= 2000;
  offset_gyro_x /= 2000;
  offset_gyro_y /= 2000;
  offset_gyro_z /= 2000;
  Serial.println("done");
}
void scof_mpu_6050_data(){                                            //SCaling and OFfsetting the raw gyro and accelerometer data
  /*
  //accl_x -= offset_accl_x;
  accl_x /= MPU_ACCL_16_SCALE;
  //accl_y -= offset_accl_y;
  accl_y /= MPU_ACCL_16_SCALE;
  //accl_z -= offset_accl_z;
  accl_z /= MPU_ACCL_16_SCALE;
  */
  gyro_x -= offset_gyro_x;
  gyro_x /= MPU_GYRO_2000_SCALE;
  gyro_y -= offset_gyro_y;
  gyro_y /= MPU_GYRO_2000_SCALE;
  gyro_z -= offset_gyro_z;
  gyro_z /= MPU_GYRO_2000_SCALE;
  
}









