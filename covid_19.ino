// import library for the bluetooth
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
// import wire library for i2c communication used to communicate with mpu6050
#include<Wire.h>

int LED_BUILTIN = 2; // pin 2 have a builtin led in esp32
const int MPU_addr=0x68; // MPU6050 address
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // variable initialization

int minVal=265;
int maxVal=402;

double x; // to store vale of angle on x axis
double y; // to store vale of angle on y axis
double z; // to store vale of angle on z axis
BluetoothSerial SerialBT;
const int analogIn = 34; // temperature sensor is attached at analog pin 34
int RawValue= 0;
double Voltage = 0;
double tempC = 0;
double tempF = 0;

void setup() {
pinMode (LED_BUILTIN, OUTPUT); // set LED_BUILTIN as output
Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  RawValue = analogRead(analogIn); // to store raw value of temperature 
  Voltage = (RawValue / 2048.0) * 3300; // 5000 to get millivots. 
  tempC = Voltage * 0.1; // temp in celcius
  tempF = int((tempC * 1.8) + 32); // conver to F
  //Serial.print("\t Temperature in F = ");
  //Serial.println(tempF,1);
  
  SerialBT.print(tempF);
  delay(1000);
  if(tempF > 90){
    SerialBT.print("A");
    delay(1000);
  }

Wire.beginTransmission(MPU_addr);
Wire.write(0x3B); // sending register value 0x3B to receive the acceleration values
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read(); // get value of acceleration on x axis
AcY=Wire.read()<<8|Wire.read(); // get value of acceleration on y axis
AcZ=Wire.read()<<8|Wire.read(); // get value of acceleration on z axis
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);

x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); // angle on x axis in degree
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); // angle on y axis in degree
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI); // angle on z axis in degree
/* printing values on serial monitor*/
Serial.print("AngleX= ");
Serial.println(x);

Serial.print("AngleY= ");
Serial.println(y);

Serial.print("AngleZ= ");
Serial.println(z);
Serial.println("-----------------------------------------");
/* setting the angle for detecting the position of hand when it comes near to the face and glow the blue led on esp 32*/
if((x > 200 & x< 260) & (y > 70 & y < 120) & (z >320 & z < 360)){
  Serial.print("alert");
  digitalWrite(LED_BUILTIN, HIGH);
}
else{
  digitalWrite(LED_BUILTIN, LOW);
}
  
  //delay(20);
}
