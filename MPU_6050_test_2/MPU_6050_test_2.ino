#include<Wire.h>
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void  setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(115200);
}
void  loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  /*
  Serial.print("Variable_1:"); Serial.println(AcX);
  Serial.print("Variable_2:"); Serial.println(AcY);
  Serial.print("Variable_3:"); Serial.println(AcZ); 
  */
  Serial.print("Variable_1:"); Serial.println(GyX);
  Serial.print("Variable_2:"); Serial.println(GyY);
  Serial.print("Variable_3:"); Serial.println(GyZ);
  
  delay(100);
}