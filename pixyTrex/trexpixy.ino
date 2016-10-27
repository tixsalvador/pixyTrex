//Remove motorspeed map and constrain.

#include <EEPROM.h>
#include <Wire.h>

#define leftMotorDirPin 2
#define leftMotorPWMPin 3
#define leftMotorBreakPin 4

#define rightMotorBreakPin 9
#define rightMotorDirPin 10
#define rightMotorPWMPin 11

byte I2Caddr;

int MotorSpeed;

void setup()
{
	Serial.begin(9600);
	Wire.begin(I2Caddress());
	uint8_t m[]={2,3,4,9,10,11};
	for(int i=0;i<6;i++){
		pinMode(m[i],OUTPUT);
	}
}

byte  I2Caddress()
{

	byte n=EEPROM.read(0);
	if(n!=0x55){
		EEPROM.write(0,0x55);
		EEPROM.write(1,0x07);
	}
	I2Caddr=EEPROM.read(1);
	return I2Caddr;
}

void getSpeed()
{
	Wire.requestFrom(0x06,2);
	if(Wire.available()==2){
		MotorSpeed=Wire.read()<<8|Wire.read();	
	}
	else {
		Serial.println("Cannot connect to Master");
	}
}

void forward(int leftSpeed, int rightSpeed)
{
	digitalWrite(leftMotorBreakPin,0);
	digitalWrite(rightMotorBreakPin,0);
	digitalWrite(leftMotorDirPin,0);
	digitalWrite(rightMotorDirPin,0);
	analogWrite(leftMotorPWMPin,MotorSpeed);
	analogWrite(rightMotorPWMPin,MotorSpeed);
}

void loop()
{	
	delay(100);
	getSpeed();
	forward(MotorSpeed,MotorSpeed);
	Serial.println(MotorSpeed);
}
