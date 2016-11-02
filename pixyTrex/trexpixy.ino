//Added maxsonar class/object
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

int LMaxSensor;
int RMaxSensor;
byte leftMotorDir,rightMotorDir;

//For troubleshooting delay()
uint32_t pastTime=0;

class sonarPID
{
public:
	sonarPID();
	double kp,ki,kd,ITerm,lastsonarReading,stopDistance,derivative;
	void maxSonarTunings(double Kp,double Ki,double Kd);
	void maxSonarCompute(double sonarReading);
	int32_t MotorSpeed;
	uint32_t previousTime;
};

sonarPID leftSonar,rightSonar;

sonarPID::sonarPID()
{
	stopDistance=8;
	previousTime=0;
}

void sonarPID::maxSonarTunings(double Kp,double Ki,double Kd)
{
        kp=Kp;
        ki=Ki;
        kd=Kd;
}

void sonarPID::maxSonarCompute(double sonarReading)
{
        unsigned long now=millis();

        double error=stopDistance-sonarReading;
        ITerm+=(ki*error);;
        double dsonarReading=(sonarReading - lastsonarReading);

	derivative=kd*dsonarReading;
        int32_t Output=kp*error+ITerm-kd*dsonarReading;
        MotorSpeed=Output>>3;

        lastsonarReading=sonarReading;
        previousTime=now;
}

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

void getMasterData()
{
        Wire.requestFrom(0x06,4);
        if(Wire.available()==4){
                LMaxSensor=Wire.read()<<8|Wire.read();
                RMaxSensor=Wire.read()<<8|Wire.read();
        }
        else {
                Serial.println("Cannot connect to Master");
        }
}

void forward(int leftSpeed, int rightSpeed)
{
        digitalWrite(leftMotorBreakPin,0);
        digitalWrite(rightMotorBreakPin,0);
        digitalWrite(leftMotorDirPin,leftMotorDir);
        digitalWrite(rightMotorDirPin,rightMotorDir);
        analogWrite(leftMotorPWMPin,leftSpeed);
        analogWrite(rightMotorPWMPin,rightSpeed);
}

void loop()
{
        delay(100);
        getMasterData();

	leftSonar.maxSonarTunings(50,0,100);
	leftSonar.maxSonarCompute(LMaxSensor);

//	forward(leftSonar.MotorSpeed,leftSonar.MotorSpeed);

	troubleShoot();
}

void troubleShoot()
{
	uint32_t currentTime;
	const int interval=1000;
	if((currentTime=millis()-pastTime)>=interval){
		Serial.print(LMaxSensor);
		Serial.print("\t");
		Serial.print(leftSonar.ITerm);
		Serial.print("\t");
		Serial.print(leftSonar.derivative);
		Serial.print("\t");
        	Serial.println(leftSonar.MotorSpeed);
		pastTime=millis();
	}
}
