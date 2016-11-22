//Added PanError
//Added maxsonar class/object
//Remove motorspeed map and constrain.

#include <EEPROM.h>
#include <Wire.h>

//#define USE_MAXSONAR

#define leftMotorDirPin 2
#define leftMotorPWMPin 3
#define leftMotorBreakPin 4

#define rightMotorBreakPin 9
#define rightMotorDirPin 10
#define rightMotorPWMPin 11

byte I2Caddr;

int LMaxSensor,RMaxSensor;
float Pgain,Igain,Dgain;
const int minSpeed=-50;
const int maxSpeed=232;
byte leftMotorDir,rightMotorDir;

//Pixy PID computation
int followError,MotorSpeed;
float proportional;

//For troubleshooting delay()
uint32_t pastTime=0;

class sonarPID
{
public:
        sonarPID();
        double kp,ki,kd,ITerm,lastsonarReading,stopDistance,outMin,outMax,proportional,derivative;
        void maxSonarTunings(double Kp,double Ki,double Kd);
        void setLimits(double Min,double Max);
        void maxSonarCompute(double sonarReading);
        int32_t MotorSpeed,Output;
};

sonarPID leftSonar,rightSonar;

sonarPID::sonarPID()
{
        stopDistance=12;
}

void sonarPID::maxSonarTunings(double Kp,double Ki,double Kd)
{
        kp=Kp;
        ki=Ki;
        kd=Kd;
}

void sonarPID::setLimits(double Min,double Max)
{
        if(Min>Max)return;
        outMin=Min;
        outMax=Max;
        if(MotorSpeed>outMax){
                MotorSpeed=outMax;
        }
        else if(MotorSpeed<outMin){
                MotorSpeed=outMin;
        }
        if(ITerm>outMax){
                ITerm=outMax;
        }
        else if(ITerm<outMin){
                ITerm=outMin;
        }
}

void sonarPID::maxSonarCompute(double sonarReading)
{
        double error=sonarReading-stopDistance;
        ITerm+=(ki*error);
        if(ITerm>outMax){
                ITerm=outMax;
        }
        else if(ITerm<outMin){
                ITerm=outMin;
        }
        double dsonarReading=(sonarReading-lastsonarReading);

        proportional=kp*error;
        derivative=kd*dsonarReading;
        Output=kp*error+ITerm-kd*dsonarReading;
        MotorSpeed=Output>>3;
        if(MotorSpeed>outMax){
                MotorSpeed=outMax;
        }
        else if(MotorSpeed<outMin){
                MotorSpeed=outMin;
        }

        lastsonarReading=sonarReading;
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
        Wire.requestFrom(0x06,12);
        if(Wire.available()==12){
                LMaxSensor=Wire.read()<<8|Wire.read();
                RMaxSensor=Wire.read()<<8|Wire.read();
                Pgain=Wire.read()<<8|Wire.read();
                Igain=Wire.read()<<8|Wire.read();
                Dgain=Wire.read()<<8|Wire.read();
		followError=Wire.read()<<8|Wire.read();
        }
        else {
                digitalWrite(leftMotorBreakPin,1);
                digitalWrite(rightMotorBreakPin,1);
                Serial.println("Cannot connect to Master");
        }
}

void forward(int leftSpeed, int rightSpeed)
{
        if(leftSpeed>0){
                leftMotorDir=0;
        }
        else{
                leftMotorDir=1;
        }
        if(rightSpeed>0){
                rightMotorDir=0;
        }
        else{
                rightMotorDir=1;
        }
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

	#ifdef USE_MAXSONAR
        	leftSonar.setLimits(minSpeed,maxSpeed);
        	leftSonar.maxSonarTunings(Pgain,Igain,Dgain);
        	leftSonar.maxSonarCompute(LMaxSensor);
        	rightSonar.setLimits(minSpeed,maxSpeed);
        	rightSonar.maxSonarTunings(Pgain,Igain,Dgain);
        	rightSonar.maxSonarCompute(RMaxSensor);
        	forward(leftSonar.MotorSpeed,rightSonar.MotorSpeed);
	#endif
	
	#ifndef USE_MAXSONAR
		proportional=Pgain*followError;
		MotorSpeed=Pgain*followError;
	#endif

	troubleShoot();
}

void troubleShoot()
{
        uint32_t currentTime;
        const int interval=1000;
        if((currentTime=millis()-pastTime)>=interval){
                Serial.println(followError);
	/*	
		Serial.print("\t");
		Serial.print(Pgain);
		Serial.print("\t");
		Serial.print(proportional);
		Serial.print("\t");
		Serial.println(MotorSpeed);
                
                Serial.print(LMaxSensor);
                Serial.print("\t");
                Serial.print(Pgain);
                Serial.print("\t");
                Serial.print(Igain);
                Serial.print("\t");
                Serial.print(Dgain);
                Serial.print("\t");
                Serial.print(leftSonar.proportional);
                Serial.print("\t");
                Serial.print(leftSonar.ITerm);
                Serial.print("\t");
                Serial.print(leftSonar.derivative);
                Serial.print("\t");
                Serial.println(leftSonar.MotorSpeed);
                */
                pastTime=millis();
        }
}
