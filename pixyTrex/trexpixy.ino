//Added size
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

//Pixy PID computation
int32_t followError;

int32_t lmspeed=0;
int32_t rmspeed=0;
const int speedMin=-100;
const int speedMax=100;

union siz2bytes
{
	float sizf;
	char sizb[sizeof(float)];
};

int32_t size;

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
	siz2bytes si2b;
        Wire.requestFrom(0x06,16);
        if(Wire.available()==16){
                LMaxSensor=Wire.read()<<8|Wire.read();
                RMaxSensor=Wire.read()<<8|Wire.read();
                Pgain=Wire.read()<<8|Wire.read();
                Igain=Wire.read()<<8|Wire.read();
                Dgain=Wire.read()<<8|Wire.read();
		followError=Wire.read()<<8|Wire.read();
		si2b.sizb[0]=Wire.read();
		si2b.sizb[1]=Wire.read();
		si2b.sizb[2]=Wire.read();
		si2b.sizb[3]=Wire.read();
		size=(long)si2b.sizf;
        }
        else {
                Serial.println("Cannot connect to Master");
        }
}

void motors(int leftSpeed, int rightSpeed)
{
        digitalWrite(leftMotorBreakPin,size<1000);
        digitalWrite(rightMotorBreakPin,size<1000);
        digitalWrite(leftMotorDirPin,leftSpeed<0);
        digitalWrite(rightMotorDirPin,rightSpeed<0);
        analogWrite(leftMotorPWMPin,abs(leftSpeed));
        analogWrite(rightMotorPWMPin,abs(rightSpeed));
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
        	motors(leftSonar.MotorSpeed,rightSonar.MotorSpeed);
	#endif
	
	#ifndef USE_MAXSONAR
		lmspeed=constrain(speedMax-(size/70),speedMin,speedMax);
		rmspeed=constrain(speedMax-(size/70),speedMin,speedMax);
		motors(lmspeed,rmspeed);
	#endif

	troubleShoot();
}

void troubleShoot()
{
        uint32_t currentTime;
        const int interval=1000;
        if((currentTime=millis()-pastTime)>=interval){
		Serial.print(lmspeed);
		Serial.print("\t");
		Serial.print(rmspeed);
		Serial.print("\t");
		Serial.println(size);
                pastTime=millis();
        }
}
