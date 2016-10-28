//Added PID computation on slave
//Added troubleshoot function
//Remove motorspeed map and constrain.

#include <EEPROM.h>
#include <Wire.h>

#define INPUT_Potentiometer //For troubleshooting Input using potentiometer

#define leftMotorDirPin 2
#define leftMotorPWMPin 3
#define leftMotorBreakPin 4

#define rightMotorBreakPin 9
#define rightMotorDirPin 10
#define rightMotorPWMPin 11

byte I2Caddr;

//Sonar Pid variables
const int Setpoint=15;
int Input,lastInput,kp;
int32_t Output;

int MotorSpeed;

//For troubleshooting delay()
uint32_t pastTime=0;

void setup()
{
        Serial.begin(9600);
        Wire.begin(I2Caddress());
        uint8_t m[]={2,3,4,9,10,11};
        for(int i=0;i<6;i++){
                pinMode(m[i],OUTPUT);
        }
        Wire.onRequest(send_Sensor_Data);

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

void send_Sensor_Data()
{
        byte buffer[2];
        buffer[0]=MotorSpeed>>8;
        buffer[1]=MotorSpeed&0xFF;
        Wire.write(buffer,2);
}

void getSpeed()
{
        Wire.requestFrom(0x06,2);
        if(Wire.available()==2){
                Input=Wire.read()<<8|Wire.read();
                #ifdef INPUT_Potentiometer
                        if(Input>50){
                                Input=lastInput;
                        }
                        lastInput=Input;
                #endif
        }
        else {
                Serial.println("Cannot connect to Master");
        }
}

void sonarPIDcompute()
{
        double error=Input-Setpoint;

        Output=kp*error;
        MotorSpeed=Output>>3;


}

void sonarTunings(int Kp)
{
        kp=Kp;
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

        sonarPIDcompute();
        sonarTunings(100);

        troubleshoot();
}

void troubleshoot()
{
        uint32_t currentTime;
        const int interval=500;
        if((currentTime=millis()-pastTime)>=interval){
                Serial.print(Input);
                Serial.print("\t");
                Serial.print(Output);
                Serial.print("\t");
                Serial.println(MotorSpeed);
                pastTime=millis();
        }
}
