uint32_t lastTime;
const int Setpoint=15;
double kp,ki,kd,lastErr;
int32_t  Output;
double errSum;

int SampleTime=1000;

void setup()
{
        Serial.begin(9600);
}


void loop()
{
        tunings(100,100,2000);
        compute();
        setSampleTime(0);
}

void compute()
{
        int16_t Input=analogRead(A0);
        Input=map(Input,0,1024,0,50);
        Input=constrain(Input,0,50);

        uint32_t now=millis();
        int timeChange=(now-lastTime);
        if(timeChange>=SampleTime){
                double error=Setpoint-Input;
                errSum+=(error*timeChange);
                double dErr=(error-lastErr)/timeChange;

                double proportional=kp*error;
                double integral=ki*errSum;
                double derivative=kd*dErr;
                Output=(proportional+integral+derivative);
        //      Output=kp*error+ki*errSum+kd*dErr;
        //      Output=Output>>15;

                lastErr=error;
                lastTime=now;

                Serial.print(Input);
                Serial.print("\t");
                Serial.print(proportional);
                Serial.print("\t");
                Serial.print(integral);
                Serial.print("\t");
                Serial.print(derivative);
                Serial.print("\t");
                Serial.println(Output);
        }
}

void tunings(double Kp,double Ki,double Kd)
{
        double SampleTimeinSec=((double)SampleTime)/1000;
        kp = Kp;
        ki = Ki*SampleTimeinSec;
        kd = Kd/SampleTimeinSec;
}

void setSampleTime(int NewSampleTime)
{
        if(NewSampleTime>0){
                double ratio=(double)NewSampleTime/(double)SampleTime;
                ki*=ratio;
                kd/=ratio;
                SampleTime=(unsigned long)NewSampleTime;
        }
}
