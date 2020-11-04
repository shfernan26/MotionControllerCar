

/*Next steps / current status: using single ints to give direction, speed changes not in effect yet

Long term: Send encoded numbers that represent magnitude of speed related to extent of MPU6050
movement
*/

//Remember to change board settings to Aruduino Nano if using it

//port 1421
//both nl and cr

//Connections
/*
HC05----> RXD->D5, TXD->D4, GND->GND, VCC->3.3V
MPU6050-----> VCC->3.3V, GND->3.3V, SCL->A5, SDA->A4, INT->D2
*/


//Portions copied from JR's code denoted as //JRstart & //JRend
//JRstart
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2  
#define LED_PIN 13 
bool blinkState = false;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3];           

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
//JRend

//////////MY ADDITIONS//////////


#include <SoftwareSerial.h>
SoftwareSerial BTSerial(4, 5); // RX | TX
// Connect the HC-05 TX to Arduino pin 4 RX. 
// Connect the HC-05 RX to Arduino pin 5 TX 

//GLOBAL VARIABLES
int YAW = 0;
int PITCH = 0;
int ROLL = 0;

int FB_value = 0; //determines which of Y, P or R controls which direction later on in code
int LR_value = 0;

bool initial_calibration = true;
bool ready_for_calibration = false;
bool calibrated = false;


int a = 0; // so datum, max and min can be calibrated on consecutive loops
int b = 0; //so only says 'ready for calibration once'

//Control values
int FB_datum = 0;
int FB_upper_bound = 0;
int FB_lower_bound = 0;
int FB_max = 0;
int FB_min = 0;

int LR_datum = 0;
int LR_upper_bound = 0;
int LR_lower_bound = 0;
int LR_max = 0;
int LR_min = 0;


//int rescaled_YAW = 0;

//for debugging purposes
char e = ' ';
int d = 0;
String readString;

//JRstart

volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // start the serial communication with the host computer
    Serial.begin(115200);
    Serial.println("Arduino with HC-05 is ready");
 
    // start communication with the HC-05 using 9600
    BTSerial.begin(9600);  
    Serial.println("BTserial started at 9600");
    
    while (!Serial); 
   
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); 
    while (!Serial.available());                 
    while (Serial.available() && Serial.read()); 

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

//JRend

}


void loop() {
//JRstart
   
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {}
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;   
           
           mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.println(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
        
//JRend

///////////////MY ADDITIONS/////////////////
            YAW = ypr[0] * 180/M_PI;
            PITCH = ypr[1] * 180/M_PI;
            ROLL = ypr[2] * 180/M_PI;
            
    }
            //-1 multiplication depends on orientation, in this case it means forward tilt is + and right tilt is +
           FB_value = -1*ROLL; 
           LR_value = -1*PITCH;


    
            if (initial_calibration)
            {
                if ((YAW>48) && (YAW<68) && (PITCH>-6) && (PITCH<4) && (ROLL>-2) && (ROLL < 2)) 
              {
                if (b == 0)
                {
                Serial.println ("Ready for calibration");
                //delay(500);
                ready_for_calibration = true;
                b++;
                }
              }
            }

            if (ready_for_calibration)
            { 
              initial_calibration = false;
              
              switch (a)
              {
              case 0:
                calibrate(FB_datum); 
                b=0;
                break;
              case 1:
                if (b==0)
                {
                Serial.print("FB_datum is ");
                Serial.println(FB_datum); // correct value only registered for previous switch case after it passes through all branches once, comes to this case on the next loop
                b++;
                }
                calibrate(FB_max);
                break;
              case 2:
                if (b==1)
                {
                Serial.print("FB max is ");
                Serial.println(FB_max);
                b++;
                }
                calibrate(FB_min);
                break;
              case 3:
                if (b==2)
                {
                Serial.print("FB min is ");
                Serial.println(FB_min);
                b++;
                }
                calibrate(LR_datum);
                break;
               case 4:
                if (b==3)
                {
                Serial.print("LR datum is ");
                Serial.println(LR_datum);
                b++;
                }
                calibrate(LR_max);
                break;
               case 5:
                if (b==4)
                {
                Serial.print("LR max is ");
                Serial.println(LR_max);
                b++;
                }
                calibrate(LR_min);
                break;
              case 6:
                Serial.print("LR min is ");
                Serial.println(LR_min);
                Serial.println("Calibration complete");
                calibrated = true;
                break;
     
              }
            }
            
            if (calibrated)
              {
                ready_for_calibration = false;
                
                FB_upper_bound = FB_datum + 5;
                FB_lower_bound = FB_datum - 5;

                LR_upper_bound = LR_datum + 5;
                LR_lower_bound = LR_datum - 5;
                
                
                if ((FB_value > FB_upper_bound) && (LR_value > LR_upper_bound))
                {
                  Serial.println("FORWARD-RIGHT");
                  send_to_serial(1);
                }
                else if ((FB_value > FB_upper_bound) && (LR_value < LR_lower_bound))
                {
                  Serial.println("FORWARD-LEFT");
                  send_to_serial(2);
                }
                else if ((FB_value < FB_lower_bound) && (LR_value > LR_upper_bound))
                {
                  Serial.println("BACKWARD-RIGHT");
                  send_to_serial(3);
                }
                else if ((FB_value < FB_lower_bound) && (LR_value < LR_lower_bound))
                {
                  Serial.println("BACKWARD-LEFT");
                  send_to_serial(4);
                }
                else if (FB_value > FB_upper_bound) 
                {
                  Serial.println("FORWARD");
                  send_to_serial(5);
                }
                else if (FB_value < FB_lower_bound) 
                {
                  Serial.println("BACKWARD");
                  send_to_serial(6);   
                }

                else if (LR_value > LR_upper_bound) 
                {
                  Serial.println("RIGHT");
                  send_to_serial(7);   
                }
                else if (LR_value < LR_lower_bound) 
                {
                  Serial.println("LEFT");
                  send_to_serial(8);   
                }
                else
                {
                  Serial.println("REST");
                  send_to_serial(9);   
                }
                             
              }
// for debugging
//    while (BTSerial.available())
//    {
//      int f = BTSerial.read();
//      Serial.print(f);
//    }         
}


void calibrate (int &value)
{
   char enter_key = '0';
   if (Serial.available()>0)
    {
      enter_key = Serial.read();
       if (enter_key == 'f')  //press f to enter calibrated value, for FB (roll)
      {
        value = FB_value;
        a++; // move on to next switch case
      }

      if (enter_key == 'l')  //press l to enter calibrated value, for LR (pitch)
      {
        value =  LR_value;
        a++; // move on to next switch case
      }
      
    }   
}


void send_to_serial (int instruction)
{
      BTSerial.write(instruction);
      //Serial.print(instruction);
}

