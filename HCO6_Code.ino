

//Connections
/*
HC06----> RXD->D3, TXD->D2, GND->GND, VCC->3.3V
LED-----> '+'->RESISTOR TO D8, '-'->GND
*/

//port 1411
// no line ending

#include <AFMotor.h>
AF_DCMotor FB_motor(1, MOTOR12_8KHZ);// set frequency of motor motor
AF_DCMotor LR_motor(2, MOTOR12_8KHZ);

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(2, 3); // RX | TX



char c = ' ';
int BT_instruction = 0;
int led_pin = 8;
String readString;


void setup() {

    Serial.begin(115200);
    Serial.println("Arduino with HC-06 is ready");
    BTSerial.begin(9600);  
    Serial.println("BTserial started at 9600");

    FB_motor.setSpeed(255); //controls power delivered to motor. Parameter is between 0 and 255 but tested usable range of speed for RC car motor is 50-255
    LR_motor.setSpeed(255); 

    pinMode(led_pin, OUTPUT);
}


void loop() 
{
  delay(500);

  while(BTSerial.available()>=1)
  {
    BT_instruction = BTSerial.read();
    BTSerial.write(BT_instruction);
      if (BT_instruction == 1) // FORWARD-RIGHT
      {
        FB_motor.run (FORWARD);
        LR_motor.run (FORWARD);
        delay(500);
      }
      if (BT_instruction == 2) // FORWARD-LEFT
      {
        FB_motor.run (FORWARD);
        LR_motor.run (FORWARD);
        delay(500);
      }
      if (BT_instruction == 3) // BACKWARD-RIGHT
      {
        FB_motor.run (BACKWARD);
        LR_motor.run (FORWARD);
        delay(500); 
        blink_led(); 
      }
      if (BT_instruction == 4) // BACKWARD-LEFT
      {
        FB_motor.run (BACKWARD);
        LR_motor.run (BACKWARD);
        delay(500); 
      }
      if (BT_instruction == 5) // FORWARD
      {
        FB_motor.run (FORWARD);
        delay(500);
      }
      if (BT_instruction == 6) // BACKWARD
      {
        FB_motor.run (BACKWARD);
        delay(500);
      }

      if (BT_instruction == 7) // RIGHT
      {
        LR_motor.run (FORWARD);
        delay(500);
      }
      if (BT_instruction == 8) // LEFT
      {
        LR_motor.run (BACKWARD);
        delay(500);
        blink_led(); 
      }
      if (BT_instruction == 9) // REST
      {
        FB_motor.run (RELEASE);
        LR_motor.run (RELEASE);
        delay(500);
      }
//Ends serial comm and then re-establishes on next line, this causes deletion of anything previously stored in the buffer or cache
//Without this 'flush' values recieved / read by Serial are sporratic because of
//wait time previously recieved data (concept of first in first out)
     BTSerial.end();
     BTSerial.begin(9600);
  }
   
}

void blink_led()
{
  digitalWrite(led_pin, HIGH);
  delay(1000);
  digitalWrite(led_pin, LOW);
  delay(200);
}





