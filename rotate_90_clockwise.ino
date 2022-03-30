#include "Arduino_NineAxesMotion.h" 
#include <Servo.h>
#include <Wire.h>
#define echoPin 29
#define trigPin 25
#define pwm_min 1300
#define pwm_max 1700


Servo motorFront ;
byte pin_motor_left = 5;
byte pin_motor_right = 6;
Servo motorL,motorR ;
NineAxesMotion mySensor ; // Object that for the sensor
unsigned long lastStreamTime = 0; // To store the last streamed time
const int streamPeriod = 20;


void setup() {
Serial.begin(9600);
pinMode(trigPin,OUTPUT);
pinMode(echoPin,INPUT);

// motorFront.attach (3, 530 , 2500) ; // for the sg90 servomotor
// motorFront.write(90); //look in front of him

motorL.attach(pin_motor_left,pwm_min,pwm_max);
motorR.attach(pin_motor_right,pwm_min,pwm_max);
stop_();

Wire1.begin(); 
mySensor.initSensor(); // Sensor Initialization
mySensor.setOperationMode( OPERATION_MODE_NDOF ); 
mySensor.setUpdateMode( MANUAL ); 
}

int getRobotYaw (){
   if (( micros() - lastStreamTime ) >= streamPeriod ){
   lastStreamTime = micros(); //mettre micros() pour voir si ça réduti le temps et avoir un balayage resserré
   mySensor.updateEuler(); 
   int orientation = mySensor.readEulerHeading();
   Serial.print(" H: ");
   Serial.print( mySensor.readEulerHeading()); // Heading data
   Serial.println("deg ");
   return orientation;
   }
}

void stop(){
  motorL.writeMicroseconds(1500);   
  motorR.writeMicroseconds(1500);      
}

int orientation_init = getRobotYaw();

void turn_right(){
  int orientation = getRobotYaw();
  if(orientation == orientation_init + 90){
    stop_();
    delay(5000);
    Serial.println("90 degrees clockwise rotation done!\n");
  }
  motorL.writeMicroseconds(1700); //makes the left wheel going forward
  motorR.writeMicroseconds(1700); //makes the right wheel going backward
}



void loop (){
  turn_right();
}
