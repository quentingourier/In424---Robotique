#include <Servo.h>
#define echoPin 29
#define trigPin 25
#define pwm_min 500
#define pwm_max 2400

Servo motorFront ;

byte pin_motor_left = 5;
byte pin_motor_right = 6;
Servo motorL,motorR ;

void setup() {
Serial.begin(9600);
pinMode(trigPin,OUTPUT);
pinMode(echoPin,INPUT);
//motorFront.attach (3, 530 , 2500) ; // for the sg90 servomotor
//motorFront.write(90);
motorL.attach(pin_motor_left,pwm_min,pwm_max);
motorR.attach(pin_motor_right,pwm_min,pwm_max);
}

void backward(){
  motorL.writeMicroseconds(1300) ;      //[recul<1500 ; avance>1500]
  motorR.writeMicroseconds(1700) ;      //[avance<1500 ; recul>1500]
}

void forward(){
  motorL.writeMicroseconds(1700) ;      //[recul<1500 ; avance>1500]
  motorR.writeMicroseconds(1380) ;      //[avance<1500 ; recul>1500] 
}

void stop_(){
  motorL.writeMicroseconds(1500) ;      //[recul<1500 ; avance>1500]
  motorR.writeMicroseconds(1500) ;      //[avance<1500 ; recul>1500]
}

void loop () {
  forward();
  delay(4000);
  backward();
  delay(10000);
  stop_();
}
