#include "Arduino_NineAxesMotion.h" 
#include <Servo.h>
#include <Wire.h>
#define echoPin 29
#define trigPin 25
#define pwm_min 1300
#define pwm_max 1700
#define ncols 13
#define nrows 28


//----------------------------------------------------------------------------------------------
//              INITIALIZE DATA                                        
//----------------------------------------------------------------------------------------------


Servo motorFront ;
byte pin_motor_left = 5;
byte pin_motor_right = 6;
Servo motorL,motorR ;
NineAxesMotion mySensor ; // Object that for the sensor
unsigned long lastStreamTime = 0; // To store the last streamed time
const int streamPeriod = 20;
int distance;
int disp[nrows][ncols];
int orientation_init=0;


//----------------------------------------------------------------------------------------------
//              FUNCTIONS                                       
//----------------------------------------------------------------------------------------------


void setup(){
  
  Serial.begin(9600); //initialize the Serial communication
  pinMode(trigPin,OUTPUT); //set the pin 25 to behave as an output
  pinMode(echoPin,INPUT); //set the pin 29 to behave as an input
  
  motorFront.attach (3, 530 , 2500) ; // for the sg90 servomotor
  motorFront.write(90); //set the sensor to make the robot look forward
  
  motorL.attach(pin_motor_left,pwm_min,pwm_max); //set the left servoMotor to a pin
  motorR.attach(pin_motor_right,pwm_min,pwm_max); //set the left servoMotor to a pin
  stop_(); 
  
  Wire1.begin(); 
  mySensor.initSensor(); // Sensor Initialization
  mySensor.setOperationMode( OPERATION_MODE_NDOF ); 
  mySensor.setUpdateMode( MANUAL );


  for(int i=0 ; i<nrows ; i++){         //
    disp[i][0] = 2;                      //
    disp[i][ncols-1] = 2;                 //
  }                                        // create a matrix with marked-perimeter 
  for(int i=0 ; i<ncols ; i++){           //
    disp[0][i] = 2;                      //
    disp[nrows-1][i] = 2;               //
  }
}


int getRobotYaw (){ //function from the pdf document returning the angle orientation value of the robot
   if (( micros () - lastStreamTime ) >= streamPeriod ){
     lastStreamTime = micros(); //can be replaced by millis()
     mySensor.updateEuler(); 
     int orientation = mySensor.readEulerHeading();
     Serial.print(" H: ");
     Serial.print( mySensor.readEulerHeading()); // Heading data
     Serial.println("deg ");
     return orientation;
   }
}

void backward(){
  motorL.writeMicroseconds(1300) ;      //[backward<1500 ; forward>1500] 
  motorR.writeMicroseconds(1700) ;      //[forward<1500 ; backward>1500]
}

void forward(){
  motorL.writeMicroseconds(1700) ;      //[backward<1500 ; forward>1500]
  motorR.writeMicroseconds(1385) ;      //[forward<1500 ; backward>1500] 
}

void forward2(){
  motorL.writeMicroseconds(1800) ;      //[backward<1500 ; forward>1500]
  motorR.writeMicroseconds(1300) ;      //[forward<1500 ; backward>1500] 
}

void stop_(){
  motorL.writeMicroseconds(1498) ;      //[backward<1500 ; forward>1500] adjustment on motor left of 2us
  motorR.writeMicroseconds(1500) ;      //[forward<1500 ; backward>1500]
}

int dist(){//function returning the distance from the robot sensor to the closer obstacle
  digitalWrite(trigPin, LOW); //change the pin state to low
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); //change the pin state to high
  delayMicroseconds(2);
  digitalWrite(trigPin, LOW); //change the pin state to low
  unsigned long duration = pulseIn(echoPin, HIGH); //use pulseIn to return the time the wave has taken
  int value = duration*0.034/2; //converting the previous time into a distance
  return value;
}

void balayage(){ //optional function scanning around (0/180) and detecting obstacles using dist() function
  for(int i=0; i<=180; i++){ //from right to the left
    motorFront.write(i); //setting up the sensor to the angle i (incrementing)
    delay(10);
    if(i==180){ 
      Serial.println("obstacle GAUCHE à "); //stops at the end of the "neck rotation" and use dist()
      dist();
      for(i=180; i>=0; i--){ //from left to the right
        motorFront.write(i); //setting up the sensor to the angle i (decrementing)
        delay(10);
        if(i==0){
          Serial.println("obstacle DROITE à "); //stops at the end of the "neck rotation" and use dist()
          dist();
        }
      }
    }
  }
}

void turn_right(){ //function making the robot turn right using the accelerometer (angle orientation value)
  // mySensor.initSensor();
  mySensor.updateEuler(); 
  int orientation = mySensor.readEulerHeading();
  
  while(mySensor.readEulerHeading()-orientation_init<0 || mySensor.readEulerHeading()-orientation_init < 90){
    mySensor.updateEuler();
    // Serial.println(mySensor.readEulerHeading());
    motorR.writeMicroseconds(1700); //going backward 
    motorL.writeMicroseconds(1700); //going forward    //that way the rotation is optimized and quicker
  }
  stop_(); //stops the both motors once the rotation has been done
}


void turn_left(){ //same as above but turning left
  // mySensor.initSensor();
  mySensor.updateEuler(); 
  int orientation = mySensor.readEulerHeading();
    
  while(mySensor.readEulerHeading()-orientation_init>275 || mySensor.readEulerHeading()-orientation_init < 5){
    mySensor.updateEuler();
    //Serial.println(mySensor.readEulerHeading()-orientation_init);
    motorL.writeMicroseconds(1300);
    motorR.writeMicroseconds(1370);
  }
  stop_();
}

void turn_left2(){ //turning left using only motor speed (quickest and safest way to turn, avoiding angle errors)
  // mySensor.initSensor();  

    motorL.writeMicroseconds(1300);
    motorR.writeMicroseconds(1370);
    delay(810); //adjustment of the delay depending on the motor settings
    stop_(); //stops the motors at the end of the rotation
}

void turn_right2(){ //turning right using only motor speed
    motorR.writeMicroseconds(1700);
    motorL.writeMicroseconds(1700);
    delay(720); //adjustment of the delay depending on the motor settings
    stop_(); //stops the motors at the end of the rotation
}

void scan_zone(){ //same as balayage() function but using Processing to plot a radar
  for(int i=0;i<=180;i++){  
    motorFront.write(i);
    delay(10);
    distance = dist();// Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree
    Serial.print(i); // Sends the current degree into the Serial Port
    Serial.print(","); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
    Serial.print(distance); // Sends the distance value into the Serial Port
    Serial.print("."); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
  }
  for(int i=180;i>0;i--){  
    motorFront.write(i);
    delay(10);
    distance = dist();
    Serial.print(i);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(".");
  }
}


void look_mid(){ //makes the robot look forward          //
  Serial.print("mid | ");
  motorFront.write(90);                                  
  delay(750);                                            //
}
                                                        
void look_left(){ //makes the robot look left            //These functions have been useful during testing phase
  Serial.print("left | ");
  motorFront.write(180);                                  
  delay(750);                                            //
}

void look_right(){ //makes the robot look right          //
  Serial.print("right | ");
  motorFront.write(0);
  delay(750);
}

void avancer_une_case(){ //function that makes the robot going forward on 16 centimeter (one cell of the matrix)
  forward();
  delay(900); //time calculated during tests to determine what's the speed of the robot
  stop_();
  }

void red_burn(){ //during tests on the table, we couldn't analyze the matrix coordinates as it was a Serial output
  backward();    //and the robot was runned on battery, we then had to create a specific function as a flag
  delay(200);    //so finally we decided to create this function that allows the robot the do a short wheeling
  forward2();    //the name is just a reference to a Formula 1 team
  delay(300);
}

void display_mat(int disp[nrows][ncols]){ //display the matrix i.e. the table map
  for(int i=0 ; i<nrows ; i++){
    for(int j=0 ; j<ncols ; j++){
      Serial.print(disp[i][j]);
    }
    Serial.print("\n");
  }
}

void build(int disp[nrows][ncols], int startx, int starty, int finishx, int finishy){ 
  disp[starty][startx] = 3; //placing the start position in the matrix, so on the table
  disp[finishy][finishx] = 3; //placing the end position in the matrix, so on the table
}

void update_mat(int disp[nrows][ncols], int x, int y, int value){ //update the map according to obstacles, moves...
  disp[y][x] = value;
}

bool isGoal(int x, int y, int goalx, int goaly){ //function returning if yes or no the robot reached its goal
  if(x==goalx && y == goaly){
    return true;
  }
  else 
    return false;
}



//----------------------------------------------------------------------------------------------
//              MAIN PROGRAM                                      
//----------------------------------------------------------------------------------------------


void loop () {

  //-----------------------
  // Values and Displays
  //-----------------------
  int dleft, dright;
  int x=ncols-2, y=1;
  int mv = 0, old_mv = 0;
  
  Serial.println("-----------------------");
  display_mat(disp); //displaying unexplored matrix
  Serial.println("-----------------------");
  build(disp, ncols-2, 1, ncols-2, nrows-2); //placing start/end positions
  Serial.println("-----------------------");
  display_mat(disp); //check-displaying unexplored matrix with start/end positions
  Serial.println("-----------------------");

  
  //-----------------------
  // Part 1 : Exploration
  //-----------------------
  while(!isGoal(x, y, ncols-2, nrows-2) && y<24){ //keeps going while robot has not reached a certain area
    if(dist()>16){ 
      avancer_une_case(); //going forward if there is no obstacle
      // mv = 0;
      // y++;
    }
    else{ //starting the obstacle process
      if(old_mv + mv == 0){ //obstacle in front of the robot,
        update_mat(disp, x, y+1, 2); //place the obstacle on the map 
      }
      else{
        if(old_mv + mv == 1){ //obstacle to the right, 
          update_mat(disp, x-1, y, 2); //place it on the map
        }
        else{
          if(old_mv + mv == -1){ //obstacle to the left, 
            update_mat(disp, x+1, y, 2); //place it on the map
          }
          else{
            update_mat(disp, x, y-1, 2); //only solution remaining is an obstacle behind, place it on the map
          }
        }
      }

      //-----------------------
      // Pathfinding process
      //-----------------------
      stop_(); 
      look_left(); //check to the left
      dleft = dist(); //is there any close obstacle
      look_right(); //check to the right
      dright = dist(); //is there any close obstacle
      look_mid(); //set the sensor back to its original position i.e. looking forward
      Serial.println("Distance a gauche");
      Serial.println(dleft);
      Serial.println("Distance a droite");
      Serial.println(dright);
      
      if(dleft<dright){ //choosing where to go depending on the obstacles detected previously
        turn_right2();
        old_mv = mv; //save the last move as "old move" in memory
        mv = 1; //define the new move, which is turning right
       // x--;
      }
      else{
        turn_left2();
         old_mv = mv; //save the last move as "old move" in memory
         mv = -1; //define the new move, which is turning left
        // x++;
      }  
    } //end of obstacle process

    
    //-----------------------
    // Updating coordinates
    //-----------------------
    if(mv+old_mv == 0){ //analyzing what were the previous moves, current position is found
      y++; //update robot position within rows in the matrix (going south of the map)
      Serial.println("Je vais tout droit"); //real-time information on where the robot goes
    }
    else{
        if(mv+old_mv == 1){//analyzing what were the previous moves, current position is found
          x--; //update robot position within columns in the matrix (going west of the map)
          Serial.println("Je vais a droite"); //real-time information on where the robot goes
        }
        else{
          if(mv + old_mv == -1){//analyzing what were the previous moves, current position is found
            x++; //update robot position within rows in the matrix (going east of the map)
            Serial.println("Je vais a gauche"); //real-time information on where the robot goes
          }
          else{
            y--; //update robot position within rows in the matrix (going north of the map)
            Serial.println("Je vais en arrière"); //real-time information on where the robot goes
          }
        }
    }
    update_mat(disp, x, y, 1); //updating the current position in the matrix using 1 as robot flag position 
    Serial.println("-----------------------");
    display_mat(disp); //displaying the new matrix
    Serial.println("-----------------------");  
    delay(100);
  } //end of the first part exploration, the robot has reached the area

  delay(1000); //wait for 1s
  

  //-----------------------
  // Part 2 : Finishing
  //-----------------------
  turn_left2(); //heading towards the end position as quick as possible
  while(x != ncols-2){ //keeps going forward while the robot is not aligned with the end position
    avancer_une_case();
    x++;
    update_mat(disp, x, y, 1);  
    Serial.println("-----------------------");   //same updating/displaying process as previously
    display_mat(disp);
    Serial.println("-----------------------"); 
  }
  
  delay(1000); //wait for 1s
  
  while(!isGoal(x, y, ncols-2, nrows-2) && y>= 22){ //condition so the robot remains on the finishing area
    turn_right2(); //get the robot aligned with the finish line
    while(y != nrows-2){ //condition so the robot stops when arriving to the end
      avancer_une_case();
      Serial.println(y);
      y++;
      update_mat(disp, x, y, 1);  
      Serial.println("-----------------------");   //same updating/displaying process as previously
      display_mat(disp);
      Serial.println("-----------------------"); 
    }
  }
     
  Serial.println("arrivée"); //robot has arrived 
  delay(1000); //waits for 1s
  red_burn(); //and we get informed with the red burn function
  delay(2000);
}







  



 
