#include <Pixy2I2C.h>
Pixy2I2C pixy;
#include <math.h>
#include <DRV8835MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <NRF24.h>
#include <string.h>
NRF24 radio;

bool tx;
int divide_loc[2];
int k;
String Srobot1_x; // robot1_x goes through String to Float Changes in void loop
double robot1_x, robot1_y, robot2_x, robot2_y, puck_x, puck_y; // robot1_x goes through String to Float Changes in void loop
int solenoidPin = 4;         

DRV8835MotorShield motors(7,5,8,6);

// Switch States
#define check 0
#define forward 1
#define right 2
#define left 3
#define toGoal 4
float calWidth = 55; // Calibrated width reading
float actualWidth = 1.5; // Width of object in inches
float actualDist = 10; // Actual distance in inches
float F; // Focal Length
float Dist; // Distance
uint8_t mystate; //Create global state variable
int val = 0;
int viewRange = 50;
int speed = 0;
double orientation;
double desired_ori;
bool dir;
double turnToGoal;
int pingSignal = 3; // Ping Sensor pin
double ping_distance;
unsigned long pulseduration=0;

sensors_event_t event;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  
  Serial.begin(115200);
  pinMode(solenoidPin, OUTPUT);
  pinMode(pingSignal, OUTPUT);
  /* Initialize the Pixy */
  pixy.init(); 

  mystate = check;
  
//  F = (calWidth*actualDist)/actualWidth; // Calculate focal length

  /* Initialise the IMU */
  if (!bno.begin())  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  
  /* Initialize the RF */
  radio.begin(9, 10);

  tx = 0;// !digitalRead(7);

  if (tx)
  {
    radio.setAddress(0xAA); // Green Team AA 
                            // Blue Team F2
  }
  else
  {
    radio.listenToAddress(0xAA); // Green Team AA 
                                 // Blue Team F2
  }

}

void frontDist()
{
 // set pin as output so we can send a pulse
 pinMode(pingSignal, OUTPUT);
// set output to LOW
 digitalWrite(pingSignal, LOW);
 delayMicroseconds(5);
 
 // now send the 5uS pulse out to activate Ping)))
 digitalWrite(pingSignal, HIGH);
 delayMicroseconds(5);
 digitalWrite(pingSignal, LOW);
 
 // now we need to change the digital pin
 // to input to read the incoming pulse
 pinMode(pingSignal, INPUT);
 
 // finally, measure the length of the incoming pulse
 pulseduration=pulseIn(pingSignal, HIGH);
}

void loop() {
   frontDist();
   ping_distance = pulseduration*0.0001*343/2;
//   Serial.print("Ping Distance HERE: ");
//   Serial.println(ping_distance);
   
   pixy.ccc.getBlocks(); // Pixy gets blocks
   bno.getEvent(&event); // IMU gets Euler
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   Serial.print(" Orientation: ");
   Serial.println(euler.x());

//    //***Testing Values:
//    double puck_x = 1000.0;
//    double puck_y = 700.0;
//    double robot1_x = 1800.0;
//    double robot1_y = 800.0;
    
    if (radio.available()){
      
      char buf[32];
      // TEST char buf[] = "1234,5678,9123,4567,8912,3456";
      
      uint8_t numBytes = radio.read(buf, sizeof(buf));
      char *strings[6];                                 // an array of pointers to the pieces of the above array after strtok()
      char *ptr = NULL;
      
      Serial.print(F("Received "));
      Serial.print(numBytes);
      Serial.print(F(" bytes: "));
      Serial.println(buf);
      
      byte index = 0;
      ptr = strtok(buf, ":,");  // delimiter IMPORTANT
      while (ptr != NULL)
      {
        strings[index] = ptr;
        index++;
        ptr = strtok(NULL, ":,"); // delimiter IMPORTANT
      }
      
      //Serial.println(index);
      // print all the parts
      
      //   Serial.println("The Pieces separated by strtok()");
      //   for (int n = 0; n < index; n++)
      //   {
      //      Serial.print(n);
      //      Serial.print("  ");
      //      Serial.println(strings[n]);
      //   }
      
  
      Srobot1_x = strings[0];
      Srobot1_x = Srobot1_x.substring(1);
      robot1_x = Srobot1_x.toFloat();
       
      robot1_y = atof(strings[1]);
      robot2_x = atof(strings[2]);
      robot2_y = atof(strings[3]); 
      puck_x = atof(strings[4]);
      puck_y = atof(strings[5]);
      
      // Print values to Serial Monitor
      Serial.print("Robot1_x = ");
      Serial.println(robot1_x);
      Serial.print("Robot1_y = ");
      Serial.println(robot1_y);
      
      Serial.print("Robot2_x = ");
      Serial.println(robot2_x);
      Serial.print("Robot2_y = ");
      Serial.println(robot2_y);
      
      Serial.print("Puck_x = ");
      Serial.println(puck_x);
      Serial.print("Puck_y = ");
      Serial.println(puck_y);
    }
      Dist = sqrt(pow((puck_x-robot1_x),2) + pow((puck_y-robot1_y),2)); // Dist from RF data
      Serial.print("Distance = ");
      Serial.println(Dist);
  if ( ping_distance <= 8.5 ){
    // Step back if too close to the edge of the wall
    Serial.print("GO BACK");
    motors.setM1Speed(-110); // Right Wheel
    motors.setM2Speed(-90); // Left Wheel 
    delay(2);
  }
  else if (pixy.ccc.numBlocks){  
//    Serial.println("Got Block!");
    int32_t current_location = (int32_t)pixy.ccc.blocks[0].m_x;
    int32_t frameWidth = (int32_t)pixy.frameWidth;
//    Dist = (actualWidth*F)/pixy.ccc.blocks[0].m_width; // Dist from Pixy
    Dist = sqrt(pow((puck_x-robot1_x),2) + pow((puck_y-robot1_y),2)); // Dist from RF data
    Serial.print("Distance  -  ");
    Serial.println(Dist);

   
    //***state machine***
    switch (mystate) {
      case check:

        if ((Dist > 230.0) && 
            (current_location <= frameWidth/2 + viewRange) &&
            (current_location >= frameWidth/2 - viewRange)){
        // Puck is around the center range and too far away from the the robot
          mystate = forward;
        }
        // check if x position is to the right of the center point of the camera
        else if (Dist <= 230.0){ 
          mystate = toGoal;
        }
        else if (current_location > frameWidth/2 + viewRange){ 
          mystate = left;
        }        
        else if (current_location < frameWidth/2 - viewRange){ 
          mystate = right;

        }
        else {
          Serial.println("CHECK - DO NOTHING");
        }
        break;
        
      case forward:
        speed = 180;
        Serial.println("Going Forward");
        motors.setM1Speed(speed); // Right Wheel
        motors.setM2Speed(speed); // Left Wheel 
        delay(2);
        mystate = check;
        break;
      
      case right:
        speed = 120;
        speed = map(abs(current_location-frameWidth/2), 0, 158, 100, speed);
        Serial.print("Turning Right at speed: ");
        Serial.println(speed);
        // Turning command here:
        motors.setM1Speed(-speed); // Right Wheel
        motors.setM2Speed(speed); // left Wheel 
        delay(100);
        mystate = check;
        break;
                  
      case left:
        speed = 120;
        speed = map(abs(current_location-frameWidth/2) , 0, 158, 100, speed);
        Serial.print("Turning Left at speed: ");
        Serial.println(speed);
        // Turning command here:
        motors.setM1Speed(speed); // Right Wheel
        motors.setM2Speed(-speed); // Left Wheel 
        delay(100);
        mystate = check;
        break;
        
      case toGoal:
        Serial.println("******************ToGoal Mode");
        orientation = euler.x();
        /* // Blue Team
        if (robot1_y > 625.0){
          Serial.print("Turn degree: ");
          Serial.println(360.0 - atan2(abs(robot1_y - 620.0),abs(2229.0 - robot1_x))* 180.0/3.14159265);
          if ((orientation >= (360.0 - atan2(abs(robot1_y - 620.0),abs(2229.0 - robot1_x))* 180.0/3.14159265) - 15.0) &&
          (orientation <= (360.0 - atan2(abs(robot1_y - 620.0),abs(2229.0 - robot1_x))* 180.0/3.14159265) + 15.0)) {
            speed = 150;
            // go forward if it finishes turning
            Serial.println("go forward if it finishes turning!!!!!!!!!!!!!");
            delay(200);
            motors.setM1Speed(speed); // Right Wheel
            motors.setM2Speed(speed); // Left Wheel 
            delay(2000);
          }
          else {
            speed = 100;
            // turn right
            Serial.println("turn right with puck");
            motors.setM1Speed(-speed); // Right Wheel
            motors.setM2Speed(speed); // Left Wheel 
          }
        }
        else {
          Serial.print("Turn degree: ");
          Serial.println(atan2(abs(robot1_y - 620.0),abs(2229.0 - robot1_x))* 180.0/3.14159265);
          if ((orientation >= (atan2(abs(robot1_y - 620.0),abs(2229.0 - robot1_x))* 180.0/3.14159265) - 15.0) &&
          (orientation <= (atan2(abs(robot1_y - 620.0),abs(2229.0 - robot1_x))* 180.0/3.14159265) + 15.0)) {
            speed = 160;
            // go forward if it finishes turning
            Serial.println("go forward if it finishes turning!!!!!!!!!!");
            delay(200);
            motors.setM1Speed(speed); // Right Wheel
            motors.setM2Speed(speed); // Left Wheel 
            delay(2000);
          }
          else {
            speed = 100;
            // turn left
            Serial.println("turn left with puck");
            motors.setM1Speed(speed); // Right Wheel
            motors.setM2Speed(-speed); // Left Wheel 
          }
        }
        */
        // Green Team
        if (robot1_y > 625.0){
          Serial.print("Turn degree: ");
          Serial.println(atan2(abs(robot1_y - 620.0),abs(robot1_x - 233.0))* 180.0/3.14159265);
          if ((orientation >= (atan2(abs(robot1_y - 620.0),abs(robot1_x - 233.0))* 180.0/3.14159265) - 15.0) &&
          (orientation <= (atan2(abs(robot1_y - 620.0),abs(robot1_x - 233.0))* 180.0/3.14159265) + 15.0)) {
            speed = 150;
            // go forward if it finishes turning
            Serial.println("go forward if it finishes turning!!!!!!!!!!!!!");
            delay(200);
            motors.setM1Speed(speed); // Right Wheel
            motors.setM2Speed(speed); // Left Wheel 
            delay(2000);
            digitalWrite(solenoidPin, HIGH);      //Switch Solenoid ON
            delay(1000);                          //Wait 1 Second
            Serial.println("Solenoid ON");
            digitalWrite(solenoidPin, LOW);       //Switch Solenoid OFF
            delay(500);                          //Wait 1 Second
            Serial.println( "Solenoid Off");            
          }
          else {
            speed = 100;
            // turn right
            Serial.println("turn right with puck");
            motors.setM1Speed(-speed); // Right Wheel
            motors.setM2Speed(speed); // Left Wheel 
          }
        }
        else {
          Serial.print("Turn degree: ");
          Serial.println(360.0 - atan2(abs(620.0 - robot1_y),abs(robot1_x - 233.0))* 180.0/3.14159265);
          if ((orientation >= (360.0 - atan2(abs(620.0 - robot1_y),abs(robot1_x - 233.0))* 180.0/3.14159265) - 15.0) &&
          (orientation <= (360.0 - atan2(abs(620.0 - robot1_y),abs(robot1_x - 233.0))* 180.0/3.14159265) + 15.0)) {
            speed = 160;
            // go forward if it finishes turning
            Serial.println("go forward if it finishes turning!!!!!!!!!!");
            delay(200);
            motors.setM1Speed(speed); // Right Wheel
            motors.setM2Speed(speed); // Left Wheel 
            delay(2000);
            digitalWrite(solenoidPin, HIGH);      //Switch Solenoid ON
            delay(1000);                          //Wait 1 Second
            Serial.println("Solenoid ON");
            digitalWrite(solenoidPin, LOW);       //Switch Solenoid OFF
            delay(500);                          //Wait 1 Second
            Serial.println( "Solenoid Off");
          }
          else {
            speed = 100;
            // turn left
            Serial.println("turn left with puck");
            motors.setM1Speed(speed); // Right Wheel
            motors.setM2Speed(-speed); // Left Wheel 
          }
        }
        mystate = check;
        break;
        
    }
    
  }

  else {
    // When Pixy doesn't see the puck, 
    // Use RF communication to figure out where the puck is  
    Serial.println("Looking for puck.");
    orientation = euler.x();
//    Serial.print(" Orientation: ");
//    Serial.println(orientation);


//// *** Separate the puck location wrt to the robot1 location via four quadrants ***

//    if ( ((puck_x - robot1_x) < 0) && ((puck_y - robot1_y) > 0) ){
//      desired_ori = 180.0 - atan2(abs(puck_y - robot1_y),abs(puck_x - robot1_x))* 180.0/3.14159265; 
//      Serial.print("Desired Orientation: ");
//      Serial.println(desired_ori);
//    }
//    else if ( ((puck_x - robot1_x) > 0) && ((puck_y - robot1_y) > 0) ){
//      desired_ori = atan2(abs(puck_y - robot1_y),abs(puck_x - robot1_x))* 180.0/3.14159265;
//      Serial.print("Desired Orientation: ");
//      Serial.println(desired_ori);
//    }
//    else if ( ((puck_x - robot1_x) > 0) && ((puck_y - robot1_y) < 0) ){
//      desired_ori = 360.0 - atan2(abs(puck_y - robot1_y),abs(puck_x - robot1_x))* 180.0/3.14159265;
//      Serial.print("Desired Orientation: ");
//      Serial.println(desired_ori);
//    }
//    else {
//      desired_ori = 180.0 + atan2(abs(puck_y - robot1_y),abs(puck_x - robot1_x))* 180.0/3.14159265;
//      Serial.print("Desired Orientation: ");
//      Serial.println(desired_ori);
//    }
//
//    // ***Turn and go toward the puck***
//    if ((orientation >= (desired_ori - 15.0)) && (orientation <= (desired_ori + 15.0))){
//      Serial.println("***Go toward puck.***");
//      speed = 150;
//      delay(300);
//      motors.setM1Speed(speed);
//      motors.setM2Speed(speed);
//    }
//    else {
//      Serial.println("***Turning to face puck.***");
//        speed = 80;
//        motors.setM1Speed(speed); // Right Wheel
//        motors.setM2Speed(-speed); // Left Wheel         
//      }
    }
  

}
