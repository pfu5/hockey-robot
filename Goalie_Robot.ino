#include <Pixy2I2C.h>
Pixy2I2C pixy;
#include <math.h>
#include <DRV8835MotorShield.h>
#include <Wire.h>
#include <SPI.h>
#include <NRF24.h>
#include <string.h>
NRF24 radio;

bool tx;
int divide_loc[2];
int k;
String Srobot1_x; // robot1_x goes through String to Float Changes in void loop
double robot1_x, robot1_y, robot2_x, robot2_y, puck_x, puck_y; // robot1_x goes through String to Float Changes in void loop


DRV8835MotorShield motors(7,5,8,6);

double speed;

void setup() {
  
  Serial.begin(115200);
  pixy.init(); 

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

void loop() {
  //double robot2_y = 500.0;
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

  pixy.ccc.getBlocks(); // Pixy gets blocks
  if ((robot1_y >= 620.0) && (robot1_y <= 580.0)){
    
    if (pixy.ccc.numBlocks){  
      Serial.println("Got Blocks");
      // Pixy mode here
      int32_t current_location = (int32_t)pixy.ccc.blocks[0].m_x;
      int32_t frameWidth = (int32_t)pixy.frameWidth;
  
      if (current_location > frameWidth/2){ 
          speed = 170;
          Serial.println("Going Forward");
          motors.setM1Speed(speed); // Right Wheel
          motors.setM2Speed(speed); // Left Wheel 
          delay(2);
      }        
      else if (current_location < frameWidth/2){ 
          speed = 170;
          Serial.println("Going Backward");
          motors.setM1Speed(-speed); // Right Wheel
          motors.setM2Speed(-speed); // Left Wheel 
          delay(2);
      }
    }
    else {
          // RF mode here
    }
  }
}
