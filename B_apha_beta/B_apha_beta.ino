/****************************************************************************************************************
                  |-------------------------------------------------------------------------|                                            
                  |  AUTONOMOUS ROBOT SWARM FOR GOAL SEARCHING USING ALPHA BETA ALGORITHM   |
                  |-------------------------------------------------------------------------|
                       
PURPOSE:  This code is for our B.Tech project entitled "Development of Autonomous robot swarm 
          for goal searching using ALpha Beta coodination" 
          - Anand Krishnamoorthy, Girish Kumar, Atul Balachandran   
          For More Information : https://sites.google.com/site/swarmrobotproject    
                    
                           |-----------------------------------------------------------|
                           |Copyright (c) 2015 Anand Krishnamoorthy, Girsh Kumar       |
                           |Contact Details : anandkrish894@gmail.com                  |
                           |-----------------------------------------------------------|
IMPOTANT NOTICE: 
  
      This program was modified extensively from the MPU drivers from InvenSense Corporation 
      (Copyright (C) 2011-2012 All Rights Reserved.) and the RF24 Network 
      (Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>)
      This program has been written and redistributed in conformation with the GPL under which
      the original code was licensed. 
             This Code modified specifically for the Gesture controlled audio player project.
             
LICENSE AGREEMENT-

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3.0 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

For further information regarding the license agreement : https://opensource.org/licenses/GPL-3.0

**************************************************************************************************************/

/* Header Files */
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Servo.h>
#include "MyMPU9150.h"
#include <RF24Network.h>

/* Pin Definitions*/
#define CE 10
#define CSN 9
#define LMT1 5
#define LMT2 4
#define RMT1 6
#define RMT2 7
#define FtrigPin 14
#define FechoPin 15
#define BtrigPin 16
#define BechoPin 17
#define servoPin 3
#define LED 2
#define rf 8


/* Class Objects*/
Servo myservo;
MyMPU9150 mpu;
RF24 radio(CE, CSN); //ce,csn
RF24Network network(radio);

/* Node addresses */
const uint8_t node1 = 000; //hb
const uint8_t node2 = 001; //a
const uint8_t node3 = 011; //b
const uint8_t node4 = 021; //c

RF24NetworkHeader header1(node1); //to hb
RF24NetworkHeader header3(node3); //to b
RF24NetworkHeader header4(node4); //to c

/* Payload Structure */
struct payload_t
{
  char id;
  float heading;
  int target;
  float distance;
  bool state;

} rec, trans, A_status, C_status;

/* Global variables */
float heading = 0.0;
float dist = 0.0, Dist=0.0;
float d =0.0;
float time;
uint32_t t;
byte st = 0;
float deg = 0.0;
byte threshold;
uint32_t interval, interval2;
int flag, flag2 =0;
int trgt=0;
double dT;
int rf_deg;
bool State;

void setup()
{
  SPI.begin();
  radio.begin();
  Wire.begin();
  network.begin(30, node2); //channel,this_node_addr
  myservo.attach(3, 650, 2850); // 650, 2850

  pinMode(FtrigPin, OUTPUT);
  pinMode(FechoPin, INPUT);
  pinMode(BtrigPin, OUTPUT);
  pinMode(BechoPin, INPUT);
  pinMode(LMT1, OUTPUT);
  pinMode(LMT2, OUTPUT);
  pinMode(RMT1, OUTPUT);
  pinMode(RMT2, OUTPUT);
  pinMode(LED, OUTPUT);
   t = millis();

}

void loop()
{
    /* Alpha beta algorithm*/

  // START

  /* step 1: establish connection - ping test :
      each bot shud send and receive their ID to all others including HB */

  /* Step 2: set alpha mode for all the bots:
     (i)  Turn on the red LED light
     (ii) disperse in different directions- at 20, 140, 260 deg for each bot depending on how close the direction is to each bot
     (iii) move in the same direction and navigate through obstacles while communicating the sensor measures to other bots and HB */

  /* Step 3: check the time elapsed
     if it is less than threshold time then the step 2 actions are continued
     else execute step 4 */

  /* Step 4: Switching mode phase 1: Bsed on the sensor measure execute beta mode (the lowest measure bot becomes the beta bot)
    (i) switch off the red LED and ON the blue LED
    (ii) stop navigation and keep receiving the data from other bots
    (iii) follow the highest sensor measure bot SLOWLY (i.e move every small intervals of time)
    (iv) still keep getting ur sensor measure... */

  /* Step 5: Check the time elapsed
     if it is less than the second time threshold, then continue step 4 irrespective of the
     sensor measure sof the other two bots else step 6 */

  /* Step 6: Switching mode phase 2: if sensor measure of the other two alpha bots is
     less than or equal to the current measure of the beta bot then switch that bot to the beta mode as shown in Step 4
     (by the end there will be only one alpha bot) the bots must continue their nagivation towards the target while the other
      alpha bot wanders until the third threshold after which it switches to beta mode*/

  /* Step 7: the beta bots must nagivate till it reaches the target whose position will be confirmed by the HB and
     all the bots shall converge at the target */

  //STOP

  /* procedure(s) to be implemented in the main program
    1) obtacle avoidance - (SCAN (rotate the ultra) )the ultrasonic sensor must store its current heading and if it meets an obstacle, shift its heading away to
       the angular position with greatest distance value from the sensor provided it is different from the current heading +180 degrees.
       Give a threshold distance value (say 10 cm) if all the distances are less than the threshold, then reverse the direction and scan again
       (here too the current heading must be different than the chosen heading)

  */
  
  /* Alpha beta algorithm*/
  if ( st == 1)
    goto BETA;

  dT = t - millis();
  if (dT <= interval)
  {
  
ALPHA:
    set_alpha();
    State = 0;
    navigate();
    communicate();
    goto END;
  }
  else
  {
    communicate();
    if ((trgt < min(A_status.target, C_status.target)) || flag2 ==1 )
    {
BETA:
      beta();
      goto END;
    }
    else
    {  
      interval = interval2;
      flag2 = 1;
    }
       
  }
END:
trgt = analogRead(A6);
delay(50);
}

//Subroutines required


//1) Navigation -get heading (filtered if space is available at the end, use kalman too)
//              - get distance
void beta()
{
  st = 1;
  State = 1;
  set_beta();
  int turn_angle;
  if ( A_status.target > C_status.target)
  {
    turn_angle =  A_status.heading;

  }
  else if ( A_status.target < C_status.target)
  {
    turn_angle =  C_status.heading;

  }
  align(turn_angle);
  navigate();
  if (analogRead(A7) > threshold)
    STOP();
}

float Get_FiltHeading()
{
  //-------------------------------------------------------------------------------
  //Get SGA FIltered Heading
  mpu.wake_accel();
  delay(10);
  mpu.get_Heading(&heading);
  heading = mpu.get_SGAfilt(heading);
  mpu.set_accel_sleep();
  delay(5);
  return heading;
  //-------------------------------------------------------------------------------
}

void scan_area(int deg1)
{
  dist = get_ultra_valF();
  float prev_dist = dist;
  for (deg = 0; deg <= 180; deg += 12)
  {
    myservo.write(deg);
    dist = get_ultra_valF();
    prev_dist = max(dist, prev_dist);
    delay(50);

    if (prev_dist == dist)
    {

      deg1 = deg;
    }
  }
  myservo.write(90);
  delay(50);

}

int get_ultra_valF()
{
  digitalWrite(FtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(FtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(FtrigPin, LOW);
  time = pulseIn(FechoPin, HIGH);
  dist = (time / 2) / 29.1;
  dist = constrain(dist, 0, 200);
  return dist;
}

int get_ultra_valB()
{
  digitalWrite(BtrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(BtrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(BtrigPin, LOW);
  time = pulseIn(BechoPin, HIGH);
  dist = (time / 2) / 29.1;
  dist = constrain(dist, 0, 200);
  return dist;
}
/*2) robot movements - right
                   - left
                   - forward
                   - reverse
                   - stop */

void FWD()
{
  // Forward
  digitalWrite(LMT1, HIGH);
  digitalWrite(LMT2, LOW);
  digitalWrite(RMT1, HIGH);
  digitalWrite(RMT2, LOW);
}

void REV()
{
  // Reverse
  digitalWrite(LMT1, LOW);
  digitalWrite(LMT2, HIGH);
  digitalWrite(RMT1, LOW);
  digitalWrite(RMT2, HIGH);
}

void LEFT()
{
  // Left
  digitalWrite(LMT1, LOW);
  digitalWrite(LMT2, HIGH);
  digitalWrite(RMT1, HIGH);
  digitalWrite(RMT2, LOW);
}

void RIGHT()
{
  // Right
  digitalWrite(LMT1, HIGH);
  digitalWrite(LMT2, LOW);
  digitalWrite(RMT1, LOW);
  digitalWrite(RMT2, HIGH);
}

void STOP()
{
  // Stop
  digitalWrite(LMT1, LOW);
  digitalWrite(LMT2, LOW);
  digitalWrite(RMT1, LOW);
  digitalWrite(RMT2, LOW);
}

//3)communication - communicate with HB, other two bots (transmit and receive)
void send_data(char ID, float head, int trgt, float dist, bool st)
{
  network.update();
  trans = {ID, head, trgt, dist, st};
  network.write(header3, &trans, sizeof(trans));
  network.write(header4, &trans, sizeof(trans));
  network.write(header1, &trans, sizeof(trans));
}

void get_data()
{
  network.update();
  while ( network.available() )
  {
    RF24NetworkHeader header;
    network.read(header, &rec, sizeof(rec));
    if (rec.id == 'A')
    {
      A_status = rec;
    }
    else if (rec.id == 'C')
    {
      C_status = rec;
    }
  }
}

//4) status routine - set alpha mode //sets the flag
//                  - set beta mode
void set_alpha()
{
  digitalWrite(LED, HIGH);
}

void set_beta()
{
  digitalWrite(LED, LOW);
  delay(500);                  //TO BE FINE TUNED ACCORDING TO THE PROGRAM
  digitalWrite(LED, HIGH);
  delay(500);
}

void navigate ()
{
  if (get_ultra_valF() > 10)
  {
    if ( flag == 1)
    {
      d = get_ultra_valF();
      flag = 0;
    }
    FWD();
  }
  else
  {
    flag = 1;
    Dist = d - get_ultra_valF();
    communicate();
    // Serial.println(dist);
    if(get_ultra_valB() > 10)
      REV();
    else
      STOP();
    scan_area(deg);
    /* if(rng > 6)
     {*/
    align(deg);
    if (deg == 90)
    {
      FWD();

    }
    //}


  }

  delay(50);
}

void align(int deg)
{
  if (deg < 90)
  {
    RIGHT();
    delay((90 - deg) * 8); // TAKE HEADING REFERENCE
    STOP();


  }
  else if (deg > 90)
  {
    LEFT();
    delay((deg - 90) * 8); // TAKE HEADING REFERENCE
    STOP();

  }
}
void communicate()   // better way
{
   for (int retry = 0; retry < 5; retry += 1)
    {
      send_data('B', heading, trgt, Dist, State);
      delay(10);
      network.update();
    }

    get_data();
    network.update();

}
