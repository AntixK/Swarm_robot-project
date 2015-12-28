This code is for our B.Tech project entitled "Development of Autonomous robot swarm 
for goal searching using ALpha Beta coodination" 
- Anand Krishnamoorthy, Girish Kumar, Atul Balachandran   

#Team
Anand krishnamoorthy, Girish Kumar, Atul B Chandran, Prof. J. Balaji

#Project Summary
In this project, we aimed to create a minor LIM-GROUP, ARR-DYN type swarm robots to perform a goal searching procedure. The objective is to implement the Alpha-Beta coordination algorithm for effective goal searching. All the agents are to be monitored by the end-user through a computer (Home Board). The performance of the swarm was tested in various scenarios and the results were analysed. The importance of this algorithm can be realised as a basis for many complex dynamic task allocation algorithms. 

#Code Description
These are the Arduino .ino files for each swarm robot labelled 'A', 'B' and 'C'. The code implements the Apha-Beta algorithm for 3 mini autonomous swarm robots to find and locate the RF beacon situated within the arena and converge to it.

ALPHA BETA FLOW CHART

  /* Alpha beta algorithm*/

   START

  * step 1: establish connection - ping test :
      each bot shud send and receive their ID to all others including HB */

  * Step 2: set alpha mode for all the bots:
     (i)  Turn on the red LED light
     (ii) disperse in different directions- at 20, 140, 260 deg for each bot depending on how close the direction is to each bot
     (iii) move in the same direction and navigate through obstacles while communicating the sensor measures to other bots and HB */

  * Step 3: check the time elapsed
     if it is less than threshold time then the step 2 actions are continued
     else execute step 4 */

  * Step 4: Switching mode phase 1: Bsed on the sensor measure execute beta mode (the lowest measure bot becomes the beta bot)
    (i) switch off the red LED and ON the blue LED
    (ii) stop navigation and keep receiving the data from other bots
    (iii) follow the highest sensor measure bot SLOWLY (i.e move every small intervals of time)
    (iv) still keep getting ur sensor measure... */

  * Step 5: Check the time elapsed
     if it is less than the second time threshold, then continue step 4 irrespective of the
     sensor measure sof the other two bots else step 6 */

  * Step 6: Switching mode phase 2: if sensor measure of the other two alpha bots is
     less than or equal to the current measure of the beta bot then switch that bot to the beta mode as shown in Step 4
     (by the end there will be only one alpha bot) the bots must continue their nagivation towards the target while the other
      alpha bot wanders until the third threshold after which it switches to beta mode*/

  * Step 7: the beta bots must nagivate till it reaches the target whose position will be confirmed by the HB and
     all the bots shall converge at the target */

  STOP

The is under GPL v3.0 license. You can duplicate, modify the code freely.

copyright (c) 2015 Anand Krishnamoorthy

For More information: [Swarm Project] (https://sites.google.com/site/swarmrobotproject/)
