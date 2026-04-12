
/*
RCReader rev 4/9/2026 created by Aidan Carrier and Justin Sanders (Northeastern University Class of 2026 COE Capstone II)


Based on code from RoboSail
RCReader rev rev 7/30/2017
© 2014-2017 RoboSail


This program puts the Arduino micro-computer in the RC (Radio Control) system
It takes in the control signals coming in from the Receiver and
displays the following to the Serial Monitor:
  - The actual "pulse" coming in from the receiver for each channel
    (typical range of 1000 - 2000)
  - the angle at which the steering servo should be positioned
    given that command (in the RC car frame of reference)


This program helps the user determine
  - if they are reading good signals from the receiver (range of 943 - 1837)
  - if the Arduino computer is functioning correctly


Steering data from the RC receiver is read in on
digital pins 3.
*/


#include <Servo.h>


// Pin assignments
//input pins from receiver
#define STEERING_RC_PIN 3


// variables to hold input values
int steeringPulseWidth;
int steeringAngleOut;
int maxPulseWidth = 0;
int minPulseWidth = 9999;


void setup() {
  Serial.begin(115200);
  Serial.println("\nRCReader");
  // Set RC receiver on digital input pins
  pinMode(STEERING_RC_PIN, INPUT);
}


void loop() {
  // Read commanded (manual) values from the RC receiver
  // pulseIn returns the width of the command pulse in microseconds.
  steeringPulseWidth = pulseIn(STEERING_RC_PIN, HIGH);
  if (steeringPulseWidth != 0) {


    if (steeringPulseWidth > maxPulseWidth){
      maxPulseWidth = steeringPulseWidth;
    }


    if ( steeringPulseWidth < minPulseWidth) {
      minPulseWidth = steeringPulseWidth;
    }


  }
 
  steeringAngleOut = map(steeringPulseWidth, 942, 1835, -60, 60);


  // Print out the values for debug.
  Serial.print("steering pulse from receiver (microseconds): ");
  Serial.print(steeringPulseWidth);
  Serial.print("\t mapped steering angle (degrees): ");
  Serial.print(steeringAngleOut);


  Serial.print("\t (min: ");
  Serial.print(minPulseWidth);
  Serial.print("\t max: ");
  Serial.print(maxPulseWidth);
  Serial.println(")");
}
