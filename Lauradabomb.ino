//using interrupts is unreliable, just use normal digitalRead if statement
//pins 2 or 3 alone can be used for interrupts (for the Uno)

#include <Servo.h>


//Defines
#define LINEAR50PIN 10        //Linear Actuator Digital Pin

//max/min pulse values in microseconds for the linear actuators
#define LINEAR50_MIN  1050     //
#define LINEAR50_MAX  2000    //


Servo LINEAR50;  // create servo objects to control the linear actuators
const int contactPin = 3; 

int linear50Value = 1500;  //current positional value being sent to the linear actuator. 

bool contactMade = false;

void setup() 
{ 
  Serial.begin(9600);
  pinMode(contactPin,INPUT);
  //initialize servos
  LINEAR50.attach(LINEAR50PIN, LINEAR50_MIN, LINEAR50_MAX);  // attaches/activates the linear actuator as a servo object 

  //Analog pins do not need to be initialized
  
  //use the writeMicroseconds to set the linear actuators to their default positions
  LINEAR50.writeMicroseconds(LINEAR50_MAX);
  delay(10000);
} 

void loop() 
{ 
   contactMade = digitalRead(contactPin);
  if(contactMade) {
    Serial.println("On");
    // move linear actuator in
    LINEAR50.writeMicroseconds(LINEAR50_MIN);
    delay(10000);
  } else {
    Serial.println("Off");
    LINEAR50.writeMicroseconds(LINEAR50_MAX);
  }
} 
