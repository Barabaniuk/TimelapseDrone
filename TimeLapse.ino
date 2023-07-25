/*
Main code for controlling the robotic platform for slow motion video filming
Version 1.10 dated  23.06.15 (With two stepper motot and steering using servo)
*/
#include "AFMotor.h"
#include "Servo.h"
#include "TimeLapse.h"
MainPlatform Main;
void setup() 
{
  Serial.begin(9600);
  Main.TestMode=false;	// Test mode: 
//				=false - disabled, actions start according to the times specified for them
//				=true - enabled, actions begin immediately after the end of the previous one, the interval between the pulses of movement and rotation is minimal

//     ADDING PLATFORM ACTIONS EXAMPLE
// Linear movement
Main.AddAction_ForwardMotor(0,0,1, 240, 10);
 // at the time 0:0:1 (1 second from the beginning of the program) start the movement of the main drive. Take 240 steps (90 degrees) in 10 seconds

  // Manipulator rotation
  Main.AddAction_RotateArm        (0,0,45, 240, 15);
   // At the time 0:0:45 (45 seconds from the beginning of the program) start turning the manipulator.
   // rotate the manipulator 240 steps in 15 seconds, which corresponds to (360/960)*240=90 degrees
   // One step is 0.375 degrees. One revolution - 960 steps.
  
  // Manipulator lifting
  Main.AddAction_RotateBottomPart  (0,1,0, 90, 14);
  // At time 0:1:0 (60 seconds from the beginning of the program) start lifting the manipulator.
   // rotate the lower hinge 90 degrees clockwise in 14 seconds
  
  Main.AddAction_SetBottomPart  (0,1,15, 90);
  // At the time 0:1:15 (75 seconds from the beginning of the program) set the angle of rotation of the manipulator to 40 degrees
  Main.AddAction_SetVeeringPart  (0,1,30, 90);
  // At the time 0:1:30 (90 seconds from the beginning of the program) set the angle of rotation for the steering wheels to 100 degrees
  
  // Consider the front side of the manipulator, on which the fastening couplings of the servos are located. A positive angle corresponds to a clockwise rotation, a negative angle counterclockwise
   // Initial position:
   // - lower hinge in the extreme left-lower position (corresponds to an angle of 180)
   // - upper hinge in the extreme right-upper position (corresponds to angle 0)
  
   // END OF ACTION LIST
   
  pinMode(2,INPUT);        // Start stop button on PIN2
  digitalWrite(2, HIGH);   // The default pin state is positive, the button closes to ground when pressed
  Main.AttachServo();      // Servo initialization
  Main.ProgramRun=false;
}
int ButtonState=HIGH;

void(* resetFunc) (void) = 0; // Controller reset
unsigned long StateTime;
unsigned long ClickTime;
int ClickDuration;
const int SingleClick=20;
const int LongClick=1000;
boolean Click=false; 
void loop() 
{
  if ((Main.ProgramRun)&&(!(Main.ProgramPause)))
  {
  Main.Update();
  Main.LoadAction();
  }
  
  if ((millis()-StateTime)>=10) // Button polling every 10ms
  {
    if ((digitalRead(2)==LOW)&&(ButtonState==HIGH))  // Button pressed, previously released 
    {
      if (!Click)
      {
        ClickTime=millis();
        Click=true;
      }
    }
    /*if ((Click)&&((millis()-ClickTime)>=LongClick))
    {
         Click=false;
         ClickTime=millis();
         ClickDuration=0;
         Main.ProgramRun=false;
         resetFunc();
    }*/
    if ((digitalRead(2)==HIGH)&&(ButtonState==LOW)) // Button released, previously pressed
    {
      if (Click)
      {
        ClickDuration=millis()-ClickTime;
        //Serial.print("Click, duration=");
        //Serial.println(ClickDuration);
        Click=false;
      }
      if ((ClickDuration>=SingleClick)&&(ClickDuration<=LongClick))  // The duration of a click is greater than the minimum for a single click, but less than for a long one.
      {
        //Serial.println("SingleClick");
        if (!Main.ProgramRun)
        {
           Main.Start();
        }
        else 
        {
          if (Main.ProgramPause)
          {
             Main.Resume();
          }
          else 
          {
            Main.Pause();
          }
        }
      }
      else if (ClickDuration>=LongClick) // The click duration is greater than the minimum for a long click
      {
        //Serial.println("LongClick");
        MainPlatform Main;
        Main.ProgramRun=false;
        resetFunc(); 
      }
    }
  ButtonState=digitalRead(2);
  StateTime=millis();    
  }
}

