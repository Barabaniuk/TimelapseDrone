/*
Library for controlling the robotic platform for slow motion video filming

 Version 1.10 dated 23.06.15
*/
#include "Arduino.h"
#include "AFMotor.h"
#include "Servo.h"
#include "TimeLapse.h"
  MainStepper::MainStepper(int pin):StepMotor(200,pin) 	//Constructor for the stepper motor control class
  {
  }
  void MainStepper::Update()
  {
    if (Mode_StepperPause)    // Stepper motor waiting mode
    {
      if ((millis()-StepperPauseTimer)>=StepperStepInterval)
      {
        Mode_StepperStep=true;
        Mode_StepperPause=false;
        StepperPauseTimer=millis();
        
      }
    }
    if (Mode_StepperStep)    // Stepper motot move mode
    {
      if (StepperStep==0)    // If all steps dine - stop
      {
        Mode_StepperStep=false;
        Mode_StepperPause=false;
        StepMotor.release();
      }
      else
      {
      StepMotor.onestep(StepperDirection,StepperStepstyle);
      StepperStep--;
      Mode_StepperStep=false;
      Mode_StepperPause=true;
      }
    }
  }
  void MainStepper::AddStepper(int angle,int interval,int movespeed=1,int stepstyle=INTERLEAVE)  // Add steps to stepper motor
  {
    if (angle>=0)
    {
      StepperDirection=FORWARD;
      StepperStep=angle;
    }
    else
    {
      StepperDirection=BACKWARD;
      StepperStep=(-1)*angle;
    }
    StepperStepInterval=interval;
    StepperStepstyle=stepstyle;
    Mode_StepperStep=true;
    Mode_StepperPause=false;
    
  }

  MainServo::MainServo() : Servos()
  {
	
  }
  void MainServo::AttachServo(int pin, int posmin, int posmax, int posinit)  // Servo initialization
  {
    Servos.attach(pin);  // Servo pins initialization
    ServoMIN=posmin;    // Set extreme positions
    ServoMAX=posmax;
    ServoPos=posinit;      // Set to initial position
    Servos.write(ServoPos);
  }
   void MainServo::AddServoStep(int angle,int steptime, int interval=1)    // Add step movement to servo motor
  {
    if (angle>=0) ServoIncrement=interval; else ServoIncrement=(-1)*interval;
    Mode_ServoStep=true;
    ServoStep=abs(angle);
    Mode_ServoStep=true;
    ServoStepInterval=steptime;
    ServoTimer=millis()+ServoStepInterval;  // Firs step immidiately, than check previous inteval
  }
  void MainServo::Update()
  {
    if (Mode_ServoStep)    // 1st servo movement mode
    {
      if (ServoStep==0)    // If all steps done - stop movement 
      {
        Mode_ServoStep=false;
      }
      else
      {
        if ((millis()-ServoTimer)>=ServoStepInterval)
        {
          
            ServoPos+=ServoIncrement;
            Servos.write(ServoPos);
    
          ServoStep--;
          ServoTimer=millis();
        }
      }
    }
  }
  


  MainPlatform::MainPlatform() : MainStepper2(1), MainStepper1(2),  MainServo1(), MainServo2()  //Platform class constructor, connecting motors to pins 1 and 2 of the  block, Stepper motor to pins 3 and 4

  { 
	 //Setting default parameters
	ProgramRun=false;
 	ProgramPause=false;
	Actions=0;
	ActionsLoaded=0;
	Serial.begin(9600);
  }
  void MainPlatform::AttachServo()  // Servo initialization
  {
    MainServo1.AttachServo(9,Servo1MIN,Servo1MAX,Servo1INIT);
    MainServo2.AttachServo(10,Servo2MIN,Servo2MAX,Servo2INIT);
  }


  void MainPlatform::Update()  // Made all movement
  {
    unsigned long timer=millis();

    MainServo1.Update();
    MainServo2.Update();
    MainStepper1.Update();
    MainStepper2.Update();
  }


void MainPlatform::AddAction_ForwardMotor(int hours,int minutes,int sec,int p1,int p2)	// adding main motor turn by timer  - p1-angle, p2-time to turn
{
	if (TestMode)
	{
	   unsigned long timer=TestActionTimer;
	   int ActionTestTime=int(abs(p1*TestStepperInterval)/1000);
	   Program[Actions][0]=timer;
	   Program[Actions][1]=1;
	   Program[Actions][2]=p1;	//Rotation angle
	   Program[Actions][3]=ActionTestTime; //Required time
           TestActionTimer+=ActionTestTime+1;
	    Serial.print("Add action: ");
            Serial.print(Actions);
	    Serial.print(" - RotateArm (TEST)");
	    Serial.print(" , time=");
            Serial.println(timer);
Actions++;

	}
	else
	{
unsigned long timer=(sec*1)+(minutes*60)+(hours*3600);
    Program[Actions][0]=timer;
    Program[Actions][1]=1;
    Program[Actions][2]=p1;
    Program[Actions][3]=p2;
    Serial.print("Add action: ");
    Serial.print(Actions);
    Serial.print(" - RotateArm");
    Serial.print(", time=");
    Serial.println(timer);
    Actions++;
}
}
void MainPlatform::AddAction_RotateArm(int hours,int minutes,int sec,int p1,int p2)	// adding manipulator main joint movement  - p1-angle, p2-time to turn
{
	if (TestMode)
	{
	   unsigned long timer=TestActionTimer;
	   int ActionTestTime=int(abs(p1*TestStepperInterval)/1000);
	   Program[Actions][0]=timer;
	   Program[Actions][1]=3;
	   Program[Actions][2]=p1;	//Rotation angle
	   Program[Actions][3]=ActionTestTime; //Required time
           TestActionTimer+=ActionTestTime+1;
	    Serial.print("Add action: ");
            Serial.print(Actions);
	    Serial.print(" - RotateArm (TEST)");
	    Serial.print(" , time=");
            Serial.println(timer);
Actions++;

	}
	else
	{
unsigned long timer=(sec*1)+(minutes*60)+(hours*3600);
    Program[Actions][0]=timer;
    Program[Actions][1]=3;
    Program[Actions][2]=p1;
    Program[Actions][3]=p2;
    Serial.print("Add action: ");
    Serial.print(Actions);
    Serial.print(" - RotateArm");
    Serial.print(", time=");
    Serial.println(timer);
    Actions++;
}
}
void MainPlatform::AddAction_RotateBottomPart(int hours,int minutes,int sec,int p1,int p2) // adding manipulator bottom  joint movement  - p1-angle, p2-time to turn
{
	if (TestMode)
	{
	   unsigned long timer=TestActionTimer;
	   int ActionTestTime=int(abs(p1*TestServoInterval)/1000);
	   Program[Actions][0]=timer;
	   Program[Actions][1]=5;
	   Program[Actions][2]=p1;	//Rotation angle
	   Program[Actions][3]=ActionTestTime; //Required time
           TestActionTimer+=ActionTestTime+1;
	    Serial.print("Add action: ");
    	Serial.print(Actions);
	    Serial.print(" - RotateBottomPart (TEST)");
	    Serial.print(" , time=");
            Serial.println(timer);
Actions++;
	}
	else
	{
unsigned long timer=(sec*1)+(minutes*60)+(hours*3600);
    Program[Actions][0]=timer;
    Program[Actions][1]=5;
    Program[Actions][2]=p1;
    Program[Actions][3]=p2;

    Serial.print("Add action: ");
    Serial.print(Actions);
    Serial.print(" - RotateBottomPart");
    Serial.print(", time=");
    Serial.println(timer);
    Actions++;
	}
}
void MainPlatform::AddAction_SetBottomPart(int hours,int minutes,int sec,int p1) // adding manipulator bottom  joint movement  - p1-position
{
	if (TestMode)
	{
	   unsigned long timer=TestActionTimer;
	   int ActionTestTime=int(abs(p1*TestServoInterval)/1000);
	   Program[Actions][0]=timer;
	   Program[Actions][1]=6;
	   Program[Actions][2]=p1;	//”гол поворот
	    Serial.print("Add action: ");
            Serial.print(Actions);
	    Serial.print(" - SetBottomPart (TEST)");
	    Serial.print(" , time=");
            Serial.println(timer);
Actions++;
	}
	else
	{
	   unsigned long timer=TestActionTimer;
	   int ActionTestTime=int(abs(p1*TestServoInterval)/1000);
	   Program[Actions][0]=timer;
	   Program[Actions][1]=6;
	   Program[Actions][2]=p1;

    Serial.print("Add action: ");
    Serial.print(Actions);
    Serial.print(" - SetBottomPart");
    Serial.print(", time=");
    Serial.println(timer);
    Actions++;
	}
}
void MainPlatform::AddAction_SetVeeringPart(int hours,int minutes,int sec,int p1) // adding manipulator main  joint movement  - p1-position
{
	if (TestMode)
	{
	   unsigned long timer=TestActionTimer;
	   int ActionTestTime=int(abs(p1*TestServoInterval)/1000);
	   Program[Actions][0]=timer;
	   Program[Actions][1]=7;
	   Program[Actions][2]=p1;	//”гол поворот
	    Serial.print("Add action: ");
            Serial.print(Actions);
	    Serial.print(" - SetVeeringPart (TEST)");
	    Serial.print(" , time=");
            Serial.println(timer);
Actions++;
	}
	else
	{
	   unsigned long timer=TestActionTimer;
	   int ActionTestTime=int(abs(p1*TestServoInterval)/1000);
	   Program[Actions][0]=timer;
	   Program[Actions][1]=7;
	   Program[Actions][2]=p1;

    Serial.print("Add action: ");
    Serial.print(Actions);
    Serial.print(" - SetVeeringPart");
    Serial.print(", time=");
    Serial.println(timer);
    Actions++;
	}
}
    int i=0;
void MainPlatform::LoadAction()		// Processing the action program and initializing the desired action at the time specified for it
{

  if (ProgramRun)
  {
long CurrentTiming=long(millis()-CurrentTime);	// ¬рем€, прошедшее с запуска программы
    for (i=ActionsLoaded; i<Actions; i++)
    {
      unsigned long ProgActTimer=1000*long(Program[i][0]);	// Time elapsed since program start
      /* Serial.print("Action #");
	Serial.print(i);
        Serial.print(" - ");
        Serial.print(CurrentTiming);
        Serial.print("/");
      Serial.println(ProgActTimer);*/
      
      if (CurrentTiming >= ProgActTimer )  // If more time has elapsed since the beginning of the program than is set for the start of the action
      {
        Serial.print("Action #");
	Serial.print(i+1);
        Serial.print("/");
	Serial.print(Actions);
        Serial.print(", timing=");
	Serial.println(CurrentTiming);
        if (Program[i][1]==1) //Stepper motor of main movement
        {
           int StepsNum=Program[i][2];
	   int StepInt=abs(int(float(Program[i][3])/float(StepsNum)*1000));
	    Serial.print("Steps for rotation=");
	    Serial.print(StepsNum);
	    Serial.print(", Interval between steps=");
	    Serial.print(StepInt);
	    Serial.println(" ms;");
           MainStepper2.AddStepper(StepsNum,StepInt);
           ActionsLoaded++;
        }  else

        
        if (Program[i][1]==3) //Rotation stepper motor 
        {
	int StepsNum=Program[i][2];
	int StepInt=abs(int(float(Program[i][3])/float(StepsNum)*1000));
	    Serial.print("Steps for rotation=");
	    Serial.print(StepsNum);
	    Serial.print(", Interval between steps=");
	    Serial.print(StepInt);
	    Serial.println(" ms;");
           MainStepper1.AddStepper(StepsNum,StepInt);
           ActionsLoaded++;
        }else
        if (Program[i][1]==5) //Servo 1
        {

		int StepInt=abs(int(float(Program[i][3])/float(Program[i][2])*1000));
/*Serial.print("StepsNum=");
	    Serial.print(Program[i][2]);
	    Serial.print(", StepInt=");
	    Serial.println(StepInt);*/
           MainServo1.AddServoStep(-Program[i][2],StepInt);
           ActionsLoaded++;
        }else
        if (Program[i][1]==6) //Servo 1 - instant movement
        {
	  MainServo1.ServoPos=Program[i][2];
          MainServo1.Servos.write(MainServo1.ServoPos);
           ActionsLoaded++;
        }else
        if (Program[i][1]==7) //Servo 2 - instant movement
        {
	  MainServo2.ServoPos=Program[i][2];
          MainServo2.Servos.write(MainServo2.ServoPos);
           ActionsLoaded++;;

        }
        /*Serial.print(", Begin action=");
        Serial.println(ActionsLoaded-1);
        Serial.print("Timing=");
        Serial.print((millis()-CurrentTime));*/

      }
    }
  }
}

void MainPlatform::Start()	//Run novement 
{
	//Program=int[100][6];
	ProgramRun=true;
 	ProgramPause=false;
	ActionsLoaded=0;
	ProgramRun=true;
        ProgramPause=false;
        BeginingTime=millis();
        CurrentTime=BeginingTime;
        Serial.print("Program BEGIN:");
        Serial.print(" time=");
        Serial.println(BeginingTime);
}
void MainPlatform::Restart()
{
}
void MainPlatform::Pause()	// Pause
{
	  ProgramPause=true;
          PauseTime=millis()-BeginingTime;  //Time from program start
          Serial.print("Program pause:");
          Serial.print(" time=");
          Serial.print(millis());
          Serial.print(" program timer=");
          Serial.println(PauseTime);

}
void MainPlatform::Resume()		// resume from pause
{
	  ProgramPause=false;
          CurrentTime=CurrentTime+(millis()-PauseTime);    //Time from pause start
          Serial.print("Program restore:");
          Serial.print(" time=");
          Serial.print(millis());
          Serial.print(" program timer=");
          Serial.println(CurrentTime);	
}


