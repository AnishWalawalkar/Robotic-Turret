/********************************************************************
* A-1: MTE100/GENE121 1A Clinic Project - Automated & Manual Turret
=====================================================================
* Purpose: To control the electronics component of the
	Automated Turret using the Lego NXT2.0 Robot
* Name: Victor Vuong, Abhijith Ramalingam, Anish Walawalkar, Sungjin Lee
* Date: November 23rd/13
* Language: RobotC
********************************************************************/

//Program Constants:
const int RANGE = 181;

const int LTOUCH = S1;
const int RTOUCH = S2;
const int COLOUR = S3;
const int SONAR = S4;

const int DRIVEMOTOR = motorA;
const int TURRETMOTOR = motorB;
const int BULLETMOTOR = motorC;
	
//Global Variables:
float dist[RANGE];
bool theta[RANGE];
bool target[RANGE];

/********************************************************************
 A-2 Function: defaultPos
=====================================================================
* Written By: Victor Vuong
* Purpose: Sets the TURRETMOTOR to the initial position, which is 0
	degrees on the horizontal axis, and resets the motor encoder
	so the encoder values can be used a reference.
* Parameters:None
* Return Value:None
********************************************************************/
void defaultPos()
{
	int prev = 360; //arbitrary initial value
	motor[TURRETMOTOR] = -5;
	while (abs(nMotorEncoder[TURRETMOTOR] - prev) > 5)
	{
		prev = nMotorEncoder[TURRETMOTOR];
		wait10Msec(75);
	}
	motor[TURRETMOTOR] = 0;
	nMotorEncoder[TURRETMOTOR] = 0;
}

/********************************************************************
A-3 Function: defaultBullet
=====================================================================
* Written By: Victor Vuong
* Purpose:Sets the BULLETMOTOR to its optimal firing position
	(all the way back). 
* Parameters: None
* Return Value: None
********************************************************************/
void defaultBullet()
{
	motor[BULLETMOTOR] = 0;
	wait10Msec(10);
	motor[BULLETMOTOR] = 10;
	wait1Msec(1000);
	motor[BULLETMOTOR] = 0;
}

/********************************************************************
A-4 Function: initSetup
=====================================================================
* Written By: Victor Vuong
* Purpose: Initializes the global arrays and moves the mechanical
	components to their initial positions.
* Parameters: None
* Return Value: None
********************************************************************/
void initSetup()
{
	eraseDisplay();
	for (int i = 0; i< RANGE; i++)
	{
		dist[i] = 0;
		theta[i] = false ;
		target[i]= false;
	}
	nxtDisplayString(0,"initSetup");
	
	nxtDisplayString(1,"defaultPos");
	defaultPos();

	nxtDisplayString(1,"defaultBullet");
	defaultBullet();
}

/********************************************************************
A-5 Function: scanArea
=====================================================================
* Written By: Sungjin Lee
* Purpose: Scans the area around the robot and stores the different
	distances in an array. It then uses these values to determine the
	position of a target inside the vicinity of the robot.
* Parameters: None
* Return Value: None
********************************************************************/
void scanArea()
{
	nxtDisplayString(0,"scanArea");
	motor[TURRETMOTOR] = 2;
	float d;
	int curAngle;

	while(nMotorEncoder[TURRETMOTOR] < RANGE)
	{
		d = SensorValue[SONAR];
		curAngle = nMotorEncoder[TURRETMOTOR];
		dist[curAngle] = d;

		nxtDisplayString(1,"A: %d", curAngle);
		nxtDisplayString(2,"D: %.1f", d);
		nxtDisplayString(3,"Searching...    ");

		//the robot only fires at targets within an optimal 
		//	range of 50cm.
		if(d <= 50) 
		{
			nxtDisplayString(3,"--Target Found--");
			curAngle = nMotorEncoder[TURRETMOTOR];
			theta[curAngle] = true;
			dist[curAngle] = d;
			wait10Msec(10);
		}
	}

	motor[TURRETMOTOR] = 0; //stops the motor
	nxtDisplayString(3,"Scan Complete.");
	wait10Msec(100);
	nxtDisplayString(4,"Please Wait...");
	defaultPos(); //move the turret back to the default position
}

/********************************************************************
A-6 Function: refineData
=====================================================================
* Written By: Abhijith Ramalingam
* Purpose: Fixes the erroneous readings of the ultrasonic sensor by
	adding true conditions whenever a small gap of bad values are found.
	This ensures the robot does not read multiple targets and ignores
	bad and incomplete readings of the ultrasonic sensor
* Parameters: None
* Return Value: None
********************************************************************/
void refineData()
{
	int  i=0, begin =0, end =0, TOL = 5;

	// the program looks for when the 1's start (where a target has 
	//	been detected)
	while(begin < RANGE && theta[begin] != 1) 
			begin++; 
	// begin is now set to the first stream of 1's

	while (begin < RANGE && end < RANGE)
	{
		if(begin < RANGE)
		{
			// Looks for when the 1's end 
			//	(where the target was last detected)
			while(begin < RANGE && theta[begin] ==1) 
				begin++;

			if(begin == RANGE)
				break ;
			
			end = begin ;
			
			// Looks for when the 1's start AGAIN or when 0's end
			while((end+1) < RANGE && theta[end+1] != 1) 
				end++; 
			// end now has the index value in which the stream 
			//	of 0's end
			
			//determines if the gap is small enough that there was 
			//	a misreading
			if((end-begin) < TOL)
				for(i=begin; i<=end;i++)
					theta[i] = true;
			begin = end + 1;
		}
	}
}

/********************************************************************
A-7 Function: pinpointTarget
=====================================================================
* Written By: Victor Vuong
* Purpose: Analyzes the data collected in the array and systematically
	Finds and pinpoints the correct angle to shoot down the detected 
	target(s).
* Parameters: None
* Return Value: # of targets detect by the robot
********************************************************************/
int pinpointTarget()
{
	eraseDisplay();
	nxtDisplayString(0,"pinpointTarget");
	int tCount = 0;
	int end = 0, begin = 0;
	nxtDisplayString(1,"Analyzing Data...");
	while (begin < RANGE && end < RANGE)
	{
		//determines where the trail of 1s in the angle array begins
		while(begin < RANGE && theta[begin] != 1) 
			begin++;
		if (begin < RANGE)
		{
			end = begin;
			//determines the end of the trail and ensures 
			//	that the object is within the distance tolerance
			while ((end+1) < RANGE && theta[end+1] == 1) 
				end++;
			if (theta[end] == 1 && (end - begin) > 1)
			{
				tCount++;
				target[(begin+end)/2] = true;
			}
			begin = end + 1;
		}
	}
	eraseDisplay();
	int i=0;
	while( i < RANGE )
	{
		if( target[i] !=0)
		{
			nxtDisplayString(0,"targetAt = %d " ,i);
			wait10Msec(20);
		}
		i++;
	}
	return tCount;
}

/********************************************************************
A-8 Function: fireProjectile
=====================================================================
* Written By: Victor Vuong
* Purpose: Directs the turret to go to all positions at which targets were
  found and shoot at them
* Parameters: the power at which the motor should fire the bullets at
* Return Value: None
********************************************************************/
void fireProjectile(int firingPower)
{
	for(int i=0; i<RANGE; i++)
	{
		eraseDisplay();
		nxtDisplayString(0,"fireProjectile");
		nxtDisplayString(1,"target[%d]", i);
		
		if(target[i]==1)
		{
			nxtDisplayString(2,"--TARGET FOUND--");

			nxtDisplayString(3,"Positioning...  ");
			motor[TURRETMOTOR] = 5;
			while(nMotorEncoder[TURRETMOTOR] < i);
			motor[TURRETMOTOR] = 0;
			nxtDisplayString(3,"Ready...        ");

			nxtDisplayString(3,"Firing...       ");
			motor[BULLETMOTOR] = firingPower;
			wait1Msec(110);
			defaultBullet();
			eraseDisplay();
	 }
	}
	nxtDisplayString(3,"DefaultPos          ");
	defaultPos();
}

/********************************************************************
A-9 Function: runAuto
=====================================================================
* Written By: Victor Vuong
* Purpose: Calls all the above function and directs the robot to find and
  fire at given targets
* Parameters: None
* Return Value: None
********************************************************************/
void runAuto()
{
	eraseDisplay();
	nxtDisplayString(0,"runAutoTurret");

	nxtDisplayString(1,"Press Any Key...");
	while (nNxtButtonPressed ==-1);
	while (nNxtButtonPressed != -1);

	initSetup();
	wait10Msec(100);
	
	eraseDisplay();
	scanArea();
	wait10Msec(100);
	
	refineData() ;
	nxtDisplayString(1,"Data Refined");
	wait10Msec(100);

    int numTarget = pinpointTarget();
	wait10Msec(100);
	nxtDisplayString(1,"Found %d Targets",numTarget);
  	wait10Msec(100);
	
	if (numTarget >0)
		fireProjectile(-90);
}

/********************************************************************
A-10 Function: manualDrive
=====================================================================
* Written By: Anish Walawalkar
* Purpose: If manual mode of the robot is engaged, this function
  allows the user to drive back and forth and utilize the machine gun
  mode of the turret where it fires a large number of bullets continuously
* Parameters: None
* Return Value: None
********************************************************************/
void manualDrive()
{
	if(SensorValue[RTOUCH]==1 && SensorValue[LTOUCH]==0)
		motor[DRIVEMOTOR]=30;
	else if(SensorValue[LTOUCH]==1 && SensorValue[RTOUCH]==0)
		motor[DRIVEMOTOR]=-30;
	else if (SensorValue[LTOUCH]==1 && SensorValue[RTOUCH]==1)
	{
		motor[DRIVEMOTOR]=0;
		motor[BULLETMOTOR]=-90;
	}
	else
	{
		motor[DRIVEMOTOR]=0;
		motor[BULLETMOTOR]=0;
	}
}

/********************************************************************
A-11 Function: manualFire
=====================================================================
* Written By: Anish Walawalkar
* Purpose: During the manual mode, this function calculates the speed
  at which the bullet should travel by controlling the power of the
  BULLETMOTOR. This is done by calculating the time the user holds down
  both the touch sensor buttons
* Parameters: None
* Return Value: None
********************************************************************/
void manualFire()
{
	while(SensorValue[LTOUCH]==0 && SensorValue[RTOUCH] == 0);
	time1[0]=0;
	while(SensorValue[LTOUCH]==1 && SensorValue[RTOUCH]==1);
	int a=(time1[0]>2500)? 100: (time1[0]/500+7)*10;
	motor[BULLETMOTOR]=-a;
	nxtDisplayString(5,"power= %d",a);
	wait1Msec(a);
	defaultBullet();
	eraseDisplay();
}

/********************************************************************
A-12 Function: manualRotate
=====================================================================
* Written By: Anish Walawalkar
* Date: November 14th/13
* Purpose: During the manual mode, this function allows the user to rotate
  the turret with the aid of two touch sensor buttons
* Parameters: None
* Return Value: None
********************************************************************/
void manualRotate()
{
	if(SensorValue[RTOUCH]==1 && SensorValue[LTOUCH]==0)
		motor[TURRETMOTOR]=5;
	else if(SensorValue[LTOUCH]==1 && SensorValue[RTOUCH]==0)
		motor[TURRETMOTOR]=-5;
	else if(SensorValue[LTOUCH]==1 && SensorValue[RTOUCH]==1)
	{
		motor[TURRETMOTOR]=0;
		manualFire();
	}
	else
		motor[TURRETMOTOR]=0;
}

/********************************************************************
A-13 Function: runManual
=====================================================================
* Written By: Anish Walawalkar
* Purpose: Calls the functions with the 'manual' prefix and enables 
	the user to manually operate the turret.
* Parameters: None
* Return Value: None
********************************************************************/
void runManual()
{
	eraseDisplay();
	nxtDisplayString(0,"runManual");
	nxtDisplayString(1,"Press Any Key...");
	while(nNxtButtonPressed == -1);
	while(nNxtButtonPressed != -1);
	nxtDisplayString(1,"Instructions:....");
	nxtDisplayString(2,"Use Controller.");
	nxtDisplayString(3,"to fire and move");
	nxtDisplayString(4,"White Mode: Fire ");
	nxtDisplayString(5,"Black Mode: Move ");

	while(true)
	{
		nxtDisplayString(6,"Colour: %d",SensorValue[COLOUR]);
		//black
		if(SensorValue[COLOUR] >= 1 && SensorValue[COLOUR] <= 3) 
			manualDrive();
		//white
		else if(SensorValue[COLOUR]>=4 && SensorValue[COLOUR]<=6) 
			manualRotate();
	}
}

/********************************************************************
A-14 Function: mainMenu
=====================================================================
* Written By: Victor Vuong
* Purpose: This calls all the above functions and gives options to the
  operator to either let the turret fire in automatic mode or operate it
  manually. It also has an option to give the details about the
  team members.
* Parameters: None
* Return Value: None
********************************************************************/
void mainMenu()
{
	while (true)
	{
		eraseDisplay();
		nxtDisplayString(0,"Group 23:");
		nxtDisplayString(1,"   --Turret--   ");
		nxtDisplayString(2,"Commands:");
		nxtDisplayString(3,"Middle: Auto");
		nxtDisplayString(4,"Left: Manual");
		nxtDisplayString(5,"Right: About");
		nxtDisplayString(6,"Bottom: Exit");

		while (nNxtButtonPressed == -1);
		int btn = nNxtButtonPressed;
		while (nNxtButtonPressed != -1);
		
		if (btn == 3)
			runAuto();
		else if (btn == 2)
		{
			initSetup();
			runManual();
		}
		else if(btn == 1)
		{
			eraseDisplay();
			nxtDisplayString(0,"by");
			nxtDisplayString(1," Victor Vuong");
			nxtDisplayString(2," A. Ramalingam");
			nxtDisplayString(3," Anish Walawalkar");
			nxtDisplayString(4," Sungjin Lee");
			nxtDisplayString(5,"MTE100 & GENE121");
			nxtDisplayString(6,"1A, November 2013");
			while (nNxtButtonPressed == -1);
			while (nNxtButtonPressed != -1);
		}
	}
}

/********************************************************************
A-15 Function: main
=====================================================================
* Written By: Victor Vuong
* Purpose: Sets up the hardware sensors and calls main menu to thereby 
	operate the turret.
* Parameters:None
* Return Value: None
********************************************************************/
task main()
{
	SensorType[SONAR] = sensorSONAR;
	SensorType[LTOUCH] = sensorTouch;
	SensorType[RTOUCH] = sensorTouch;
	SensorType[COLOUR] = sensorCOLORFULL;

	mainMenu();
}
