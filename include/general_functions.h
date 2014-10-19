/*
 * general_functions.h
 *
 *  Created on: Oct 18, 2014
 *      Author: guywenck
 */

#ifndef GENERAL_FUNCTIONS_H_
#define GENERAL_FUNCTIONS_H_



#endif /* GENERAL_FUNCTIONS_H_ */

//////////
//Motors//
//////////
typedef struct {
	unsigned char port;
	unsigned char imeAddress;
	bool reversed;
} Motor;

extern unsigned char connectedIntegratedMotorEncoders;

Motor createMotor(unsigned char port, bool reversed);
Motor createMotorWithIME(unsigned char port, unsigned char imeAddress, bool reversed);

extern Motor frontLeftWheel;
extern Motor frontRightWheel;
extern Motor backLeftWheel;
extern Motor backRightWheel;

extern Motor topLeftLift;
extern Motor middleLeftLift;
extern Motor bottomLeftLift;
extern Motor topRightLift;
extern Motor middleRightLift;
extern Motor bottomRightLift;

/////////
//Drive//
/////////

typedef enum{
	same,
	towards,
	away
} WheelDirection;

typedef enum{
	left,
	right,
	forward,
	backward
}Direction;

extern const int DRIVE_THRESHOLD;

void setRightMotorSpeed(int speed, WheelDirection dir);
void setLeftMotorSpeed(int speed, WheelDirection dir);

void strafe(int speed, Direction dir);
void drive(int speed, Direction dir);

void handleDriveInput();
void handleStrafingInput();

void handleDriveOrStrafing();

////////
//Lift//
////////

extern const short liftHighPower;
extern const short liftLowPower;

////////////
//Controls//
////////////

extern unsigned char connectedJoysticks;

typedef struct {
	unsigned char btn;
	unsigned char channel;
	bool onPartnerJoystick;

} JoyBtn;

extern JoyBtn liftUp;
extern JoyBtn liftDown;
extern JoyBtn forward_backward_drive;
extern JoyBtn left_right_drive;
extern JoyBtn forward_backward_strafe;
extern JoyBtn left_right_strafe;

int readJoystick(JoyBtn btn);

JoyBtn createButton(unsigned char channel, unsigned char btn);
JoyBtn createButtonOnPartnerJoystick(unsigned char channel, unsigned char btn);
JoyBtn createAxis(unsigned char channel);
JoyBtn createAxisOnPartnerJoystick(unsigned char channel);

void setMotorPower(Motor m, int speed);
void setLiftPower(int speed);
unsigned char getNumConnectedJoysticks();
