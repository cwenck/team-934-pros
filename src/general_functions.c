#include "main.h"
#include "general_functions.h"

/////////////
//Variables//
/////////////

//Init Drive Motors
Motor frontLeftWheel;
Motor frontRightWheel;
Motor backLeftWheel;
Motor backRightWheel;

//Init Lift Motors
Motor topLeftLift;
Motor middleLeftLift;
Motor bottomLeftLift;
Motor topRightLift;
Motor middleRightLift;
Motor bottomRightLift;

//Init Controller Buttons
JoyBtn liftUp;
JoyBtn liftDown;

JoyBtn forward_backward_drive;
JoyBtn left_right_drive;
JoyBtn forward_backward_strafe;
JoyBtn left_right_strafe;

void handleAllInput() {
	handleDriveOrStrafing();
	handleLiftInput();
}

////////////
//Joystick//
////////////

JoyBtn createButton(unsigned char channel, unsigned char btn) {
	JoyBtn button;
	button.channel = channel;
	button.btn = btn;
	button.onPartnerJoystick = false;
	return button;
}

JoyBtn createButtonOnPartnerJoystick(unsigned char channel, unsigned char btn) {
	JoyBtn button;
	button.channel = channel;
	button.btn = btn;
	button.onPartnerJoystick = true;
	return button;
}

JoyBtn createAxis(unsigned char channel) {
	JoyBtn button;
	button.channel = channel;
	button.btn = NULL;
	button.onPartnerJoystick = false;
	return button;
}

unsigned char getNumConnectedJoysticks() {
	unsigned char joysticksConnected = 0;
	if (isJoystickConnected(1)) {
		joysticksConnected++;
	}
	if (isJoystickConnected(2)) {
		joysticksConnected++;
	}
	return joysticksConnected;
}

//If it is a button a 1 is returned for true and 0 for false
//otherwise a value between -127 and 127 is returned for the particular axis
int readJoystick(JoyBtn button) {
	unsigned char joy = 1;
	if (button.onPartnerJoystick) {
		joy = 2;
	}
	if (button.channel == 1 || button.channel == 2 || button.channel == 3 || button.channel == 4) {
		return joystickGetAnalog(joy, button.channel);
	} else {
		if (joystickGetDigital(joy, button.channel, button.btn)) {
			return 1;
		} else {
			return 0;
		}
	}
}

//////////
//Motors//
//////////

void setMotorPower(Motor motor, int speed) {
	if (motor.reversed) {
		speed = -speed;
	}
	motorSet(motor.port, speed);
}

Motor createMotor(unsigned char port, bool reversed) {
	Motor motor;
	motor.port = port;
	motor.reversed = reversed;
	motor.imeAddress = NULL;
	return motor;
}

//Integrated motor encoders closest to the cortex in the chain get assigned an address of 0
//encoders futher down the line get an address that is incremented down the line
//so the next further one from the cortex would be 1 then 2 then 3 etc.
Motor createMotorWithIME(unsigned char port, unsigned char imeAddress, bool reversed) {
	Motor motor;
	motor.port = port;
	motor.reversed = reversed;
	motor.imeAddress = imeAddress;
	return motor;
}

////////
//Lift//
////////

const short liftHighPower = 127;
const short liftLowPower = 80;

void setLiftPower(int speed) {
	setMotorPower(topLeftLift, speed);
	setMotorPower(middleLeftLift, speed);
	setMotorPower(bottomLeftLift, speed);
	setMotorPower(topRightLift, speed);
	setMotorPower(middleRightLift, speed);
	setMotorPower(bottomRightLift, speed);
}

void handleLiftInput() {
	if (readJoystick(liftUp) == 1) {
		setLiftPower(liftHighPower);
	} else if (readJoystick(liftDown) == 1) {
		setLiftPower(-liftHighPower);
	} else {
		setLiftPower(0);
	}
}

/////////
//Drive//
/////////

const int DRIVE_THRESHOLD = 5;

/*
 * A positive value for speed is forwards
 * A negetive value for speed is backwards (only works if the direction is same)
 */
void setRightMotorSpeed(int speed, WheelDirection dir) {
	if (dir == same) {
		setMotorPower(frontRightWheel, speed);
		setMotorPower(backRightWheel, speed);
	} else if (dir == towards) {
		speed = abs(speed);
		setMotorPower(frontRightWheel, -speed);
		setMotorPower(backRightWheel, speed);
	} else if (dir == away) {
		speed = abs(speed);
		setMotorPower(frontRightWheel, speed);
		setMotorPower(backRightWheel, -speed);
	}
}

/*
 * A positive value for speed is forwards
 * A negetive value for speed is backwards (only works if the direction is same)
 */
void setLeftMotorSpeed(int speed, WheelDirection dir) {
	if (dir == same) {
		setMotorPower(frontLeftWheel, speed);
		setMotorPower(backLeftWheel, speed);
	} else if (dir == towards) {
		speed = abs(speed);
		setMotorPower(frontLeftWheel, -speed);
		setMotorPower(backLeftWheel, speed);
	} else if (dir == away) {
		speed = abs(speed);
		setMotorPower(frontLeftWheel, speed);
		setMotorPower(backLeftWheel, -speed);
	}
}

/*
 * Strafe left and right at a particular speed
 * or move forawrd or backward
 */
void strafe(int speed, Direction dir) {
	speed = abs(speed);
	if (dir == left) {
		setLeftMotorSpeed(speed, towards);
		setRightMotorSpeed(speed, away);
	} else if (dir == right) {
		setLeftMotorSpeed(speed, away);
		setRightMotorSpeed(speed, towards);
	} else if (dir == forward) {
		setLeftMotorSpeed(speed, same);
		setRightMotorSpeed(speed, same);
	} else if (dir == backward) {
		setLeftMotorSpeed(-speed, same);
		setRightMotorSpeed(-speed, same);
	}
}

/*
 * Drive forward or backward or turn left or turn right at a particular speed
 */

void drive(int speed, Direction dir) {
	speed = abs(speed);
	if (dir == left) {
		setLeftMotorSpeed(-speed, same);
		setRightMotorSpeed(speed, same);
	} else if (dir == right) {
		setLeftMotorSpeed(speed, same);
		setRightMotorSpeed(-speed, same);
	} else if (dir == forward) {
		setLeftMotorSpeed(speed, same);
		setRightMotorSpeed(speed, same);
	} else if (dir == backward) {
		setLeftMotorSpeed(-speed, same);
		setRightMotorSpeed(-speed, same);
	}
}

void handleDriveInput() {
	if (abs(readJoystick(forward_backward_drive) > abs(readJoystick(left_right_drive)))) {
		if (readJoystick(forward_backward_drive) > 0) {
			drive(readJoystick(forward_backward_drive), forward);
		} else {
			drive(readJoystick(forward_backward_drive), backward);
		}
	} else {
		if (readJoystick(left_right_drive) > 0) {
			drive(readJoystick(left_right_drive), right);
		} else {
			drive(readJoystick(left_right_drive), left);
		}
	}
}

void handleStrafingInput() {
	if (abs(readJoystick(forward_backward_strafe) > abs(readJoystick(left_right_strafe)))) {
		if (readJoystick(forward_backward_strafe) > 0) {
			strafe(readJoystick(forward_backward_strafe), forward);
		} else {
			strafe(readJoystick(forward_backward_strafe), backward);
		}
	} else {
		if (readJoystick(left_right_strafe) > 0) {
			strafe(readJoystick(left_right_strafe), right);
		} else {
			strafe(readJoystick(left_right_strafe), left);
		}
	}
}

//Decides whether to drive or strafe
void handleDriveOrStrafing() {
	if (abs(readJoystick(forward_backward_drive)) <= DRIVE_THRESHOLD
			&& abs(readJoystick(left_right_drive)) <= DRIVE_THRESHOLD) {
		handleDriveInput();
	} else {
		handleStrafingInput();
	}
}

/////////////////
//Other Sensors//
/////////////////

//Init bumpers in initializeIO()
Bumper bumperInit(unsigned char port) {
	Bumper bumper;
	bumper.port = port;
	pinMode(bumper.port, INPUT);
	return bumper;
}

// digitalRead() returns LOW if Pressed or HIGH if released
// the function returns true if the bumper is pressed
bool bumperPressed(Bumper bumper) {
	if (digitalRead(bumper.port) == LOW) {
		return true;
	}
	return false;
}

// Init limit switches in initializeIO()
LimitSwitch limitSwitchInit(unsigned char port) {
	LimitSwitch limitSwitch;
	limitSwitch.port = port;
	pinMode(limitSwitch.port, INPUT);
	return limitSwitch;
}

// digitalRead() returns LOW if Pressed or HIGH if released
// the function returns true if the bumper is pressed
bool limitSwitchPressed(LimitSwitch limitSwitch) {
	if (digitalRead(limitSwitch.port) == LOW) {
		return true;
	}
	return false;
}

