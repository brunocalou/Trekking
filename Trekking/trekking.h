#ifndef TREKKING_H
#define TREKKING_H

#include "../Log/log.h"
#include "../XLMaxSonarEZ/sonarlist.h"
#include "../Robot/robot.h"
// #include "../Timer/timer.h"
#include "trekkingmath.h"
#include "position.h"
#include "locator.h"

// enum OperationMode {

// 	STANDBY,
// 	SEARCH_STAGE,
// 	REFINED_SEARCH_STAGE,
// 	LIGHTING
// };

class Trekking {
private:
	const float MAX_LINEAR_VELOCITY;  // [m/s]
	const float MAX_ANGULAR_VELOCITY; // [ang/s]

	float desired_linear_velocity;
	float desired_angular_velocity;

	LinkedList<Position *> targets;
	Position init_position;
	int current_target_index;

	const int COMMAND_BAUD_RATE;
	const int LOG_BAUD_RATE;
	const int ENCODER_BAUD_RATE;

	const byte MAX_MOTOR_PWM;
	const float MAX_RPS;
	const int LIGHT_DURATION;
	const float PROXIMITY_RADIUS;
	
	const int READ_ENCODERS_TIME;

	/*
		Input states
	*/

	bool init_button;
	bool emergency_button;
	bool operation_mode_switch;

	int min_distance_to_enable_lights;
	int min_distance_to_refine_search;

	bool is_aligned;

	char current_command;

	bool sirene_is_on;
	bool is_tracking;

	float distance_to_target;

	Robot robot;
	
	//Sonars
	SonarList sonar_list;
	XLMaxSonarEZ right_sonar;
	XLMaxSonarEZ left_sonar;
	XLMaxSonarEZ center_sonar;

	Log log;
	// Kalman kalman;
	// Radio radio;

	//Holds witch is the serial stream to receive
	//the commands
	Stream *command_stream;
	Stream *log_stream;
	Stream *encoder_stream;

	Locator locator;

	//Timers
	TimerForMethods<Locator> encoders_timer;
	TimerForMethods<Trekking> sirene_timer;
	TimerForMethods<Trekking> tracking_regulation_timer;

	void reset();
	void stop();
	void readInputs();
	void turnOnSirene();
	void turnOffSirene();

	void startTimers();
	void stopTimers();
	void resetTimers();
	void updateTimers();

	//Returns 1 if all the sensors are working
	bool checkSensors();

	//auxiliar functions for control
	Position plannedPosition(bool is_trajectory_linear, unsigned long t);
	void controlMotors(float v, float w);
	void trackTrajectory();

	/*----Operation modes----*/
	void standby();
	void search();
	void refinedSearch();
	void lighting();
	void (Trekking::*operation_mode)(void);

	void goToNextTarget();

	void debug();

public:
	Trekking(float max_linear_velocity, float max_angular_velocity);
	~Trekking();

	void addTarget(Position *target);

	void update();

	void start();
};

#endif //TREKKING_H