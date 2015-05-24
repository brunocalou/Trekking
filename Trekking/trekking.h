#ifndef TREKKING_H
#define TREKKING_H

#include "../Log/log.h"
#include "../XLMaxSonarEZ/sonarlist.h"
#include "../Robot/robot.h"
#include "trekkingmath.h"
#include "position.h"

enum OperationMode {

	STANDBY,
	SEARCH_STAGE,
	REFINED_SEARCH_STAGE,
	LIGHTING
};

class Trekking {
private:

	float linear_velocity;
	float angular_velocidy;

	// Position* targets;
	LinkedList<Position *> targets;

	// Position* current_target;

	OperationMode operation_mode;

	const int COMMAND_BAUD_RATE;
	const int LOG_BAUD_RATE;
	const int ENCODER_BAUD_RATE;

	const byte MAX_MOTOR_PWM;
	const int LIGHT_DURATION;

	/*
		Input states
	*/

	bool init_button;
	bool emergency_button;
	bool operation_mode_switch;

	int min_distance_to_enable_lights;
	int min_distance_to_refine_search;

	bool is_aligned;

	Robot robot;
	
	//Sonars
	SonarList sonar_list;
	XLMaxSonarEZ right_sonar;
	XLMaxSonarEZ left_sonar;
	XLMaxSonarEZ center_sonar;

	Log log;
	// Kalman kalman;
	
	// Timer timer;
	// Locator locator;
	// Radio radio;

	//Holds witch is the serial stream to receive
	//the commands
	Stream *command_stream;
	Stream *log_stream;
	Stream *encoder_stream;
	

	void reset();
	void stop();
	void readInputs();
	void synalyze(bool light_enabled);
	void search();
	void refinedSearch();

	//Returns 1 if all the sensors are working
	bool checkSensors();

public:
	Trekking();
	~Trekking();

	void addTarget(Position *target);

	// void setSonarList(SonarList sonars);

	//void setRobot(Robot *robot);

	void update();
};

#endif //TREKKING_H