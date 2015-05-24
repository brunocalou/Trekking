#include "trekking.h"
#include "trekkingpins.h"

Trekking::Trekking():

	//Motors
	MAX_MOTOR_PWM(255),
	robot(R_ENABLE_PIN,R_MOTOR_1_PIN,R_MOTOR_2_PIN,L_ENABLE_PIN,L_MOTOR_1_PIN,L_MOTOR_2_PIN),

	COMMAND_BAUD_RATE(9600),
	LOG_BAUD_RATE(9600),
	ENCODER_BAUD_RATE(57600),

	LIGHT_DURATION(3000),

	right_sonar(RIGHT_SONAR_TX_PIN, RIGHT_SONAR_RX_PIN),
	left_sonar(LEFT_SONAR_TX_PIN, LEFT_SONAR_RX_PIN),
	center_sonar(CENTER_SONAR_TX_PIN, CENTER_SONAR_RX_PIN),

	sonar_list(Sonar::CHAIN),

	targets()
{
	Serial.begin(COMMAND_BAUD_RATE);
	Serial1.begin(COMMAND_BAUD_RATE);
	Serial2.begin(ENCODER_BAUD_RATE);

	command_stream = &Serial;
	log_stream = &Serial;
	encoder_stream = &Serial;

	log.setTarget(log_stream);

	robot.setMinPWM(80, 80);

	//MUST CHECK THE RIGHT ORDER ON THE BOARD
	sonar_list.addSonar(&left_sonar);
	sonar_list.addSonar(&center_sonar);
	sonar_list.addSonar(&right_sonar);


	// targets = NULL;
	// current_target = NULL;

	reset();
}

Trekking::~Trekking() {
	log.info("memory management", "freeing memory");

	for(int i = targets.size() - 1; i >= 0; i--) {
		Position *target = targets.remove(0);
		log.info("memory management", (int)target);
		delete target;
	}
	log.info("memory management", "done");
}

void Trekking::addTarget(Position *target)
{
	log.info("target", "adding target");
	targets.add(target);
	log.info("target", "done");
}

// void Trekking::setSonarList(SonarList sonars)
// {
// }

// void Trekking::setRobot(Robot *robot)
// {
// 	this->robot = robot;
// }

void Trekking::update()
{
	//Read all the inputs
	readInputs();

	char command = ' ';

	//Read the serial
	if(command_stream->available()) {
		command = command_stream->read();
		log.debug("received command", command);

	} else {
		//Read the radio
	}

	//Stop the robot if the emergency button is pressed or
	//the stop command was received
	if(command == 'S' || emergency_button) {
		robot.stop();
		operation_mode = STANDBY;
		log.assert("stop", "stopping robot");
		log.assert("operation mode", "standby");
	}

	//Check the operation mode
	if(operation_mode == STANDBY) {
		if(operation_mode_switch == AUTO_MODE) {
			if(init_button) {
				reset();
				if(checkSensors()) {
					operation_mode = SEARCH_STAGE;
					log.assert("operation mode", "search stage");
				} else {
					log.error("sensors", "sensors not working as expected");
				}
			}
		} else {
			robot.useCommand(command);
			log.debug("using command", command);
		}

	} else if(operation_mode == SEARCH_STAGE) {

	} else if(operation_mode == REFINED_SEARCH_STAGE) {

	} else if(operation_mode == LIGHTING) {

	}
}

void Trekking::reset()
{
	log.assert("reset", "resetting...");
	linear_velocity = 0;
	angular_velocidy = 0;

	operation_mode = STANDBY;

	min_distance_to_enable_lights = 0;
	min_distance_to_refine_search = 0;

	is_aligned = false;

	// Kalman kalman;
	
	// Timer timer;
	// Locator locator;
	// Radio radio;
	
	robot.setPWM(MAX_MOTOR_PWM, MAX_MOTOR_PWM);
	robot.stop();

	log.assert("reset", "done.");

	readInputs();
}

void Trekking::stop()
{
	robot.stop();
}

void Trekking::readInputs()
{
	init_button = digitalRead(INIT_BUTTON_PIN);
	emergency_button = digitalRead(EMERGENCY_BUTTON_PIN);
	operation_mode_switch = digitalRead(OPERATION_MODE_SWITCH_PIN);
}

void Trekking::synalyze(bool light_enabled)
{
}

void Trekking::search()
{
}

void Trekking::refinedSearch()
{
}

bool Trekking::checkSensors() {
	return true;
}