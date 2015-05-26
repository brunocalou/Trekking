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

	targets(),
	encoder_stream(&Serial2),
	locator(encoder_stream),

	//Timers
	encoders_timer(&locator, &Locator::update),
	sirene_timer(this, &Trekking::goToNextTarget)
{
	//Streams
	command_stream = &Serial; //bluetooth on Serial1
	log_stream = &Serial;
	
	log.setTarget(log_stream);

	robot.setMinPWM(80, 80);

	//MUST CHECK THE RIGHT ORDER ON THE BOARD
	sonar_list.addSonar(&left_sonar);
	sonar_list.addSonar(&center_sonar);
	sonar_list.addSonar(&right_sonar);

	//Timers
	encoders_timer.setInterval(30);
	sirene_timer.setTimeout(LIGHT_DURATION);

	reset();
}

Trekking::~Trekking() {
	log.info("memory management", "freeing memory");

	for(int i = targets.size() - 1; i >= 0; i--) {
		Position *target = targets.remove(0);
		log.info("memory management", (int)target);
		log.info("target", "removed target");
		delete target;
	}
	log.info("memory management", "done");
}

void Trekking::addTarget(Position *target) {
	targets.add(target);
	log.info("target", "added target");
}

void Trekking::update() {
	//Read all the inputs
	readInputs();

	//Update all the timers
	updateTimers();

	current_command = ' ';

	//Read the serial
	if(command_stream->available()) {
		current_command = command_stream->read();
		log.debug("received command", current_command);

	} else {
		//Read the radio
	}

	//Stop the robot if the emergency button is pressed or
	//the stop command was received
	if(current_command == 'S' || emergency_button) {
		stop();
		operation_mode = &Trekking::standby;
		log.assert("operation mode", "standby");
		current_command = ' ';
	}

	//Debug
	debug();

	//Call the current operation
	(this->*operation_mode)();
}

void Trekking::reset() {
	log.assert("reset", "resetting...");

	//Trekking variables
	linear_velocity = 0;
	angular_velocidy = 0;
	min_distance_to_enable_lights = 0;
	min_distance_to_refine_search = 0;
	current_command = ' ';
	is_aligned = false;
	current_target_index = 0;
	
	//Go to the standby state
	operation_mode = &Trekking::standby;
	log.assert("operation mode", "standby");

	//Trekking objects
	locator.reset();
	robot.setPWM(MAX_MOTOR_PWM, MAX_MOTOR_PWM);
	robot.stop();
	// Kalman kalman;
	// Radio radio;

	//Trekking methods
	stopTimers();
	resetTimers();
	turnOffSirene();

	log.assert("reset", "done.");

	readInputs();
}

void Trekking::stop() {
	log.assert("stop", "stopping robot");
	robot.stop();
}

void Trekking::readInputs() {
	init_button = true;
	emergency_button = false;
	operation_mode_switch = MANUAL_MODE;
	// init_button = digitalRead(INIT_BUTTON_PIN);
	// emergency_button = digitalRead(EMERGENCY_BUTTON_PIN);
	// operation_mode_switch = digitalRead(OPERATION_MODE_SWITCH_PIN);
}

void Trekking::turnOnSirene() {
	digitalWrite(SIRENE_PIN, HIGH);
	sirene_is_on = true;
}

void Trekking::turnOffSirene() {
	digitalWrite(SIRENE_PIN, LOW);
	sirene_is_on = false;
}

bool Trekking::checkSensors() {
	return true;
}

void Trekking::standby() {
	if(operation_mode_switch == AUTO_MODE) {
		log.debug("mode switch", "auto");
		if(init_button) {
			log.debug("init button", init_button);
			reset();
			if(checkSensors()) {
				operation_mode = &Trekking::search;
				startTimers();
				log.assert("operation mode", "search stage");
			} else {
				log.error("sensors", "sensors not working as expected");
			}
		}
	} else if(current_command != ' ') {
		robot.useCommand(current_command);
		log.debug("using command", current_command);
	}
}

void Trekking::search() {

}

void Trekking::refinedSearch() {

}

void Trekking::lighting() {
	if(!sirene_is_on) {
		sirene_timer.start();
		turnOnSirene();
	}
}

void Trekking::goToNextTarget() {
	turnOffSirene();
	current_target_index ++;
	log.assert("current target", current_target_index);
	if(current_target_index >= targets.size()) {
		operation_mode = &Trekking::standby;
		log.assert("operation mode", "standby");
	} else {
		operation_mode = &Trekking::search;
		log.assert("operation mode", "search stage");
	}
}

void Trekking::startTimers() {
	encoders_timer.start();
	// sirene_timer.start();
}

void Trekking::stopTimers() {
	encoders_timer.stop();
	sirene_timer.stop();
}

void Trekking::resetTimers() {
	encoders_timer.reset();
	sirene_timer.reset();
}

void Trekking::updateTimers() {
	encoders_timer.update();
	sirene_timer.update();
}

void Trekking::start() {
	Serial.begin(COMMAND_BAUD_RATE);
	Serial1.begin(COMMAND_BAUD_RATE);
	Serial2.begin(ENCODER_BAUD_RATE);
}

void Trekking::debug() {
	if(current_command == 's') {
		log.debug("debug command", "set to standby");
		operation_mode = &Trekking::standby;
	} else if(current_command == 'e') {
		log.debug("debug command", "set to search");
		operation_mode = &Trekking::search;

	} else if(current_command == 'f') {
		log.debug("debug command", "set to refined search");
		operation_mode = &Trekking::refinedSearch;

	} else if(current_command == 'l') {
		log.debug("debug command", "set to lighting");
		operation_mode = &Trekking::lighting;

	} else if(current_command == 'r') {
		log.debug("debug command", "reset");
		reset();

	} else if(current_command == '1') {
		log.debug("debug command", "turn on sirene");
		turnOnSirene();

	} else if(current_command == '2') {
		log.debug("debug command", "turn off sirene");
		turnOffSirene();
	}
}