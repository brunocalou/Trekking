#include "trekking.h"
#include "trekkingpins.h"

Trekking::Trekking(float linear_velocity, float angular_velocity):
	//Velocities
	MAX_LINEAR_VELOCITY(linear_velocity),
	MAX_ANGULAR_VELOCITY(angular_velocity),
	
	//Motors
	MAX_MOTOR_PWM(255),
	MAX_RPS(3000),
	robot(R_ENABLE_PIN,R_MOTOR_1_PIN,R_MOTOR_2_PIN,L_ENABLE_PIN,L_MOTOR_1_PIN,L_MOTOR_2_PIN),

	COMMAND_BAUD_RATE(9600),
	LOG_BAUD_RATE(9600),
	ENCODER_BAUD_RATE(57600),

	LIGHT_DURATION(3000),
	PROXIMITY_RADIUS(3.0),

	READ_ENCODERS_TIME(30),
	READ_MPU_TIME(30),
	
	right_sonar(RIGHT_SONAR_TX_PIN, RIGHT_SONAR_RX_PIN),
	left_sonar(LEFT_SONAR_TX_PIN, LEFT_SONAR_RX_PIN),
	center_sonar(CENTER_SONAR_TX_PIN, CENTER_SONAR_RX_PIN),

	sonar_list(Sonar::CHAIN),

	targets(),
	init_position(),
	encoder_stream(&Serial2),
	locator(encoder_stream, Position(0,0,0)),

	//Timers
	encoders_timer(&locator, &Locator::update),
	sirene_timer(this, &Trekking::goToNextTarget),
	tracking_regulation_timer(this, &Trekking::trackTrajectory),
	mpu_timer(&locator, &Locator::readMPU)
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
	mpu_timer.setInterval(READ_MPU_TIME);
	mpu_timer.start(); //Must read the mpu all the time

	encoders_timer.setInterval(READ_ENCODERS_TIME);
	sirene_timer.setTimeout(LIGHT_DURATION);
	tracking_regulation_timer.setTimeout(READ_ENCODERS_TIME);

	kp = 7.2;
	ki = 0;
	kd = 8;
	bsp = 0.8;
	pid_convertion_const = (1000 * 2 * 3.1415) / 1024.0;

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
	if(current_command == 'D' || emergency_button) {
		stop();
		stopTimers();
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
	desired_linear_velocity = 0;
	desired_angular_velocity = 0;
	min_distance_to_enable_lights = 0;
	min_distance_to_refine_search = 0;
	current_command = ' ';
	is_aligned = false;
	is_tracking = false;
	current_target_index = 0;
	distance_to_target = locator.getLastPosition()->distanceFrom(targets.get(current_target_index));
	left_vel_ref = 0;
	right_vel_ref = 0;
	
	//Go to the standby state
	operation_mode = &Trekking::standby;
	log.assert("operation mode", "standby");

	//Trekking objects
	Position initial_position(0,0,0);
	locator.reset(initial_position);
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

//TODO: acho que nao vai rolar fazer essa...
void Trekking::readInputs() {
	// init_button = true;
	// emergency_button = false;
	// operation_mode_switch = MANUAL_MODE;
	init_button = digitalRead(INIT_BUTTON_PIN);
	emergency_button = digitalRead(EMERGENCY_BUTTON_PIN);
	operation_mode_switch = digitalRead(OPERATION_MODE_SWITCH_PIN);
}

void Trekking::turnOnSirene() {
	digitalWrite(SIRENE_PIN, HIGH);
	sirene_is_on = true;
}

void Trekking::turnOffSirene() {
	digitalWrite(SIRENE_PIN, LOW);
	sirene_is_on = false;
}

//TODO: vai ser necessario mesmo?
bool Trekking::checkSensors() {
	return true;
}

void Trekking::standby() {
	if(operation_mode_switch == AUTO_MODE) {
		// log.debug("mode switch", "auto");
		if(init_button) {
			// log.debug("init button", init_button);
			reset();
			if(checkSensors()) {
				operation_mode = &Trekking::search;
				startTimers();
				locator.start();
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

void Trekking::trackTrajectory() {
	bool linear = true;
	if(linear){
		desired_linear_velocity = MAX_LINEAR_VELOCITY;
		desired_angular_velocity = 0;
	}else{
		desired_linear_velocity = 0;
		desired_angular_velocity = MAX_ANGULAR_VELOCITY;
	}

	//constants to handle errors
	const int k1 = 2;
	const int k2 = 1;
	const int k3 = 2;

	Position* trekking_position = locator.getLastPosition();
	unsigned long t = tracking_regulation_timer.getElapsedTime();
	Position gap = trekking_position->calculateGap(plannedPosition(linear, t));

	//Angular transformation
	float e1 = gap.getX()*cos(trekking_position->getTheta()) + gap.getY()*sin(trekking_position->getTheta());
	float e2 = -gap.getX()*sin(trekking_position->getTheta()) + gap.getY()*cos(trekking_position->getTheta());
	float e3 = gap.getTheta();

	//Calculating linear velocity and angular velocity
	float v = desired_linear_velocity*cos(e3) + k1*e1;
	float w = desired_angular_velocity + k2*e2 + k3*e3;

	controlMotors(v, w);


	distance_to_target = trekking_position->distanceFrom(targets.get(current_target_index));
}

void Trekking::controlMotors(float v, float w){
	//Calculating rps
	float right_wheels_vel = (2*v + w*DISTANCE_FROM_RX)/(2*WHEEL_RADIUS);//[RPS]
	float left_wheels_vel = (2*v - w*DISTANCE_FROM_RX)/(2*WHEEL_RADIUS);//[RPS]

	//calculating rpm
	byte right_pwm = right_wheels_vel*MAX_MOTOR_PWM/MAX_RPS;
	bool right_reverse = (right_wheels_vel < 0);
	byte left_pwm = left_wheels_vel*MAX_MOTOR_PWM/MAX_RPS;
	bool left_reverse = (left_wheels_vel < 0);

	//PID

	// Setting PWM
	robot.setRPWM(right_pwm, right_reverse);
	robot.setLPWM(left_pwm, left_reverse);
}

Position Trekking::plannedPosition(bool is_trajectory_linear, unsigned long tempo){

	Position* destination = targets.get(current_target_index);
	Position* trekking_position = locator.getLastPosition();
	Position planned_position = Position();
	float t = (float) tempo;
	t /= 1000; //Mills to sec
	if(is_trajectory_linear){
		float velocity = desired_linear_velocity;
		float dirx = 1;
		if (init_position.getX() > destination->getY()){
			dirx = -1;
		}
		float diry = 1;
		if (init_position.getY() > destination->getY()){
			diry = -1;
		}
		float path_t = 	velocity*t;
		planned_position.set(init_position.getX() + path_t*cos(init_position.getTheta())*dirx, //x
							init_position.getY() + path_t*sin(init_position.getTheta())*diry, //y
							init_position.getTheta());
	}
	else{
		float velocity = desired_angular_velocity;
		planned_position.set(init_position.getX(),
							 init_position.getY(),
							 trekking_position->getTheta() +  velocity*t);
	}
}

void Trekking::search() {
	if(!is_tracking) {
		tracking_regulation_timer.start();
		is_tracking = true;
	}
	//Colocar a condicao de proximidade
	if(distance_to_target < PROXIMITY_RADIUS) {
		tracking_regulation_timer.stop();
		tracking_regulation_timer.reset();
		operation_mode = &Trekking::refinedSearch;
	}
}

void Trekking::refinedSearch() {
	sonar_list.read();
	// center_sonar.getDistance();
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
		//AQUIIIIIII
	}
}

void Trekking::startTimers() {
	encoders_timer.start();
	// sirene_timer.start();
}

void Trekking::stopTimers() {
	encoders_timer.stop();
	sirene_timer.stop();
	tracking_regulation_timer.stop();
}

void Trekking::resetTimers() {
	encoders_timer.reset();
	sirene_timer.reset();
	tracking_regulation_timer.reset();
}

void Trekking::updateTimers() {
	mpu_timer.update();
	encoders_timer.update();
	sirene_timer.update();
	tracking_regulation_timer.update();
}

void Trekking::start() {
	//Turn off the alert led
	digitalWrite(ALERT_LED, LOW);
	locator.initMPU();
	Serial.begin(COMMAND_BAUD_RATE);
	Serial1.begin(COMMAND_BAUD_RATE);
	// Serial2.begin(ENCODER_BAUD_RATE);
}

void Trekking::debug() {
	if(current_command == 's') {
		log.debug("debug command", "set to standby");
		operation_mode = &Trekking::standby;
		stopTimers();
	} else if(current_command == 'e') {
		log.debug("debug command", "set to search");
		// operation_mode = &Trekking::search;
		operation_mode_switch = AUTO_MODE;
		init_button = true;
		standby();
	} else if(current_command == 'f') {
		log.debug("debug command", "set to refined search");
		operation_mode = &Trekking::refinedSearch;

	} else if(current_command == 'l') {
		log.debug("debug command", "set to lighting");
		operation_mode = &Trekking::lighting;

	} else if(current_command == 'r') {
		log.debug("debug command", "reset");
		reset();

	} else if(current_command == 'W') {
		log.debug("debug command", "turn on sirene");
		turnOnSirene();

	} else if(current_command == 'w') {
		log.debug("debug command", "turn off sirene");
		turnOffSirene();
	} else if(current_command == 'n') {
		log.debug("debug command", "print encoders");
		for(int i = 0; i < locator.encoder_list.size(); i++) {
			Serial.print(locator.encoder_list.get(i)->getPulses());
			Serial.print("\t");
		}
		Serial.println();
	} else if(current_command == 'L') {
		log.debug("debug command", "print locator");
		log << DEBUG << "angular\tlinear\ttime" << log_endl;
		
		log << locator.getRobotAngularSpeed() << '\t';
		log << locator.getRobotLinearSpeed() << '\t';
		log << locator.getLastUpdateTime() << log_endl;
	} else if(current_command == 'm') {
		log.debug("debug command", "print mpu");
		log << DEBUG << "x\ty\tz" << log_endl;
		
		log << locator.euler_degrees[0] << '\t';
		log << locator.euler_degrees[1] << '\t';
		log << locator.euler_degrees[2] << log_endl;
	} else if(current_command == 'o') {
		log.debug("debug command", "print sonars");
		log << DEBUG << "left\tcenter\tright" << log_endl;
		sonar_list.read();
		log << left_sonar.getDistance() << '\t';
		log << center_sonar.getDistance() << '\t';
		log << right_sonar.getDistance() << log_endl;
	} else if(current_command == 'b') {
		log.debug("debug command", "print buttons");
		log << DEBUG << "init\temergency\tmode" << log_endl;
		
		log << digitalRead(INIT_BUTTON_PIN) << '\t';
		log << digitalRead(EMERGENCY_BUTTON_PIN) << '\t';
		log << digitalRead(OPERATION_MODE_SWITCH_PIN) << log_endl;
	}
}
