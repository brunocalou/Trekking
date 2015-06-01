#include "locator.h"

Locator::Locator(Stream *encoder_stream, Position initial_position):encoder_list(encoder_stream) {
		encoder_list.addEncoder(&front_left_encoder);
		encoder_list.addEncoder(&back_left_encoder);
		encoder_list.addEncoder(&front_right_encoder);
		encoder_list.addEncoder(&back_right_encoder);
		robot_linear_speed = 0;
		robot_angular_speed = 0;

		last_robot_linear_speed = 0;
		last_robot_angular_speed = 0;
		last_position = initial_position;
		last_update_time = 0;
}


Locator::~Locator() {}

void Locator::start() {
	encoder_list.start();
}

//getters
const SimpleEncoder& Locator::getBackLeftEncoder() const {
	return back_left_encoder;
}
const SimpleEncoder& Locator::getBackRightEncoder() const {
	return back_right_encoder;
}
const EncoderList& Locator::getEncoderList() const {
	return encoder_list;
}
const SimpleEncoder& Locator::getFrontLeftEncoder() const {
	return front_left_encoder;
}
const SimpleEncoder& Locator::getFrontRightEncoder() const {
	return front_right_encoder;
}
float Locator::getLastRobotAngularSpeed() const {
	return last_robot_angular_speed;
}
float Locator::getLastRobotLinearSpeed() const {
	return last_robot_linear_speed;
}
unsigned long Locator::getLastUpdateTime() const {
	return last_update_time;
}
float Locator::getRobotAngularSpeed() const {
	return robot_angular_speed;
}
float Locator::getRobotLinearSpeed() const {
	return robot_linear_speed;
}



void Locator::update() {
	//Get the current values
	encoder_list.read();
	unsigned long now = millis();
	long delta_t = now - last_update_time;
	int delta_pulses[4];
	float rps[4];
	for (int i=0; i<4; i++){
		delta_pulses[i] = encoder_list.get(i)->getDeltaPulses();
		//Calculate rps
		rps[i] = delta_pulses[i]/(delta_t * PULSES_PER_ROTATION);
	}
	
	//Calculate speed
	calculateSpeeds(rps);
	calcutePosition(delta_t); //Update position and velocity
	
	//Update values
	last_update_time = now;
}


void Locator::reset(Position new_position) {
	encoder_list.reset();

	robot_linear_speed = 0;
	robot_angular_speed = 0;

	last_robot_linear_speed = 0;
	last_robot_angular_speed = 0;
	last_position = new_position;
	last_update_time = 0;	
}


Position* Locator::getLastPosition(){
	return &last_position;
}


void Locator::calculateSpeeds(float rps[]){
	float left_speed = (rps[0] + rps[1])*WHEEL_RADIUS/2; // em [m/s]
	float right_speed = (rps[2] + rps[3])*WHEEL_RADIUS/2;// em [m/s]
	
	robot_linear_speed = (right_speed + left_speed)/2;
	robot_angular_speed = (right_speed - left_speed)/DISTANCE_FROM_RX;
}


void Locator::calcutePosition(float dT){
	float med_angular_speed = (robot_angular_speed + last_robot_angular_speed)/2;
	float theta = last_position.getTheta() + med_angular_speed*dT;

	float med_linear_speed = (robot_linear_speed + last_robot_linear_speed)/2;
	float x = last_position.getX() + med_linear_speed*cos(theta)*dT;//angulo em radiano
	float y = last_position.getY() + med_linear_speed*sin(theta)*dT;//angulo em radiano

	//Update values
	last_position.set(x, y, theta);
	last_robot_linear_speed = robot_linear_speed;
	last_robot_angular_speed = robot_angular_speed;
}
