#include "locator.h"

Locator::Locator(Stream *encoder_stream, Position initial_position):encoder_list(encoder_stream) {
		encoder_list.addEncoder(&front_left_encoder);
		encoder_list.addEncoder(&back_left_encoder);
		encoder_list.addEncoder(&front_right_encoder);
		encoder_list.addEncoder(&back_right_encoder);
		robot_linar_speed = 0;
		robot_angular_speed = 0;

		last_robot_linar_speed = 0;
		last_robot_angular_speed = 0;
		last_position = initial_position;
		last_update_time = 0;
}


Locator::~Locator() {}

void Locator::start() {
	encoder_list.start();
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
	
	//Updat values
	last_update_time = now;
}


void Locator::reset(Position new_position) {
	encoder_list.reset();

	robot_linar_speed = 0;
	robot_angular_speed = 0;

	last_robot_linar_speed = 0;
	last_robot_angular_speed = 0;
	last_position = new_position;
	last_update_time = 0;	
}


Position Locator::getLastPosition(){
	return last_position;
}


void Locator::calculateSpeeds(float rps[]){
	float left_speed = (rps[0] + rps[1])*WHEEL_RADIUS/2; // em [m/s]
	float right_speed = (rps[2] + rps[3])*WHEEL_RADIUS/2;// em [m/s]
	
	robot_linar_speed = (right_speed + left_speed)/2;
	robot_angular_speed = (right_speed - left_speed)/DISTANCE_FROM_RX;
}


void Locator::calcutePosition(float dT){
	float med_angular_speed = (robot_angular_speed + last_robot_angular_speed)/2;
	float theta = last_position.getTheta() + med_angular_speed*dT;

	float med_linar_speed = (robot_linar_speed + last_robot_linar_speed)/2;
	float x = last_position.getX() + med_linar_speed*cos(theta)*dT;//angulo em radiano
	float y = last_position.getY() + med_linar_speed*sin(theta)*dT;//angulo em radiano

	//Update values
	last_position.set(x, y, theta);
	last_robot_linar_speed = robot_linar_speed;
	last_robot_angular_speed = robot_angular_speed;
}
