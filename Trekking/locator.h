#ifndef LOCATOR_H
#define LOCATOR_H

#include "Arduino.h"
#include "inttypes.h"

#include "../Timer/timer.h"
#include "../EncoderList/encoderlist.h"
#include "../Encoder/simpleencoder.h"
#include "position.h"

const float WHEEL_RADIUS = 0.075;
const float PULSES_PER_ROTATION = 64.0;
const float DISTANCE_FROM_RX = 0.138;

#ifndef PI
#define PI 3.141592653589793238;
#endif

class Locator {
public:
	Locator(Stream *encoder_stream, Position initial_position);
	~Locator();
	void start();
	void update();
	Position* getLastPosition();
	void reset(Position new_position);
	EncoderList encoder_list;

public:
	SimpleEncoder front_left_encoder;
	SimpleEncoder front_right_encoder;
	SimpleEncoder back_left_encoder;
	SimpleEncoder back_right_encoder;

	
	unsigned long last_update_time;
	float robot_linear_speed;
	float robot_angular_speed;
	// Stream *encoder_stream;


	float last_robot_linear_speed;
	float last_robot_angular_speed;
	Position last_position;

	
	void calculateSpeeds(float pps[]);
	void calcutePosition(float deta_t);
	
};

#endif
