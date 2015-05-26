#ifndef LOCATOR_H
#define LOCATOR_H

#include "Arduino.h"
#include "inttypes.h"

#include "../Timer/timer.h"
#include "../EncoderList/encoderlist.h"
#include "../Encoder/simpleencoder.h"

class Locator {
public:
	Locator(Stream *encoder_stream);
	~Locator();
	void update();
	void setPulsesPerRotation(int number_of_pulses);
	void start();
	void reset();

private:
	SimpleEncoder front_left_encoder;
	SimpleEncoder front_right_encoder;
	SimpleEncoder back_left_encoder;
	SimpleEncoder back_right_encoder;
	EncoderList encoder_list;
	// Stream *encoder_stream;
};

#endif// LOCATOR_H