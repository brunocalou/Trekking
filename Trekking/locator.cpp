#include "locator.h"

Locator::Locator(Stream *encoder_stream):
	encoder_list(encoder_stream) {
		encoder_list.addEncoder(&front_left_encoder);
		encoder_list.addEncoder(&front_right_encoder);
		encoder_list.addEncoder(&back_left_encoder);
		encoder_list.addEncoder(&back_right_encoder);
}

Locator::~Locator() {

}

void Locator::update() {
	encoder_list.read();
}

void Locator::setPulsesPerRotation(int number_of_pulses) {
	for(int i = 0; i < encoder_list.size(); i++) {
		encoder_list.get(i)->setPulsesPerRotation(number_of_pulses);
	}
}

void Locator::start() {
	encoder_list.start();
}

void Locator::reset() {
	encoder_list.reset();
}