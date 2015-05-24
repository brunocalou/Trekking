#ifndef ENCODERLIST_H
#define ENCODERLIST_H

//#include "Stream.h"
#include "/usr/share/arduino/hardware/arduino/cores/arduino/Stream.h"
#include <inttypes.h>
#include <Arduino.h>

/*
	To get the linked list class, if not available, check https://github.com/ivanseidel/LinkedList
*/
#ifndef LinkedList_h
// #include "../LinkedList/LinkedList.h"
#include "/usr/share/arduino/libraries/LinkedList/LinkedList.h"
#endif

//#include "../Encoder/simpeencoder.h"
#include "/usr/share/arduino/libraries/Encoder/simpleencoder.h"

#define ACKNOWLEDGEMENT 'a'
#define TOKEN ','
#define RESET 'r'

class EncoderList: public LinkedList<SimpleEncoder*> {
public:
	EncoderList(Stream *serial);
	~EncoderList();

	/*
		Adds an encoder to the list
	*/
	void addEncoder(SimpleEncoder * encoder);

	/*
		Reads the encoders on the serial stream and
		sends an acknowledgement character.
	*/
	bool read();

	/*
		Prints all the encoders with the specified format
	*/
	void print();

	/*
		Resets all the encoders
	*/
	void reset();

	/*
		Start the communication
	*/
	void start();

private:
	Stream *serial;

};

#endif //ENCODERLIST_H