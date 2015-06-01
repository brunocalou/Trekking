#include "position.h"
#include "Arduino.h"

//construtores
Position::Position(){
	Position:Position(0,0,0);
}

Position::Position(float x, float y, float theta){
	this->x = x;
	this->y = y;
	this->theta = theta;
}
	
//getters
float Position::getX(){
	return x;
}

float Position::getY(){
	return y;
}

float Position::getTheta(){
	return theta;
}

//setters
void Position::set(float x, float y, float theta){
	this->x = x;
	this->y = y;
	this->theta = theta;	
}


void Position::setX(float x){
	this->x = x;
}

void Position::setY(float y){
	this->y = y;
}

void Position::setTheta(float theta){
	this->theta = theta;
}

float Position::distanceFrom(Position *position) {
	return sqrt( sq((x - position->getX())) + sq(y - position->getY()) );
}
