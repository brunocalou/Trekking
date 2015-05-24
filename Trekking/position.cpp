#include "position.h"

//construtores
Position::Position(){
	Position:Position(0,0,0);
}

Position::Position(int x, int y, int theta){
	this->x = x;
	this->y = y;
	this->theta = theta;
}
	
//getters
int Position::getX(){
	return x;
}

int Position::getY(){
	return y;
}

int Position::getTheta(){
	return theta;
}

//setters
void Position::setX(int x){
	this->x = x;
}

void Position::setY(int y){
	this->y = y;
}

void Position::setTheta(int theta){
	this->theta = theta;
}