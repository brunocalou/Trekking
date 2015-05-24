#ifndef POSITION_H
#define POSITION_H

class Position{
private:
	int x;
	int y;
	int theta;

public:
	//construtores
	Position();
	Position(int x, int y, int theta);

	//getters
	int getX();
	int getY();
	int getTheta();

	//setters
	void setX(int x);
	void setY(int y);
	void setTheta(int theta);	
};

#endif