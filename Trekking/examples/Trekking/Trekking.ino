#include "trekking.h"
#include "trekkingmath.h"
#include "sonarlist.h"
#include "encoderlist.h"
#include "timer.h"
#include "position.h"
// #include "XLMaxSonarEZ.h"
#include "log.h"

#include "robot.h"
#include "autocontrolboard.h"
#include "sensor.h"
#include "sensorboard.h"
#include "ultrasonic.h"
#include "simpleencoder.h"
#include "lightsensor.h"

Trekking trekking;
Position *cone_1 = new Position(0,0,0);
Position *cone_2 = new Position(0,0,0);
Position *cone_3 = new Position(0,0,0);

void setup() {
	trekking.start();
	trekking.addTarget(cone_1);
	trekking.addTarget(cone_2);
	trekking.addTarget(cone_3);
}

void loop() {
	trekking.update();
}