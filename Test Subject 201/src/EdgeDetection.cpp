/*
 * EdgeDetection.cpp
 *
 *  Created on: Jan 12, 2017
 *      Author: Andy
 */

#include <EdgeDetection.h>

Edge::Edge(bool inital) :
lastTime(inital),
thisTime(inital)
{
}

void Edge::update(bool state)
{
	lastTime=thisTime;
	thisTime = state;
}

bool Edge::isPressed()
{
	return (thisTime == true && lastTime == false);

}

bool Edge::getState(){

	return thisTime;

}



