/*
 * EdgeDetection.cpp
 *
 *  Created on: Jan 12, 2017
 *      Author: Andy
 */

#include <EdgeDetection.h>

EdgeDetection::EdgeDetection(bool inital) :
lastState(inital),
thisState(inital)
{
}

void EdgeDetection::update(bool state)
{
	lastState=thisState;
	thisState = state;
}

bool EdgeDetection::isEdge()
{
	return thisState != lastState;
}

bool EdgeDetection::isRising()
{
	return (thisState == true && lastState == false);
}

bool EdgeDetection::isFalling()
{
	return (thisState == false && lastState == true);
}

bool EdgeDetection::getState()
{
	return thisState;
}



