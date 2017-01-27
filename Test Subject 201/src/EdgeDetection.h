/*
 * EdgeDetection.h
 *
 *  Created on: Jan 12, 2017
 *      Author: Andy
 */

#ifndef SRC_EDGEDETECTION_H_
#define SRC_EDGEDETECTION_H_

class EdgeDetection {
public:
	EdgeDetection(bool inital=false);
	void update(bool state);

	bool isEdge();
	bool isRising();
	bool isFalling();
	bool getState();

private:
	bool lastState;
	bool thisState;
};

#endif /* SRC_EDGEDETECTION_H_ */
