/*
 * EdgeDetection.h
 *
 *  Created on: Jan 12, 2017
 *      Author: Andy
 */

#ifndef SRC_EDGEDETECTION_H_
#define SRC_EDGEDETECTION_H_

class Edge {
public:
	Edge(bool inital=false);
	void update(bool state);

	bool isPressed();
	bool getState();

private:
	bool lastTime;
	bool thisTime;
};

#endif /* SRC_EDGEDETECTION_H_ */
