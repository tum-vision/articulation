/*
 * structure_learner.cpp
 *
 *  Created on: Jun 22, 2010
 *      Author: sturm
 */

#include "structure_learner_st.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "model_learner");

	KinematicStructureLearner learner;

	ros::NodeHandle nh;
	ros::Publisher lifebeat_pub = nh.advertise<std_msgs::Empty> ("/lifebeat", 1);
	while (nh.ok()) {
//		cout << "lifebeat.." << learner.intersectionOfStamps().size() << endl;
		ros::Rate loop_rate(3);
		loop_rate.sleep();
		ros::spinOnce();

		lifebeat_pub.publish(std_msgs::Empty());
	}
}
