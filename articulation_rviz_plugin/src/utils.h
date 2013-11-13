/*
 * utils.h
 *
 *  Created on: Oct 1, 2009
 *      Author: sturm
 */

#ifndef ARTICULATION_RVIZ_PLUGIN_UTILS_H_
#define ARTICULATION_RVIZ_PLUGIN_UTILS_H_

#include "LinearMath/btVector3.h"

namespace articulation_rviz_plugin {

btVector3 HSV_to_RGB(btVector3 color);

btVector3 RGB_to_HSV(btVector3 color);

}

#endif /* UTILS_H_ */
