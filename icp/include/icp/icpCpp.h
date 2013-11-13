/*
 * icpCpp.h
 *
 *  Created on: Feb 15, 2010
 *      Author: sturm
 */

#ifndef ICPCPP_H_
#define ICPCPP_H_

#include "kdtree_common.h"

namespace icp {

void icp(
double *trpr,
double *ttpr,
double *modelz,
unsigned int nmodelz,
double *dataz,
double *qltyz,
unsigned int ndataz,
unsigned int *randvecz,
unsigned int nrandvecz,
unsigned int nrandz,
unsigned int iimax,
icp::Tree *pointer_to_tree
);

}

#endif /* ICPCPP_H_ */
