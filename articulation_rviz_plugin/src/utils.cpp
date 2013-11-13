/*
 * utils.cpp
 *
 *  Created on: Oct 1, 2009
 *      Author: sturm
 */

#include "utils.h"

namespace articulation_rviz_plugin {

btVector3 HSV_to_RGB(btVector3 color) {
	double h = color.x(), s = color.y(), v = color.z();
	h = color.x();
	h -= floor(h);
	h *= 6;
	int i;
	float m, n, f;

	i = floor(h);
	f = h - i;
	if (!(i & 1))
		f = 1 - f; // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);
	switch (i) {
	case 6:
	case 0:
		return btVector3(v, n, m);
	case 1:
		return btVector3(n, v, m);
	case 2:
		return btVector3(m, v, n);
	case 3:
		return btVector3(m, n, v);
	case 4:
		return btVector3(n, m, v);
	case 5:
		return btVector3(v, m, n);
	default:
		return btVector3(1, 0.5, 0.5);
	}
}

btVector3 RGB_to_HSV( btVector3 color ) {
	double r = color.x(), g = color.y(), b = color.z();
	// RGB are from 0..1, H is from 0..360, SV from 0..1
	double maxC = b;
	if (maxC < g) maxC = g;
	if (maxC < r) maxC = r;
	double minC = b;
	if (minC > g) minC = g;
	if (minC > r) minC = r;

	double delta = maxC - minC;

	double V = maxC;
	double S = 0;
	double H = 0;

	if (delta == 0)
	{
		H = 0;
		S = 0;
	}
	else
	{
		S = delta / maxC;
		double dR = 60*(maxC - r)/delta + 180;
		double dG = 60*(maxC - g)/delta + 180;
		double dB = 60*(maxC - b)/delta + 180;
		if (r == maxC)
			H = dB - dG;
		else if (g == maxC)
			H = 120 + dR - dB;
		else
			H = 240 + dG - dR;
	}

	if (H<0)
		H+=360;
	if (H>=360)
		H-=360;

	return( btVector3(H/360.0, S, V) );
}

}
