#include "Colors.h"
#include <opencv2/core/core_c.h>

namespace Ubitrack { namespace Vision {

CvScalar getGradientRampColor(double v,double vmin,double vmax)
{
	double c[3] = {1.0,1.0,1.0}; // white
	double dv;

	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;

	if (v < (vmin + 0.25 * dv)) {
		c[0] = 0;
		c[1] = 4 * (v - vmin) / dv;
	} else if (v < (vmin + 0.5 * dv)) {
		c[0] = 0;
		c[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
	} else if (v < (vmin + 0.75 * dv)) {
		c[0] = 4 * (v - vmin - 0.5 * dv) / dv;
		c[2] = 0;
	} else {
		c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
		c[2] = 0;
	}

	return CV_RGB(255 * c[0], 255 * c[1], 255 * c[2]);
}

}}