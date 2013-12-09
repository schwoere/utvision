#ifndef __COLORS_UBITRACK_H_INCLUDED__
#define __COLORS_UBITRACK_H_INCLUDED__

#include <opencv2/core/core.hpp>
#include <utVision.h>

namespace Ubitrack { namespace Vision {

	/*
	Return a RGB colour value given a scalar v in the range [vmin,vmax]
	In this case each colour component ranges from 0 (no contribution) to
	1 (fully saturated), modifications for other ranges is trivial.
	The colour is clipped at the end of the scales if v is outside
	the range [vmin,vmax]
	*/

UTVISION_EXPORT CvScalar getGradientRampColor(double v,double vmin,double vmax);

}}

#endif