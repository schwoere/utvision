/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the 
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision
 * @file
 * TODO
 *
 * @author Frieder Pankratz
 */

#ifndef __UBITRACK_VISION_PIXELFLOWXY_H_INCLUDED__
#define __UBITRACK_VISION_PIXELFLOWXY_H_INCLUDED__

#include <utVision.h>

#include "AbstractPixelFlow.h"

namespace Ubitrack { namespace Vision {




class UTVISION_EXPORT PixelFlowXY :
	public AbstractPixelFlow
{
protected:
	unsigned int *sumx;
	unsigned int *sumy;
	unsigned int *sumx_old;
	unsigned int *sumy_old;


	int shiftX;
	int shiftY;
	unsigned int badnessX;
	unsigned int badnessY;

	unsigned int maximumBadnessX;
	unsigned int maximumBadnessY;
public:
	PixelFlowXY(int width, int height, int maxShift);
	~PixelFlowXY(void);

	void update(IplImage* image);
	void update(PixelFlowXY *other_flow);

	void updateWithoutFlow(IplImage* image);
	void updateWithoutFlow(PixelFlowXY *other_flow);
	void updateFlow();
	void updateFlow(const int startIndexX, const int startIndexY, const int maxShift);

	void getFlowXY(int &shiftX, int &shiftY, unsigned int &badnessX, unsigned int &badnessY);

	unsigned int getMaximumBadnessX();
	unsigned int getMaximumBadnessY();
};

}}

#endif