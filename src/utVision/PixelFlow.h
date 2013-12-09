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
 * Pixel Flow routines.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 * @modified Artashes Mkhitaryan <artashm@gmail.com>
 */

#ifndef __UBITRACK_VISION_PIXELFLOW_H_INCLUDED__
#define __UBITRACK_VISION_PIXELFLOW_H_INCLUDED__

#include "Image.h"
#include <opencv/cv.h>


namespace Ubitrack { namespace Vision { 

/**
 * Computes the pixel flow of a block from one image to another
 */
class UTVISION_EXPORT PixelFlow
{
public:
	/**
	 * Empty Constructor
	 */
	PixelFlow();
	/**
	 * Destructor
	 */
	~PixelFlow();
	
	/** 
	 * compute the projection buffers from the reference image.
	 * @param image the reference image
	 * @param topLeft top-left corner of the area in the reference image for which to compute the pixel flow
	 * @param bottomRight bottom-right corner of the area in the reference image for which to compute the pixel flow
	 */
	void calcProjectionBuffer( const Image& image, const Math::Vector< int, 2 >& topLeft, const Math::Vector< int, 2 >& bottomRight );
	
	/** 
	 * Search for the most likely image shift in another image.
	 * @param image the  image in which to search.
	 * @param result the shift vector. (-2,1) means the area was moved two pixels left and one down
	 * @param difference the RMS error of the pixel difference (sum of x and y differences)
	 */
	void computeFlow( const Image& image, Math::Vector< int, 2 >& result, int& difference , Image* pDebugImg);
	

protected:	
	
	/** 
	 * compute the size of increased rectangles that are used to calculate new projection buffers
	 * @param width the width of the image
	 * @param height the heght of the image
	 * @param incr the amount on which the current size of projection buffer should be increased
	 * @param topL top left coordinat of the current rectangle
	 * @param bottomE bootom right coordinat of the current rectangle
	 * @param xRec new Increased in x direction rectangle
	 * @param yRec new Increased in y direction rectangle
	 */
	void calculateIncrisedCoordiants(int width, int height,double incr, std::vector<int> topL,
			std::vector<int> bottomR, Math::Vector< int, 4 >& xRec, Math::Vector< int, 4 >& yRec);

	/** 
	* Calcualte the error buffer for the current two vectores
	* @param v the projection buffer from the previous frame
	* @param newV the projection buffer from the current frame
	* @param ErrorBuf the refrence to the error buffer that has
	*		 been just calculated
	* @param nCut flag which tells whether the new buffer have 
	*         been cut, if 1 then the new buuer is cut from the 
	*		 right or bottom if -1 then from laft or top if 0 
	*		 then it has not been cut.
	*/
	void calculateErrorBuffer(const std::vector<int>& v,const std::vector<int>& newV, std::vector<int>& ErrorBuf, int nCut);
				

	/**the x pixel flow buffer*/
	std::vector<int> x;

	/**the y pixel flow buffer*/
	std::vector<int> y;
	

	/**the position of the frame*/
	std::vector<int> topL;
	std::vector<int> bottomR;

};

} } // namespace Ubitrack::Vision
#endif
