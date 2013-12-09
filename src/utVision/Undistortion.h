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
 * A class for encapsulating the image undistortion logic
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#ifndef __UBITRACK_VISION_UNDISTORTION_H_INCLUDED__
#define __UBITRACK_VISION_UNDISTORTION_H_INCLUDED__

#include <boost/scoped_ptr.hpp>
#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include "Image.h"

namespace Ubitrack { namespace Vision {

class UTVISION_EXPORT Undistortion
{
public:
	
	/**
	 * Initialize from two filenames.
	 */
	Undistortion( const std::string& intrinsicMatrixFile, const std::string& distortionFile );
	
	/**
	 * Initialize from matrix+vector.
	 */
	Undistortion( const Math::Matrix< double, 3, 3 >& intrinsicMatrix, const Math::Vector< double, 8 >& distortion );
	
	/** 
	 * Undistorts an image.
	 */
	boost::shared_ptr< Image > undistort( const Image& image );

	/** 
	 * Undistorts an image.
	 * Returns the original pointer if no distortion.
	 */
	boost::shared_ptr< Image > undistort( boost::shared_ptr< Image > pImage );

	/** returns the intrinsic matrix */
	const Math::Matrix< double, 3, 3 >& getIntrinsics() const
	{ return m_intrinsics; }
	
	/** returns the radial distortion coefficients */
	const Math::Vector< double, 8 >& getRadialCoeffs() const
	{ return m_coeffs; }
	
	/** is distortion active? */
	bool hasDistortion() const
	{ return m_coeffs( 0 ) != 0.0; }

	/** returns the x-coordinate map */
	const Image& getMapX() const
	{ return *m_pMapX; }
	
	/** returns the y-coordinate map */
	const Image& getMapY() const
	{ return *m_pMapY; }

	/** 
	 * initialize the undistortion map. 
	 * Usually is called implicitly by undistort(), only necessary if getMapX/Y() ist used.
	 */
	void initMap( int width, int height, int origin );
	
protected:

	/** initialize parameters from two filenames */
	void initParams( const std::string& intrinsicMatrixFile, const std::string& distortionFile );

	/** initialize from matrix+vector */
	void initParams( const Math::Matrix< double, 3, 3 >& intrinsicMatrix, const Math::Vector< double, 8 >& distortion );

	// distortion parameters
	Math::Vector< double, 8 > m_coeffs;
	Math::Matrix< double, 3, 3 > m_intrinsics;
	
	// undistortion maps
	boost::scoped_ptr< Image > m_pMapX;
	boost::scoped_ptr< Image > m_pMapY;
};

} } // namespace Ubitrack::Vision

#endif
