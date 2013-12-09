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
 * Implementation of routines to detect square markers in images.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#ifndef __UBITRACK_VISION_EDGEEXTRACTION_H_INCLUDED__
#define __UBITRACK_VISION_EDGEEXTRACTION_H_INCLUDED__

#include <limits.h>
#include <boost/scoped_array.hpp>
#include <utVision.h>
#include <limits.h>
#include "Image.h"

namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Vision {

/**
 * Sample a line centered in a given point with subpixel precision and apply a sobel filter.
 *
 * This function does not respect the origin flag and assumes a y-coordinate of 0 is top.
 *
 * @param src grey-scale image
 * @param pDst array of nExtension integers, where results of sobel filter application will be stored
 * @param center center-point of the line in the image
 * @param nExtension length of line in pixels
 * @param direction direction vector of the line (normalized)
 */
UTVISION_EXPORT void sobelLineSubPix( const Image& src, int* pDst, const Math::Vector< float, 2 >& center, 
	const Math::Vector< float, 2 >& direction, int nExtension );

/**
 * Sample a line centered in a given point with subpixel precision and apply compute the gradient.
 *
 * This function does not respect the origin flag and assumes a y-coordinate of 0 is top.
 *
 * @param pSrc grey-scale image
 * @param pDst array of nExtension integers, where results of sobel filter application will be stored
 * @param center center-point of the line in the image
 * @param nExtension length of line in pixels
 * @param direction direction vector of the line (normalized)
 */
UTVISION_EXPORT void simpleLineGradient( const Image& src, int* pDst, const Math::Vector< float, 2 >& center, 
	const Math::Vector< float, 2 >& direction, int nExtension );

/**
 * Policy class for findEdge: find the edge with the highest positive intensity.
 */
struct FindEdgePositiveMaximum
{
	int value;
	int index;
	
	FindEdgePositiveMaximum()
		: value( INT_MIN )
		, index( 0 )
	{}
	
	void compare( int i, int v )
	{ 
		if ( v > value ) 
		{ 
			index = i; 
			value = v; 
		} 
	}
};


/**
 * Policy class for findEdge: find the edge with the highest absolute intensity.
 */
struct FindEdgeAbsoluteMaximum
{
	int value;
	int index;
	
	FindEdgeAbsoluteMaximum()
		: value( 0 )
		, index( 0 )
	{}
	
	void compare( int i, int v )
	{ 
		if ( v > value ) 
		{ 
			index = i; 
			value = v; 
		} 
		else if ( -v > value )
		{
			index = i;
			value = -v;
		}
	}
};


/**
 * Policy class for findEdge: Extract a line from the image and apply the sobel operator.
 * Wraps sobelLineSubPix.
 */
struct ExtractLineSobel
{
	void sample( const Image& image, int* pDst, const Math::Vector< float, 2 >& center, 
		const Math::Vector< float, 2 >& direction, int distance )
	{ sobelLineSubPix( image, pDst, center, direction, distance * 2 ); }
};

 
/**
 * Policy class for findEdge: Extract a line from the image and compute the simple gradient.
 * Wraps simpleLineGradient.
 */
struct ExtractLineSimpleGradient
{
	void sample( const Image& image, int* pDst, const Math::Vector< float, 2 >& center, 
		const Math::Vector< float, 2 >& direction, int distance )
	{ simpleLineGradient( image, pDst, center, direction, distance * 2 ); }
};

 
/**
 * Finds an edge along a line.
 *
 * Note: integer coordinates lie at pixel centers.
 * This function does not respect the origin flag and assumes that a y-coordinate of 0 is on top.
 *
 * @param EdgeComparison class to compare edge intensities. Use e.g. \c FindEdgePositiveOnly
 * @param LineExtraction class to extract the line. Use e.g. \c ExtractLineSobel
 * @param T usually float or double
 *
 * @param image image to search for edge in
 * @param start point to start searching (in both directions)
 * @param dir search direction (length determines the )
 * @param maxDistance maximum distance to search (in pixels)
 * @param response will store the strength of the maximum edge
 * @return the distance of the pixel with maximum response from \c start. Positive values are in the direction of \c dir.
 */
template< class EdgeComparison, class LineExtraction, class T >
T findEdge( const Image& image, const Math::Vector< T, 2 >& start, const Math::Vector< T, 2 >& dir, int maxDistance, int& maxResponse )
{
	// extract the sampled line
	boost::scoped_array< int > pBuffer( new int[ 2 * maxDistance + 1 ] );
	LineExtraction().sample( image, pBuffer.get(), start, dir, maxDistance );
	
	// find maximal edge
	EdgeComparison max;
	for ( int i = 1; i < 2 * maxDistance; i++ )
		max.compare( i, pBuffer[ i ] );
		
	// subpixel localisation of maximum: fit 3 pixels to parabola and compute zero of derivation
	T fMaxSubPix = static_cast< T >( max.index - maxDistance );
	T a = 0.5f * ( pBuffer[ max.index - 1 ] + pBuffer[ max.index + 1 ] ) - pBuffer[ max.index ];
	T b = 0.5f * ( pBuffer[ max.index + 1 ] - pBuffer[ max.index - 1 ] );
	if ( ( a != 0 ) && (fabsf( b ) <= fabsf( a * 10 ) ))
		fMaxSubPix -= b / ( 2.0f * a );

	maxResponse = max.value;
	return fMaxSubPix;
}


/**
 * Samples a point with subpixel precision and without outlier checks.
 *
 * @param pSrc image to sample
 * @param point to sample
 */
UTVISION_EXPORT int subpixSampleFast( const Image& src, const Math::Vector< float, 2 >& p );
	
/**
 * Samples a point with subpixel precision and outlier checks.
 *
 * @param pSrc image to sample
 * @param point to sample
 */
UTVISION_EXPORT int subpixSampleSafe( const Image& src, const Math::Vector< float, 2 >& p );

} } // namespace Ubitrack::Vision

#endif
