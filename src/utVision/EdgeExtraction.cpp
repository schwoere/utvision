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

#include <math.h>
#include <boost/scoped_array.hpp>

#include "utVision.h"
#include "Image.h"
#include "EdgeExtraction.h"

namespace ublas = boost::numeric::ublas;

namespace Ubitrack { namespace Vision { 

int subpixSampleFast( const Image& src, const Math::Vector< float, 2 >& p )
{
	int x = static_cast< int >( floorf( p( 0 ) ) );
	int y = static_cast< int >( floorf( p( 1 ) ) );
	int dx = static_cast< int >( 256 * ( p( 0 ) - floorf( p( 0 ) ) ) );
	int dy = static_cast< int >( 256 * ( p( 1 ) - floorf( p( 1 ) ) ) );
	unsigned char* i = reinterpret_cast< unsigned char* >( src.imageData ) + y * src.widthStep + x;
	int a = i[ 0 ] + ( ( dx * ( i[ 1 ] - i[ 0 ] ) ) >> 8 );
	i += src.widthStep;
	int b = i[ 0 ] + ( ( dx * ( i[ 1 ] - i[ 0 ] ) ) >> 8) ;
	return a + ( ( dy * ( b - a ) ) >> 8 );
}


int subpixSampleSafe( const Image& src, const Math::Vector< float, 2 >& p )
{
	int x = static_cast< int >( floorf( p( 0 ) ) );
	int y = static_cast< int >( floorf( p( 1 ) ) );

	if ( x < 0 || x >= src.width - 1 || y < 0 || y >= src.height - 1 )
	{
		// continue border pixels
		if ( x < 0 ) 
			x = 0;
		else if ( x >= src.width )
			x = src.width - 1;
			
		if ( y < 0 )
			y = 0;
		else if ( y >= src.height )
			y = src.height - 1;
			
		return *( reinterpret_cast< unsigned char* >( src.imageData ) + y * src.widthStep + x );
	}
	
	// do normal sampling
	int dx = static_cast< int >( 256 * ( p( 0 ) - floorf( p( 0 ) ) ) );
	int dy = static_cast< int >( 256 * ( p( 1 ) - floorf( p( 1 ) ) ) );
	unsigned char* i = reinterpret_cast< unsigned char* >( src.imageData ) + y * src.widthStep + x;
	int a = i[ 0 ] + ( ( dx * ( i[ 1 ] - i[ 0 ] ) ) >> 8 );
	i += src.widthStep;
	int b = i[ 0 ] + ( ( dx * ( i[ 1 ] - i[ 0 ] ) ) >> 8 );
	return a + ( ( dy * ( b - a ) ) >> 8 );
}


/**
 * \internal
 * Sample a line centered in a given point with subpixel precision and apply a sobel filter.
 *
 * This function does not respect the origin flag and assumes a y-coordinate of 0 is top.
 *
 * @param pSrc grey-scale image
 * @param pDst array of nExtension integers, where results of sobel filter application will be stored
 * @param center center-point of the line in the image
 * @param nExtension length of line in pixels
 * @param direction direction vector of the line (normalized)
 */
void sobelLineSubPix( const Image& src, int* pDst, const Math::Vector< float, 2 >& center, 
	const Math::Vector< float, 2 >& direction, int nExtension )
{
	Math::Vector< float, 2 > n, p;
	n( 0 ) = direction( 1 );
	n( 1 ) = -direction( 0 );

	// check if one corner is outside the image
	int t;
	for ( t = 0; t < 4; t++ )
	{
		p = center + ( (t&1) ? -1 : 1 ) * ( ( nExtension >> 1 ) + 1 ) * direction + ( (t&2) ? -1 : 1 ) * n;
		int x = static_cast< int >( floorf( p( 0 ) ) );
		int y = static_cast< int >( floorf( p( 1 ) ) );
		if ( x < 0 || x >= src.width - 1 || y < 0 || y >= src.height - 1 )
			break;
	}

	// sample averaged line into buffer
	boost::scoped_array< int > buffer( new int[ nExtension + 2 ] );
	p = center - direction * ( ( nExtension >> 1 ) + 1 );
	if ( t == 4 )
		// fast version
		for ( int i = 0; i < nExtension + 2; i++ )
		{
			buffer[ i ] = 2 * subpixSampleFast( src, p ) +	subpixSampleFast( src, p - n ) + subpixSampleFast( src, p + n );
			p += direction;
		}
	else
		// slow but safe version
		for ( int i = 0; i < nExtension + 2; i++ )
		{
			buffer[ i ] = 2 * subpixSampleSafe( src, p ) + subpixSampleSafe( src, p - n ) + subpixSampleSafe( src, p + n );
			p += direction;
		}

	// apply difference and copy to destination buffer
	for ( int i = 0; i < nExtension; i++ )
		pDst[ i ] = ( buffer[ i ] - buffer[ i + 2 ] );
}


/**
 * \internal
 * Sample a line centered in a given point with subpixel precision and compute the gradient.
 *
 * This function does not respect the origin flag and assumes a y-coordinate of 0 is top.
 *
 * @param pSrc grey-scale image
 * @param pDst array of nExtension integers, where the gradient values will be stored
 * @param center center-point of the line in the image
 * @param nExtension length of line in pixels
 * @param direction direction vector of the line (normalized)
 */
void simpleLineGradient( const Image& src, int* pDst, const Math::Vector< float, 2 >& center, 
	const Math::Vector< float, 2 >& direction, int nExtension )
{
	Math::Vector< float, 2 > p;

	// check if one end is outside the image
	int t;
	for ( t = 0; t < 2; t++ )
	{
		p = center + ( (t&1) ? -1 : 1 ) * ( ( nExtension >> 1 ) + 1 ) * direction;
		int x = static_cast< int >( floorf( p( 0 ) ) );
		int y = static_cast< int >( floorf( p( 1 ) ) );
		if ( x < 0 || x >= src.width - 1 || y < 0 || y >= src.height - 1 )
			break;
	}

	// sample line and compute gradient
	p = center - direction * ( ( nExtension >> 1 ) + 0.5f );
	if ( t == 2 )
	{
		// fast version
		int lastVal = subpixSampleFast( src, p );
		for ( int i = 0; i < nExtension; i++ )
		{
			p += direction;
			int thisVal = subpixSampleFast( src, p );
			pDst[ i ] = lastVal - thisVal;
			lastVal = thisVal;
		}
	}
	else
	{
		// slow but safe version
		int lastVal = subpixSampleSafe( src, p );
		for ( int i = 0; i < nExtension; i++ )
		{
			p += direction;
			int thisVal = subpixSampleSafe( src, p );
			pDst[ i ] = lastVal - thisVal;
			lastVal = thisVal;
		}
	}
}

} } // namespace Ubitrack::Vision
