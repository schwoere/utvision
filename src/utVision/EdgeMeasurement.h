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
 * Measurement function for edge-based tracking.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#ifndef __UBITRACK_VISION_EDGEMEASUREMENT_H_INCLUDED__
#define __UBITRACK_VISION_EDGEMEASUREMENT_H_INCLUDED__

#include <vector>
#include <utVision.h>
#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include <utCalibration/Function/MultiplePointProjection.h>
#include "EdgeExtraction.h"
#include "Image.h"

//#include <iostream>

namespace Ubitrack { namespace Vision {

// magic numbers
static int g_minEdgeIntensity = 10;

/**
 * Computes a function that needs to be minimized for edge-based tracking
 * @param T double or float
 * @param MaximumT type of edge to find can be \c FindEdgeAbsoluteMaximum or \c FindEdgePositiveMaximum
 * @param GradientT type of gradient detector, e.g. ExtractLineSimpleGradient or ExtractLineSobel
 */
template< typename T, class MaximumT = FindEdgeAbsoluteMaximum, class GradientT = ExtractLineSimpleGradient >
class EdgeListMeasurementFunction
{
public:
	/**
	 * Constructor
	 * @param p3d reference to list of edgel-centerpoints
	 * @param p3d2 reference to list of other points on edge to define the edge direction.
	 * @param K camera intrinsics matrix
	 * @param scale defines the size of the search region by computing 
	 *	 len = sqrt( n^T * scale * n )
	 *  where n is the normalized gradient direction of each edge (|n|=1). If you want to 
	 * use a constant search length l, set scale = l^2 * I.
	 * @param image image to search for gradients
	 * @param pDebugImage if not NULL, will draw some information in this image
	 */
	EdgeListMeasurementFunction( const std::vector< Math::Vector< T, 3 > >& p3d, 
		const std::vector< Math::Vector< T, 3 > >& p3d2, const Math::Matrix< T, 3, 3 >& K, 
		const Math::Matrix< T, 2, 2 >& scale, const Image& image, Image* pDebugImage = 0, T outlierThreshold = 1 )
		: m_p3d( p3d )
		, m_p3d2( p3d2 )
		, m_K( K )
		, m_scale( scale )
		, m_image( image )
		, m_pDebugImage( pDebugImage )
		, m_outlierThreshold(outlierThreshold )
		, m_fBoundedResidual( 1e50 )
	{
	}

	template< class VT1, class VT2 > 
	void evaluate( VT1& result, const VT2& input ) const
	{
		namespace ublas = boost::numeric::ublas;

		m_goodEdgels = 0;

		// compute edge points
		if ( !m_edgePoints.size() )
			findEdgePoints( input );

		// project first points and calculate jacobian
		Math::Vector< T > p2d( m_p3d.size() * 2 );
		Calibration::Function::MultiplePointProjection< T >( m_p3d, m_K ).evaluate( p2d, input );

		for ( unsigned i = 0; i < m_p3d.size(); i++ )
		{
			// remove too small intensity edges
			// TODO: magic number -> bad
			if ( m_intensities[ i ] < g_minEdgeIntensity )
				result( i ) = 0;
			else
			{
				// compute result
				result( i ) = ublas::inner_prod( m_normals[ i ], 
					Math::Vector< T, 2 >( p2d( i * 2 ), p2d( i * 2 + 1 ) ) - m_edgePoints[ i ] );

				if ( fabs( result( i ) ) < m_outlierThreshold * m_searchLengths[ i ] )
					m_goodEdgels++;
			}
			//std::cout << i << ": 3d (" << m_p3d[ i ] << ") -> 2d (" << p2d(i*2) << ", " << p2d(i*2+1) << 
			//	"), edge " << m_edgePoints[ i ] << ", normal " << m_normals[ i ] << ", error " << result( i ) << std::endl;

		}
		//std::cout << "error :" << double( ublas::norm_2( result ) ) << std::endl;
	}

	template< class VT1, class VT2, class MT > 
	void evaluateWithJacobian( VT1& result, const VT2& input, MT& J ) const
	{
		namespace ublas = boost::numeric::ublas;
		m_goodEdgels = 0;
		double fBoundedResidual = 0;

		// compute edge points
		if ( !m_edgePoints.size() )
			findEdgePoints( input );

		// project first points and calculate jacobian
		Math::Vector< T > p2d( m_p3d.size() * 2 );
		Math::Matrix< T, 0, 0 > j2d( m_p3d.size() * 2, input.size() );
		Calibration::Function::MultiplePointProjection< T >( m_p3d, m_K ).evaluateWithJacobian( p2d, input, j2d );

		for ( unsigned i = 0; i < m_p3d.size(); i++ )
			// remove too small intensity edges
			// TODO: magic number -> bad
			if ( m_intensities[ i ] < g_minEdgeIntensity )
			{
				noalias( ublas::row( J, i ) ) = Math::Vector< T >::zeros( input.size() );
				result( i ) = 0;
				fBoundedResidual += m_outlierThreshold * m_searchLengths[ i ] * m_outlierThreshold * m_searchLengths[ i ];
			}
			else
			{
				// compute result
				result( i ) = ublas::inner_prod( m_normals[ i ], 
					Math::Vector< T, 2 >( p2d( i * 2 ), p2d( i * 2 + 1 ) ) - m_edgePoints[ i ] );

				if ( fabs( result( i ) ) < m_outlierThreshold * m_searchLengths[ i ] )
				{
					fBoundedResidual += result( i ) * result( i );
					m_goodEdgels++;
				}
				else
					fBoundedResidual += m_outlierThreshold * m_searchLengths[ i ] * m_outlierThreshold * m_searchLengths[ i ];
				

				// compute jacobian
				noalias( ublas::row( J, i ) ) = ublas::prod( m_normals[ i ], ublas::subrange( j2d, i * 2, (i+1) * 2, 0, input.size() ) );
			}
			
		if ( m_fBoundedResidual > fBoundedResidual )
			m_fBoundedResidual = fBoundedResidual;
	}

	template< class VT2, class MT > 
	void jacobian( const VT2& input, MT& J ) const
	{
		namespace ublas = boost::numeric::ublas;

		// compute edge points
		if ( !m_edgePoints.size() )
			findEdgePoints( input );

		// project first points and calculate jacobian
		Math::Matrix< T, 0, 0 > j2d( m_p3d.size() * 2, input.size() );
		Calibration::Function::MultiplePointProjection< T >( m_p3d, m_K ).jacobian( input, j2d );

		for ( unsigned i = 0; i < m_p3d.size(); i++ )
			// remove too small intensity edges
			// TODO: magic number -> bad
			if ( m_intensities[ i ] < g_minEdgeIntensity )
				noalias( ublas::row( J, i ) ) = Math::Vector< T >::zeros( input.size() );
			else
			{
				// results...
				noalias( ublas::row( J, i ) ) = ublas::prod( m_normals[ i ], ublas::subrange( j2d, i * 2, (i+1) * 2, 0, input.size() ) );
				//std::cout << "J( " << i << " )=" << ublas::row( J, i ) << std::endl;
			}
	}

	/* do robust estimation */
	bool noWeights() const
	{ return false; }

	/* compute Tukey weights for robust estimation, assume that is is called AFTER evaluate(WithJacobian) */
	template< class VT1, class VT2 > 
	void computeWeights( const VT1& errorVector, VT2& weightVector ) const
	{
		for ( unsigned i = 0; i < errorVector.size(); i++ )
		{	
			typename VT1::value_type ro, e = 0, weight, c;
			e = fabs( errorVector( i ) );
			c = m_searchLengths[ i ] * m_outlierThreshold;
			
			if ( e != 0 )
			{
				if(!(e>c*c))
					ro=(pow(c,2)/6)*(1-pow((1- e/(c*c)),3));
				else
					ro=(c*c/6);
				weight = sqrt(ro)/ro;
			}			
			else
				weight = 0;


			weightVector( i ) = 1; //weight; // TODO
		}
		
		m_outlierThreshold = (T)(std::max( m_outlierThreshold - 0.1, 0.4 ));
	}
	
	float getGoodEdgelsPercentage() const
	{ return float( m_goodEdgels ) / m_p3d.size(); }
	
	float getBoundedResidual() const
	{ return float( m_fBoundedResidual ); }

protected:
	/** finds points on the edge */
	template< class VT2 >
	void findEdgePoints( const VT2& input ) const
	{
		// initialize points
		m_edgePoints.resize( m_p3d.size() );
		m_normals.resize( m_p3d.size() );
		m_searchLengths.resize( m_p3d.size() );
		m_intensities.resize( m_p3d.size() );

		// project first points
		Math::Vector< T > p2d( m_p3d.size() * 2 );
		Calibration::Function::MultiplePointProjection< T >( m_p3d, m_K ).evaluate( p2d, input );

		// project second points
		Math::Vector< T > p2d2( m_p3d.size() * 2 );
		Calibration::Function::MultiplePointProjection< T >( m_p3d2, m_K ).evaluate( p2d2, input );

		for ( unsigned i = 0; i < m_p3d.size(); i++ )
		{
			// compute edge direction
			Math::Vector< T, 2 > dir( p2d2( i * 2 ) - p2d( i * 2 ), p2d2( i * 2 + 1 ) - p2d( i * 2 + 1 ) );
			T searchLen = ublas::norm_2( dir );
			dir /= searchLen;

			// compute normal direction
			Math::Vector< T, 2 > normal( dir( 1 ), -dir( 0 ) );
			if ( m_image.origin )
				normal *= -1;

			// experimental
			searchLen = sqrt( ublas::inner_prod( normal, ublas::prod( m_scale, normal ) ) );

			// determine number of pixels to search
			int searchPixels = static_cast< int >( searchLen );
			if ( !searchPixels  )
				searchPixels = 1;

			// search for position of maximum
			Math::Vector< T, 2 > start( p2d( i * 2 ), p2d( i * 2 + 1 ) );
			int maxIntensity;
			T maxPos = findEdge< MaximumT, GradientT >
				( m_image, start, normal, searchPixels, maxIntensity );

			// set parameters
			m_intensities[ i ] = maxIntensity;
			m_searchLengths[ i ] = T( searchPixels );
			m_normals[ i ] = normal;
			m_edgePoints[ i ] = start + normal * maxPos;

			if ( m_pDebugImage )
			{
				Math::Vector< float, 2 > p1( start + searchPixels * normal );
				Math::Vector< float, 2 > p2( start - searchPixels * normal );
				cvLine( *m_pDebugImage, 
					cvPoint( cvRound( p1( 0 ) * 16 ), cvRound( p1( 1 ) * 16 ) ), 
					cvPoint( cvRound( p2( 0 ) * 16 ), cvRound( p2( 1 ) * 16 ) ), 
					CV_RGB( 128, 128, 0 ), 1, CV_AA, 4 );
				cvCircle( *m_pDebugImage, 
					cvPoint( cvRound( m_edgePoints[ i ]( 0 ) * 16 ), cvRound( m_edgePoints[ i ]( 1 ) * 16 ) ),
					cvRound( m_pDebugImage->width / 1600.0 * 16 ),
					m_intensities[ i ] >= g_minEdgeIntensity ? CV_RGB( 0, 255, 0 ) : CV_RGB( 255, 0, 0 ), 
					-1, CV_AA, 4 );
			}
		}
		
		// only draw on first iteration
		m_pDebugImage = 0;
	}

	// parameters
	const std::vector< Math::Vector< T, 3 > >& m_p3d; 
	const std::vector< Math::Vector< T, 3 > >& m_p3d2;
	const Math::Matrix< T, 3, 3 >& m_K;
	const Math::Matrix< T, 2, 2 >& m_scale;
	const Image& m_image;
	mutable Image* m_pDebugImage;
	mutable T m_outlierThreshold;

	/** positions of points on edge */
	mutable std::vector< Math::Vector< T, 2 > > m_edgePoints;

	/** point intensities */
	mutable std::vector< int > m_intensities;

	/** normal directions */
	mutable std::vector< Math::Vector< T, 2 > > m_normals;

	/** point intensities */
	mutable std::vector< T > m_searchLengths;

	/** number of good edgels */
	mutable int m_goodEdgels;
	
	/** residual where outliers have a constant maximum weight */
	mutable double m_fBoundedResidual;

};

} } // namespace Ubitrack::Vision

#endif

