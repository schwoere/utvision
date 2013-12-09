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

#include "Undistortion.h"
#include <utUtil/CalibFile.h>
#include <utUtil/Exception.h>
#include <opencv/cv.h>

// get a logger
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.Undistortion" ) );
 
namespace Ubitrack { namespace Vision {

Undistortion::Undistortion( const std::string& intrinsicMatrixFile, const std::string& distortionFile )
{
	initParams( intrinsicMatrixFile, distortionFile );
}


Undistortion::Undistortion( const Math::Matrix< double, 3, 3 >& intrinsicMatrix, const Math::Vector< double, 8 >& distortion )
{
	initParams( intrinsicMatrix, distortion );
}


void Undistortion::initParams( const Math::Matrix< double, 3, 3 >& intrinsicMatrix, const Math::Vector< double, 8 >& distortion )
{
	m_intrinsics = intrinsicMatrix;
	m_coeffs = distortion;
}


void Undistortion::initParams( const std::string& intrinsicMatrixFile, const std::string& distortionFile )
{
	// read intrinsics
	if ( !intrinsicMatrixFile.empty() )
	{
		
		Measurement::Matrix3x3 measMat;
		measMat.reset( new Math::Matrix< double, 3, 3 >() );
		Util::readCalibFile( intrinsicMatrixFile, measMat );
		m_intrinsics = *measMat;
		LOG4CPP_DEBUG(logger, "Loaded calibration file : " << m_intrinsics);
	}
	else
	{
		m_intrinsics( 0, 0 ) = 400;
		m_intrinsics( 0, 1 ) = 0;
		m_intrinsics( 0, 2 ) = -160;
		m_intrinsics( 1, 0 ) = 0;
		m_intrinsics( 1, 1 ) = 400;
		m_intrinsics( 1, 2 ) = -120;
		m_intrinsics( 2, 0 ) = 0;
		m_intrinsics( 2, 1 ) = 0;
		m_intrinsics( 2, 2 ) = -1;
	}
	
	// read coefficients
	if ( !distortionFile.empty() )
	{
        // first try new, eight element distortion file
		Measurement::Vector8D measVec;
		measVec.reset( new Math::Vector< double, 8 >() );
        try {
		    Util::readCalibFile( distortionFile, measVec );
            m_coeffs = *measVec;
        } catch ( Ubitrack::Util::Exception ) {
            LOG4CPP_ERROR( logger, "Cannot read new image distortion model. Trying old format." );
            // try old format, this time without exception handling
            Measurement::Vector4D measVec4D;
            measVec4D.reset( new Math::Vector< double, 4 >() );
            Util::readCalibFile( distortionFile, measVec4D );
            m_coeffs = Math::Vector< double, 8 >::zeros();
            boost::numeric::ublas::subrange(m_coeffs, 0, 4 ) = *measVec4D;
        }
		
	}
	else
		m_coeffs = Math::Vector< double, 8 >::zeros();
}


boost::shared_ptr< Image > Undistortion::undistort( boost::shared_ptr< Image > pImage )
{
	// shortcut if no distortion
	if ( !hasDistortion() )
		return pImage;

	return undistort( *pImage );
}


boost::shared_ptr< Image > Undistortion::undistort( const Image& image )
{
	// shortcut if no distortion
	if ( !hasDistortion() )
		return image.Clone();
		
	// initialize the distortion map
	initMap( image.width, image.height, image.origin );
	
	// undistort
	boost::shared_ptr< Image > pUndistorted( new Image( image.width, image.height, image.nChannels, image.depth ) );
	pUndistorted->origin = image.origin;
	memcpy( pUndistorted->colorModel,  image.colorModel, 4 );
	memcpy( pUndistorted->channelSeq,  image.channelSeq, 4 );

	cvRemap( image, *pUndistorted, *m_pMapX, *m_pMapY );
	
	// send result
	return pUndistorted;
}


void Undistortion::initMap( int width, int height, int origin )
{
	// skip if already initialized with same values
	if ( m_pMapX && m_pMapX->width == width && m_pMapX->height == height )
		return;
		
	LOG4CPP_INFO( logger, "Creating undistortion map" );
	LOG4CPP_DEBUG( logger, "coeffs=" << m_coeffs );
	LOG4CPP_DEBUG( logger, "intrinsic=" << m_intrinsics );
	
	// copy ublas to OpenCV parameters
	CvMat* pCvCoeffs = cvCreateMat( 1, 8, CV_32FC1 );
	for ( unsigned i = 0; i < 8; i++ )
		pCvCoeffs->data.fl[ i ] = static_cast< float >( m_coeffs( i ) );
		
	CvMat* pCvIntrinsics = cvCreateMat( 3, 3, CV_32FC1 );
	for ( unsigned i = 0; i < 3; i++ )
		for ( unsigned j = 0; j < 3; j++ )
			cvmSet( pCvIntrinsics, i, j, m_intrinsics( i, j ) );
	
	// compensate for left-handed OpenCV coordinate frame
	for ( unsigned i = 0; i < 3; i++ )
		cvmSet( pCvIntrinsics, i, 2, cvmGet( pCvIntrinsics, i, 2 ) * -1 );
	
	// compensate if origin==0
	if ( !origin )
	{
		cvmSet( pCvIntrinsics, 1, 2, height - 1 - cvmGet( pCvIntrinsics,  1, 2 ) );
		cvmSet( pCvCoeffs, 0, 2, cvmGet( pCvCoeffs, 0, 2 ) * -1 );
	}

	// initialize the distortion map
	// create map images
	m_pMapX.reset( new Image( width, height, 1, IPL_DEPTH_32F ) );
	m_pMapY.reset( new Image( width, height, 1, IPL_DEPTH_32F ) );
	
	cvInitUndistortMap( pCvIntrinsics, pCvCoeffs, *m_pMapX, *m_pMapY );
	
	LOG4CPP_TRACE( logger, "origin=" << origin );
	Math::Vector< double, 2 > startPixel( *reinterpret_cast< float* >( m_pMapX->imageData ), *reinterpret_cast< float* >( m_pMapY->imageData ) );
	LOG4CPP_DEBUG( logger, "first pixel (0, 0) mapped from " << startPixel );
		
	// release data
	cvReleaseMat( &pCvCoeffs );
	cvReleaseMat( &pCvIntrinsics );
}	

} } // namespace Ubitrack::Vision
