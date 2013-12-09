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
 * Implements basic routines for handling OpenCV images in a "Resource Acquisition Is Initialization" (RAII) fashion
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <utUtil/Exception.h>
#include "Image.h"

namespace Ubitrack { namespace Vision {


Image::Image( int nWidth, int nHeight, int nChannels, void* pImageData, int nDepth, int nOrigin, int nAlign )
	: m_bOwned( false )
{
	cvInitImageHeader( this, cvSize( nWidth, nHeight ), nDepth, nChannels, nOrigin, nAlign );
	imageData = static_cast< char* >( pImageData );
}


Image::Image( int nWidth, int nHeight, int nChannels, int nDepth, int nOrigin )
	: m_bOwned( true )
{
	cvInitImageHeader( this, cvSize( nWidth, nHeight ), nDepth, nChannels, nOrigin );
	imageDataOrigin = imageData = static_cast< char* >( cvAlloc( imageSize ) );
}


Image::Image( IplImage* pIplImage, bool bDestroy )
	: m_bOwned( bDestroy )
{
	memcpy( static_cast< IplImage* >( this ), pIplImage, sizeof( IplImage ) );
	if ( bDestroy )
		cvReleaseImageHeader( &pIplImage );
}

Image::Image( cv::Mat & img )
	: m_bOwned( false )
{
	IplImage imgIpl = img;
	memcpy( static_cast< IplImage* >( this ), &imgIpl, sizeof( IplImage ) );
}


Image::~Image()
{
	if ( m_bOwned )
		cvFree( reinterpret_cast< void** >( &imageDataOrigin ) );
}


boost::shared_ptr< Image > Image::CvtColor( int nCode, int nChannels, int nDepth ) const
{
	boost::shared_ptr< Image > r( new Image( width, height, nChannels, nDepth ) );
	cvCvtColor( this, *r, nCode ); 
	r->origin = origin; 
	return r;
}


void Image::Invert()
{
	if ( nChannels != 1 || depth != IPL_DEPTH_8U )
		UBITRACK_THROW ("Operation only supported on 8-Bit grayscale images");

	for ( int i=0; i < width; i++ )
	{
		for ( int j=0; j < height; j++ )
		{
			unsigned char val = 255 - getPixel< unsigned char >( i, j );
			setPixel< unsigned char >( i, j, val );
		}
	}
}

boost::shared_ptr< Image > Image::AllocateNew() const
{
    return ImagePtr( new Image( width, height, nChannels, depth, origin) );
}

boost::shared_ptr< Image > Image::Clone() const
{
	return boost::shared_ptr< Image >( new Image( cvCloneImage( *this ) ) );
}


boost::shared_ptr< Image > Image::PyrDown() const
{
	boost::shared_ptr< Image > r( new Image( width / 2, height / 2, nChannels, depth, origin ) );
	cvPyrDown( *this, *r );
	return r;
}


boost::shared_ptr< Image > Image::Scale( int width, int height ) const
{
	boost::shared_ptr< Image > scaledImg( new Image( width, height, nChannels, depth, origin ) );
	cvResize( *this, *scaledImg );
	return scaledImg;
}

/** creates an image with the given scale factor 0.0 < f <= 1.0 */
boost::shared_ptr< Image > Image::Scale( double scale ) const
{
    if (scale <= 0.0 || scale > 1.0)
        UBITRACK_THROW( "Invalid scale factor" );
    return Scale( static_cast< int >( width * scale ), static_cast< int > ( height * scale ) );
}

bool Image::isGrayscale() const
{
    return nChannels == 1;
}

Image::Ptr Image::getGrayscale( void ) const
{
    if ( !isGrayscale() ) {
        // TODO: Has the image, if not grayscale, really three channels then?
        return CvtColor( CV_RGB2GRAY, 1, depth);
    } else {
        return Clone();
    }
}


// This function is copied from http://mehrez.kristou.org/opencv-change-contrast-and-brightness-of-an-image/
boost::shared_ptr< Image > Image::ContrastBrightness( int contrast, int brightness ) const
{
	if(contrast > 100) contrast = 100;
	if(contrast < -100) contrast = -100;
	if(brightness > 100) brightness = 100;
	if(brightness < -100) brightness = -100;

	uchar lut[256];

	CvMat* lut_mat;
	int hist_size = 256;
	float range_0[]={0,256};
	float* ranges[] = { range_0 };
	int i;

	IplImage * dest = cvCloneImage(this);
	
	IplImage * GRAY;
	if (this->nChannels == 3)
	{
		GRAY = cvCreateImage(cvGetSize(this),this->depth,1);
		cvCvtColor(this,GRAY,CV_RGB2GRAY);
	}
	else
	{
		GRAY = cvCloneImage(this);
	}
    lut_mat = cvCreateMatHeader( 1, 256, CV_8UC1 );
    cvSetData( lut_mat, lut, 0 );
	/*
     * The algorithm is by Werner D. Streidt
     * (http://visca.com/ffactory/archives/5-99/msg00021.html)
     */
	if( contrast > 0 )
    {
        double delta = 127.* contrast/100;
        double a = 255./(255. - delta*2);
        double b = a*(brightness - delta);
        for( i = 0; i < 256; i++ )
        {
            int v = cvRound(a*i + b);

            if( v < 0 )
                v = 0;
            if( v > 255 )
                v = 255;
            lut[i] = v;
        }
    }
    else
    {
        double delta = -128.* contrast/100;
        double a = (256.-delta*2)/255.;
        double b = a* brightness + delta;
        for( i = 0; i < 256; i++ )
        {
            int v = cvRound(a*i + b);
            if( v < 0 )
                v = 0;

            if( v > 255 )
                v = 255;
            lut[i] = v;
        }
    }
	if (this->nChannels ==3)
	{
		IplImage * R = cvCreateImage(cvGetSize(this),this->depth,1);
		IplImage * G = cvCreateImage(cvGetSize(this),this->depth,1);
		IplImage * B = cvCreateImage(cvGetSize(this),this->depth,1);
		cvCvtPixToPlane(this,R,G,B,NULL);
		cvLUT( R, R, lut_mat );
		cvLUT( G, G, lut_mat );
		cvLUT( B, B, lut_mat );
		cvCvtPlaneToPix(R,G,B,NULL,dest);
		cvReleaseImage(&R);
		cvReleaseImage(&G);
		cvReleaseImage(&B);
	}
	else
	{
		cvLUT( GRAY, dest, lut_mat );
	}
	cvReleaseImage(&GRAY);
	cvReleaseMat( &lut_mat);
	
	return boost::shared_ptr< Image >( new Image( dest, true ) );
}


void Image::saveAsJpeg( const std::string filename, int compressionFactor ) const
{
    std::vector< int > params;
    compressionFactor = std::min( 100, std::max ( 0, compressionFactor ) );
    params.push_back( CV_IMWRITE_JPEG_QUALITY );
    params.push_back( compressionFactor );
    cv::imwrite( filename, cv::Mat( *this ), params );
}


void Image::encodeAsJpeg( std::vector< uchar >& buffer, int compressionFactor ) const
{
    std::vector< int > params;
    compressionFactor = std::min( 100, std::max ( 0, compressionFactor ) );
    params.push_back( CV_IMWRITE_JPEG_QUALITY );
    params.push_back( compressionFactor );
    cv::imencode( ".jpg", cv::Mat( *this ), buffer, params );
}


} } // namespace Ubitrack::Vision
