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
 * A class for handling OpenCV images in a "Resource Acquisition Is Initialization" (RAII) fashion
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#ifndef __UBITRACK_VISION_IMAGE_H_INCLUDED__
#define __UBITRACK_VISION_IMAGE_H_INCLUDED__

#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/utility.hpp>
#include <boost/serialization/access.hpp>
#include <opencv/cxcore.h>
#include <utVision.h>
#include <utMeasurement/Measurement.h>

namespace Ubitrack { namespace Vision {

/**
 * @ingroup vision
 * A wrapper object for IplImages in a "Resource Acquisition Is Initialization" (RAII) fashion.
 *
 * We don't want to change the OpenCV feeling more than necessary, but we need images that
 * destroy themselves when they run out of scope!
 */
class UTVISION_EXPORT Image
	: public IplImage
	, private boost::noncopyable
    //, public boost::enable_shared_from_this< Image >
{
public:
	/**
	 * Some abbreviations for shared pointers of Image
	 */
	typedef boost::shared_ptr< Image > Ptr;
	typedef boost::shared_ptr< const Image > ConstPtr;
    

    /**
     * Defining image dimensions
     */
    struct Dimension {
        Dimension( void ) {
            width = 0;
            height = 0;
        };
        Dimension( int width, int height ) {
            this->width = width;
            this->height = height;
        };
        int width;
        int height;
        bool operator== (Dimension& d) {
            return (d.width == width) && (d.height == height);
        };
        bool operator== (const Dimension& d) {
            return (d.width == width) && (d.height == height);
        };

    };


	/**
	 * Creates a reference to an existing image array.
	 * The resulting \c Image object does not own the data.
	 *
	 * @param nWidth width of image in pixels
	 * @param nHeight height of image in pixels
	 * @param nChannels number of channels (1=grey, 3=rgb)
	 * @param pImageData pointer to raw image data
	 * @param nDepth information about pixel representation (see OpenCV docs)
	 * @param nOrigin 0 if pixels start in top-left corner, 1 if in bottom-left
	 * @param nAlign alignment of rows in bytes
	 */
	Image( int nWidth, int nHeight, int nChannels, void* pImageData,
		int nDepth = IPL_DEPTH_8U, int nOrigin = 0, int nAlign = 4 );

	/**
	 * Creates a new image and allocates memory for it.
	 *
	 * @param nWidth width of image in pixels
	 * @param nHeight height of image in pixels
	 * @param nChannels number of channels (1=grey, 3=rgb)
	 * @param nDepth information about pixel representation (see OpenCV docs)
	 * @param nOrigin 0 if pixels start in top-left corner, 1 if in bottom-left
	 */
	Image( int nWidth = 0, int nHeight = 0, int nChannels = 1, int nDepth = IPL_DEPTH_8U, int nOrigin = 0 );

	/**
	 * Create from pointer to IplImage.
	 *
	 * @param pIplImage pointer to existing IplImage
	 * @param bDestroy if true, the IplImage is destroyed (using cvReleaseImageHeader) and
	 * ownership of the data is taken. Otherwise the data is not owned.
	 */
	explicit Image( IplImage* pIplImage, bool bDestroy = true );

	/**
	 * Create from Mat object
	 */
	explicit Image( cv::Mat & img );

	/** releases the image data if the data block is owned by this object. */
	~Image();

	/** convert to CvArr* for usage in some OpenCV functions */
	operator CvArr*()
		{ return static_cast< IplImage* >( this ); }

	/** convert to CvArr* for usage in some OpenCV functions */
	operator const CvArr*() const
	{ return static_cast< const IplImage* >( this ); }

	/** also convert to IplImage* for more consistent interface */
	operator IplImage*()
	{ return static_cast< IplImage* >( this ); }

    /// COMMENTED OUT: FUNCTIONALITY HAS TO BE CLARIFIED
	///** convert to cv::Mat */
	//operator cv::Mat()
	//{ return cv::Mat (&IplImage());	}


	/** also convert to IplImage* for more consistent interface */
	operator const IplImage*() const
	{ return static_cast< const IplImage* >( this ); }
	
	/** returns the value of a pixel */
	template< class T >
	T getPixel( unsigned x, unsigned y ) const
	{ return reinterpret_cast< const T* >( imageData + y * widthStep )[ x ]; }

	template< class T >
	void setPixel( unsigned x, unsigned y, T val )
	{ reinterpret_cast< T* >( imageData + y * widthStep )[ x ] = val; }

	/**
	 * Convert color space.
	 * Wraps \c cvCvtColor, but keeps the origin flag intact.
	 *
	 * @param nCode conversion Code (see OpenCV docs of \c cvCvtColor, e.g. CV_RGB2GRAY)
	 * @param nChannels number of channels (1=grey, 3=rgb)
	 * @param nDepth information about pixel representation (see OpenCV docs)
	 */
	boost::shared_ptr< Image > CvtColor( int nCode, int nChannels, int nDepth = IPL_DEPTH_8U ) const;

    /** Allocates empty memory of the size of the image */
	boost::shared_ptr< Image > AllocateNew() const;

	/** Creates a copy of this image */
	boost::shared_ptr< Image > Clone() const;
	
	/** creates an image which is half the size */
	boost::shared_ptr< Image > PyrDown() const;
	
	/** creates an image with the given size */
	boost::shared_ptr< Image > Scale( int width, int height ) const;

    /** creates an image with the given scale factor 0.0 < f <= 1.0 */
	boost::shared_ptr< Image > Scale( double scale ) const;

	/** Creates an image with adapted contrast and brightness */
	boost::shared_ptr< Image > ContrastBrightness( int contrast, int brightness ) const;

	/** inverts the image. Implemented only for 8 bit greyscale images */
	void Invert();

    /** Has the image only one channel?  */
    bool isGrayscale( void ) const;

    /** Convert to grayscale */
    Ptr getGrayscale( void ) const;

    /** Returns the dimension of the image */
    Dimension dimension( void ) const {
        return Dimension(width, height);
    };

    /**
     * @brief Saves an image as compressed JPEG
     * @param filename Filename of the image
     * @param compressionFactor Sets the quality of the compression from 0...100. Default is 95.
     * @warning The filename must end with 'jpg' to get the desired result.
     * 
     */
    void saveAsJpeg( const std::string filename, int compressionFactor = 95 ) const;

    /**
     * @brief Compresses an image in memory using JPEG compression
     * @param buffer A reference to a memory buffer holding the result. The buffer will be resized to fit the result.
     * @param compressionFactor Sets the quality of the compression from 0...100. Default is 95.
     */
    void encodeAsJpeg( std::vector< uchar >& buffer, int compressionFactor = 95 ) const;


private:
	// does this object own the data imageData points to?
	bool m_bOwned;
	
	friend class ::boost::serialization::access;
	/** boost serialization helper. does not do any useful serialization... */
	template< class Archive >
	void serialize( Archive& ar, const unsigned int )
	{ char X( 'X' ); ar & X; }
};

typedef Image::Ptr ImagePtr;
typedef Image::ConstPtr ConstImagePtr;
typedef const Image ConstImage;

} // namespace Vision



namespace Measurement {
/** define a shortcut for image measurements */
typedef Measurement< Ubitrack::Vision::Image > ImageMeasurement;
typedef Measurement< Ubitrack::Vision::ConstImage > ConstImageMeasurement;
} // namespace Measurement
} // namespace Ubitrack




#endif
