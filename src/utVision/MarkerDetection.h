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
 * Routines to detect square markers in images.
 *
 * @author Daniel Pustka <daniel.pustka@in.tum.de>
 */

#ifndef __UBITRACK_VISION_MARKERDETECTION_H_INCLUDED__
#define __UBITRACK_VISION_MARKERDETECTION_H_INCLUDED__

#include <list>
#include <vector>
#include <map>
#include <utVision.h>
#include <utMath/Vector.h>
#include <utMath/Matrix.h>
#include <utMath/Pose.h>
#include "Image.h"
#include "PixelFlow.h"

namespace Ubitrack { namespace Vision { namespace Markers {

/** a list of corner points */
typedef std::vector< Ubitrack::Math::Vector< float, 2 > > CornerList;

/** a list of polygons */
typedef std::vector< CornerList > MarkerList;

/** information about a marker */
struct MarkerInfo
{
	/** constructor */
	MarkerInfo( float _fSize = 0 )
		: fSize( _fSize )
		, refinement( EEdgeRefinedPose )
		, bEnableInterpolation( false )
		, bEnablePixelFlow( false )
		, bEnableFlipCheck( true )
		, bEnableTracking( true )
		, bEnableFastTracking( false )
		, bUseInitialPose( false )
		, bCalculateCovariance( false )
		, found( ENotFound )
		, nPrevPoseValidator( 0 )
		, fResidual( 0 )
		, nLostFrameCount( 0 )
	{}

	/** marker size in meters */
	float fSize;

	/** desired level of refinement */
	enum 
	{ 
		ECorners,        // only corner location
		EInitialPose,    // also perform initial linear pose estimation
		ERefinedPose,    // refined non-linear pose estimation
		EEdgeRefinedPose // even more refined pose estimation that takes inner-marker edges into account
	}
	refinement;

	// options
	
	/** marker pose prediction before refinement (linear extrapolation) */
	bool bEnableInterpolation;
	
	/** enables pixel flow for prediction */
	bool bEnablePixelFlow;
	
	/** check alternate marker pose to prevent flipping? */
	bool bEnableFlipCheck;
	
	/** enable tracking without full image analysis */
	bool bEnableTracking;
	
	/** 
	 * if false, full image analysis is always performed and tracking is only done 
	 * when the marker is not found. This is usually more robust.
	*/
	bool bEnableFastTracking;

	/** is the pose initialized for a previous frame? */
	bool bUseInitialPose;

	/** calculate covariance? */
	bool bCalculateCovariance;

	/** return value that describes if and how the marker was detected */
	enum FoundState
	{
		ENotFound = 0,
		EPixelFlowFound = 1,  // not found, only updated by pixel flow
		ERefinementFound = 2, // found by tracking/refinement
		EFullScanFound = 3    // found by detection
	}
	found;

	/** found marker corners */
	CornerList corners;
	
	/** pose of the marker */
	Math::Pose pose;

	/** covariance matrix */
	Math::Matrix< double, 6, 6 > covariance;

	// INTERNAL 

	/** previous pose of the marker */
	Math::Pose prevPose;

	/** does the previous pose exist? */
	unsigned nPrevPoseValidator	;

	/** the residual of marker after LM iterations */
	float fResidual;
	
	/** what procent of the marker is visible */
	int nVisibility;

	/** Counts how many frames ago the marker was lost */
	unsigned nLostFrameCount;
	
	/** information about pixel flow */
	PixelFlow pFlow;
};


/** map of marker infos */
typedef std::map< unsigned long long int, MarkerInfo > MarkerInfoMap;


#ifdef HAVE_LAPACK
/**
 * @ingroup vision
 * Detects markers in the image.
 *
 * @param img Greyscale image
 * @param markers Set of markers to detect. The map key is the normalized code of each marker. If the marker is detected,
 *    the method will set the found flag and, depending on the refinement value, also the corners and pose value.
 *    If a marker of code "0000" is present in the map, the method will detect ALL markers in the image and add them to the map.
 * @param K camera intrinsics matrix
 * @param pDebugImg creates a debug image if non-null
 * @param bRefine if true, only do refinement, no detection
 * @param uiCodeSize size of marker's bit pattern
 * @param uiMarkerSize overall size of the marker, counted in bits, including the border and the bit pattern
 * @param uiMask regions of the marker's bit pattern that belong to the ID (1) or not (0)
 * @param useInnerEdgels incorporate inner edgelets into pose refinement. May be unstable.
 */
UTVISION_EXPORT void detectMarkers( const Image& img, std::map< unsigned long long int, MarkerInfo >& markers, 
	const Math::Matrix< float, 3, 3 >& K, Image* pDebugImg = 0, bool bRefine = false, unsigned int iCodeSize = 4, 
	unsigned int iMarkerSize = 6, unsigned long long int uiMask = 0xFFFF, bool useInnerEdgels = true );
#endif

/**
 * @ingroup vision
 * Returns a list of all quadrangles found in the image.
 *
 * For better accuracy, the corner coordinates should be refined using refineCorners afterwards.
 * This function does not respect the origin flag and assumes a y-coordinate of 0 is top.
 *
 * @param img thresholded grey-scale input image. Note: The image will be destructively modified by this routine.
 * @param pDbgImg optional debug image which shows found contours.
 * @param origin additional offset that is added to contour coordinates. Useful when using ROIs.
 * @return list of found markers in image
 */
UTVISION_EXPORT MarkerList findQuadrangles( Image& img, Image * pDbgImg, cv::Point offset = cvPoint( 0, 0 ));

/**
 * @ingroup vision
 * Refines marker corners with sub-pixel accuracy.
 *
 * @param img grey-scale camera image
 * @param list of corners to be refined in clock-wise order
 * @param pDebugImg. If not NULL, some information will be drawn into this image for debugging
 * @return true if edges were considered strong enough for this to be a real marker
 */
UTVISION_EXPORT bool refineCorners( const Image& img, CornerList& corners, Image* pDebugImg = NULL );

/**
 * @ingroup vision
 * Returns a rectified image of a marker.
 *
 * @param img the grey-scale camera image
 * @param homography a homography as computed by Ubitrack::Calibration::squareHomograph
 * @param nSize sidelength of the resulting image in pixels
 */
UTVISION_EXPORT boost::shared_ptr< Image > getMarkerImage( const Image& img, const Math::Matrix< float, 3, 3 > homography, int nSize );

/**
 * @ingroup vision
 * Reads the marker code.
 *
 * @param img image of the marker
 * @param uiCodeSize size of marker's bit pattern 
 * @param uiMask regions of the marker's bit pattern that belong to the ID (1) or not (0)
 * @return unnormalized marker code, 0 if marker code could not be read
 */
UTVISION_EXPORT unsigned long long int readCode( const Image& img, unsigned int uiCodeSize, unsigned long long int uiMask );

/**
 * @ingroup vision
 * Normalizes a marker code.
 *
 * @param nCode the unnormalized code
 * @param codeSize side-length of code pattern
 * @param nRotations returns the number of counter-clockwise marker rotations used
 * @return normalized marker code
 */
UTVISION_EXPORT unsigned long long int normalizeCode( unsigned long long int nCode, int codeSize, int& nRotations );

/**
 * @ingroup vision
 * Computes edgels for a marker consisting of inner and outer edges
 *
 * @param markerCode the code of the marker
 * @param markerSize size of the marker
 * @param points1 a list of 3-vectors that will be filled with the center-points of each edgel
 * @param points2 a list of 3-vectors that will be filled with another point on each edgel
 * @param uiCodeSize size of marker's bit pattern
 * @param uiMarkerSize overall size of the marker, counted in bits, including the border and the bit pattern
 * @param uiMask regions of the marker's bit pattern that belong to the ID (1) or not (0)
 */
UTVISION_EXPORT void computeMarkerEdgels( unsigned long long int markerCode, float markerSize, 
	std::vector< Math::Vector< float, 3 > >& points1, std::vector< Math::Vector< float, 3 > >& points2, 
	unsigned int uiCodeSize, unsigned int uiMarkerSize, unsigned long long int uiMask, bool computeInnerEdgels );

/**
 * @ingroup vision
 * Computes an alternate marker pose.
 *
 * Sometimes the pose estimation process runs into a wrong local minimum. This function 
 * computes the pose that roughly corresponds to the other minimum. The resulting pose is
 * meant to be refined by the iterative pose estimation again. If the residual is smaller 
 * than for the original pose, use this one.
 *
 * @param p pose computed by the first try
 * @return alternate marker pose
 */
UTVISION_EXPORT Math::Pose alternateMarkerPose( const Math::Pose& p );

/**
 * \internal
 * Given two points, find some accurate points on the line between the points with subpixel precision.
 *
 * Note: integer coordinates lie at pixel centers.
 * This function does not respect the origin flag and assumes that a y-coordinate of 0 is on top.
 *
 * @param pPoints: array where the result will be stored
 * @param pStrengths: edge strength at the point
 * @param nPoints: maximum number of points to find
 * @param pGreyImage: greyscale image in which to find points
 * @param point1, point2: two points describing the line
 * @param nSearchPixels: number of pixels around the line to search for edges
 * @param pDebugImg if not NULL, we will draw some debugging info into this image
 * @return number of detected points
 */
 
UTVISION_EXPORT int refineLinePoints( CvPoint2D32f* pPoints, int* pStrengths, int nPoints, const Image& greyImage,
	const Math::Vector< float, 2 >& point1, const Math::Vector< float, 2 >& point2, int nSearchPixels, 
	Image* pDebugImg );

} } } // namespace Ubitrack::Vision::Markers

#endif