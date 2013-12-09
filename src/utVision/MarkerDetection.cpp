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

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <boost/scoped_array.hpp>
#include <boost/foreach.hpp>
#include <opencv/cv.h>

#include <iostream>

#include <utMath/MatrixOperations.h>
#include <utMath/cast_assign.h>
#include <utCalibration/Homography.h>
#include <utCalibration/2D3DPoseEstimation.h>
#include <utCalibration/Projection.h>
#include <utVision/Colors.h>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include "MarkerDetection.h"
#include "PixelFlow.h"
#include "EdgeExtraction.h"
#include <algorithm>


// get a logger
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.MarkerTracker" ) );

// for edge tracking
//#define OPTIMIZATION_LOGGING
//static log4cpp::Category& optLogger( log4cpp::Category::getInstance( "Ubitrack.Vision.MarkerTracker.Opt" ) );

#include <utMath/LevenbergMarquardt.h>
#include <utMath/GaussNewton.h>
#include <utVision/EdgeMeasurement.h>
#include <utCalibration/Function/ProjectivePoseNormalize.h>

#ifdef HAVE_TBB
#undef HAVE_TBB
#endif

#ifdef HAVE_TBB
#include <algorithm>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
using namespace tbb;
#endif

//#define DO_TIMING

#ifdef DO_TIMING
#include <utUtil/BlockTimer.h>

static Ubitrack::Util::BlockTimer g_blockTimer1( "Vision1", "Ubitrack.Vision.MarkerDetection.Timing" );
static Ubitrack::Util::BlockTimer g_blockTimer2( "Vision2", "Ubitrack.Vision.MarkerDetection.Timing" );
static Ubitrack::Util::BlockTimer g_blockTimer3( "Marker1", "Ubitrack.Vision.MarkerDetection.Timing" );
static Ubitrack::Util::BlockTimer g_blockTimer4( "Marker2", "Ubitrack.Vision.MarkerDetection.Timing" );
//static Ubitrack::Util::BlockTimer g_blockTimer( "MarkerDetection", "Ubitrack.Vision.MarkerDetection.Timing" );
//static Ubitrack::Util::BlockTimer g_blockTimer( "MarkerDetection", "Ubitrack.Vision.MarkerDetection.Timing" );

#endif

namespace ublas = boost::numeric::ublas;


namespace Ubitrack { namespace Vision { namespace Markers {

#ifdef HAVE_TBB
void markerCalculations(CornerList &it, const Image& img, Image* pDebugImg,MarkerInfoMap& markerInfos,
	const Math::Matrix< float, 3, 3 >& K,const Math::Matrix< float, 3, 3 >& invK, unsigned int iCodeSize, unsigned int iMarkerSize, unsigned long long int uiMask, bool useInnerEdgels);
void markerCalculationsRefine(unsigned long long int markerId,  const Image& img, Vision::Image* pDebugImg,MarkerInfoMap& markerInfos, 
	const Math::Matrix< float, 3, 3 >& K,const Math::Matrix< float, 3, 3 >& invK, unsigned int iCodeSize, unsigned int iMarkerSize, unsigned long long int uiMask, bool useInnerEdgels);

class TBBMarkerCalculations {
public:
 void operator() ( const blocked_range<size_t>& r ) const {	 	 
	 for ( size_t i = r.begin(); i != r.end(); ++i ){
		CornerList cl(markerList->at(i)); 
		markerCalculations(cl, *img, pDebugImg, *markerInfos, *K, *invK, iCodeSize, iMarkerSize, uiMask, useInnerEdgels);
		 
	 }	 
}

 TBBMarkerCalculations(MarkerList *_markerList, const Image* _img, Image* _pDebugImg,MarkerInfoMap* _markerInfos,
	const Math::Matrix< float, 3, 3 >* _K,const Math::Matrix< float, 3, 3 >* _invK, unsigned int _iCodeSize, unsigned int _iMarkerSize, unsigned long long int _uiMask, bool _useInnerEdgels):
	markerList(_markerList)
	, img(_img)
	, pDebugImg(_pDebugImg)
	, markerInfos(_markerInfos)
	, K(_K)
	, invK(_invK)
	, iCodeSize(_iCodeSize)
	, iMarkerSize(_iMarkerSize)
	, uiMask(_uiMask)
	, useInnerEdgels(useInnerEdgels)
	{}
private:
	MarkerList* markerList;
	const Image* img;
	Image* pDebugImg;
	MarkerInfoMap* markerInfos;
	const Math::Matrix< float, 3, 3 >* K;
	const Math::Matrix< float, 3, 3 >* invK;
	unsigned int iCodeSize;
	unsigned int iMarkerSize;
	unsigned long long int uiMask;
	bool useInnerEdgels;
 
};


class TBBMarkerCalculationsRefine {
public:
 void operator() ( const blocked_range<size_t>& r ) const {	 
	 for ( size_t i = r.begin(); i != r.end(); ++i ){
		 markerCalculationsRefine(markerIds->at(i), *img, pDebugImg, *markerInfos, *K, *invK, iCodeSize, iMarkerSize, uiMask, useInnerEdgels);
	 }
}

 TBBMarkerCalculationsRefine(std::vector<unsigned long long int>* _markerIds, const Image* _img, Image* _pDebugImg,MarkerInfoMap* _markerInfos,
	const Math::Matrix< float, 3, 3 >* _K,const Math::Matrix< float, 3, 3 >* _invK, unsigned int _iCodeSize, unsigned int _iMarkerSize, unsigned long long int _uiMask, bool _useInnerEdgels):
	markerIds(_markerIds)	
	, img(_img)
	, pDebugImg(_pDebugImg)
	, markerInfos(_markerInfos)
	, K(_K)
	, invK(_invK)
	, iCodeSize(_iCodeSize)
	, iMarkerSize(_iMarkerSize)
	, uiMask(_uiMask)
	, useInnerEdgels(useInnerEdgels)
	{}
private:
	std::vector<unsigned long long int>* markerIds;	
	const Image* img;
	Image* pDebugImg;
	MarkerInfoMap* markerInfos;
	const Math::Matrix< float, 3, 3 >* K;
	const Math::Matrix< float, 3, 3 >* invK;
	unsigned int iCodeSize;
	unsigned int iMarkerSize;
	unsigned long long int uiMask;
	bool useInnerEdgels;
 
};


#endif



// some constants for tuning

/** width of area in which to search for edges, as percentage of marker extension */
const float g_fEdgeSearchArea = 0.12f;

/** number of tracked points on an edge */
const int g_nEdgePoints = 8;

/** number of tracked points on an edge necessary for line fit */
const int g_nFitMinEdgePoints = 4;

/** minimal gradient response for edges */
int g_nMinimalEdgeStrength = 40;

/** maximal number of non-black pixels on marker border */
const int g_nMaxWhiteBorderPixels = 4;

/** 2D marker corner points in counter-clockwise order */
const float std2dPoints[ 4 ][ 2 ] = 
	{ { -1.0f, 1.0f }, { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f } };


// forward declaration of private functions
static void drawCube( Vision::Image& img, const Math::Pose& pose, const Math::Matrix< float, 3, 3 >& K, double markerSize, CvScalar color, double error = -1.0 );
bool checkRefinedMarker( const Math::Matrix< float, 3, 3 >& K ,Math::Pose checkPose, MarkerInfo& info, const Image& img, unsigned long long int nCode, Image* pDebugImg, unsigned int iMarkerSize, unsigned int iCodeSize );


#ifdef HAVE_LAPACK

Math::Pose edgeBasedRefinement (Math::Pose initialPose, unsigned long long int nCode, unsigned int nCodeSize, unsigned int nMarkerSize, MarkerInfo& info, 
	Math::Matrix< float, 3, 3 > K,const Image& img, CornerList it, Image* pDebugImg, bool bRefine, unsigned long long int uiMask, bool computeInnerEdgels )
{
	int sum = 0;
	for(int i = 1; i< img.imageSize; i++)
		sum += img.imageData[i];

	Math::Pose pose( initialPose );

	// create edgel lists
	std::vector< Math::Vector< float, 3 > > edgels1;
	std::vector< Math::Vector< float, 3 > > edgels2;
	computeMarkerEdgels( nCode, info.fSize, edgels1, edgels2, nCodeSize, nMarkerSize, uiMask, computeInnerEdgels );

	// optimize pose
	Math::Vector< float, 7 > poseVector;
	pose.toVector( poseVector );

	// compute scaling of normals as covariance of the marker corners
	Math::Matrix< float, 2, 2 > cornerCov( Math::Matrix< float, 2, 2 >::zeros( ) );
	Math::Vector< float, 2 > cornerAvg( Math::Vector< float, 2 >::zeros() );
	for ( unsigned int i = 0; i < 4; i++ )
	{
		cornerCov += ublas::outer_prod( (it)[ i ], (it)[ i ] );
		cornerAvg += (it)[ i ];
	}	
	cornerCov /= 4;
	cornerAvg /= 4;
	cornerCov -= ublas::outer_prod( cornerAvg, cornerAvg );

	// scale covariance matrix
	cornerCov /= ( nMarkerSize * nMarkerSize );
	if ( info.bEnablePixelFlow )
		cornerCov *= ( 1.5f * 1.5f );
	else
		cornerCov *= ( 2.0f * 2.0f );
	LOG4CPP_DEBUG( logger, "cornerCov = " << cornerCov );

	// Optimization iterations on edgels
	EdgeListMeasurementFunction< float, FindEdgePositiveMaximum > edgeMF( 
		edgels1, edgels2, K, cornerCov, img, pDebugImg );
	
	try
	{
		info.fResidual = Math::weightedLevenbergMarquardt( 
			edgeMF,
			poseVector,
			Math::Vector< float >::zeros( edgels1.size() ),
			// Max. 6 iterations, precision 1e-4
			Math::OptTerminate( 6, 1e-4 ),
			Calibration::Function::ProjectivePoseNormalize(),
			edgeMF
		);
		
		info.nVisibility = int( edgeMF.getGoodEdgelsPercentage() * 100 );
		pose = Math::Pose::fromVector( poseVector );
		
		info.fResidual = edgeMF.getBoundedResidual();
	}
	catch ( Ubitrack::Util::Exception )
	{	
		info.nVisibility = 0;
	}

	return pose;
}


void updateCorners( Math::Matrix< double, 3, 3 > K, Math::Pose pose, MarkerInfo& info )
{
	//TODO Allow for templated pose type to get rid of compiler warning due to float and double types.

	for ( unsigned i = 0; i < 4; i++ )
	{
		Math::Vector< float, 3 > p2D = 
			ublas::prod( K, pose * Math::Vector< float, 3 >( 0.5f * (float)info.fSize * 
			Math::Vector< float, 3 >( std2dPoints[ i ][ 0 ], std2dPoints[ i ][ 1 ], 0.0 ) ) );
		info.corners[ i ]( 0 ) = p2D( 0 ) / p2D( 2 );
		info.corners[ i ]( 1 ) = p2D( 1 ) / p2D( 2 );
	}
}


void calculateMarkerBoundingRectangle( const Vision::Image& img, CornerList corners, Math::Vector< int, 2 >& topLeft, Math::Vector< int, 2 >& botRight, double percentage)
{
	int maxTemp = int(corners[0](0));
	int minTemp = int(corners[0](0));
			
	for(int i = 1; i < 4; i++)
	{
		if (maxTemp < int(corners[i](0)))
			maxTemp = int(corners[i](0));
	
		if (minTemp > int(corners[i](0)))
			minTemp = int(corners[i](0));
	}

	topLeft(0) = minTemp;
	botRight(0) = maxTemp;
						
	maxTemp = int(corners[0](1));
	minTemp = int(corners[0](1));
						
	for(int i = 1; i < 4; i++)
	{
		if (maxTemp < int(corners[i](1)))
			maxTemp = int(corners[i](1));

		if (minTemp > int(corners[i](1)))
			minTemp = int(corners[i](1));
	}

	topLeft(1) = minTemp;
	botRight(1) = maxTemp;

	//calculating the incrise of the buffer
	int incrLength = int((botRight(0) - topLeft(0))*percentage/2);
	int incrHeight = int((botRight(1) - topLeft(1))*percentage/2);
	
	//calculating the incrise of x buffer right and then from left
	botRight(0) = std::min( botRight(0) + incrLength, img.width );
	topLeft(0) = std::max( topLeft(0) - incrLength, 0 );

	//calculating the incrize of the y buffer from top and below
	botRight(1) = std::min( botRight(1) + incrHeight, img.height );
	topLeft(1) = std::max( topLeft(1) - incrHeight, 0 );
}


void markerCalculations(CornerList &it, const Image& img, Image* pDebugImg,MarkerInfoMap& markerInfos,
	const Math::Matrix< float, 3, 3 >& K,const Math::Matrix< float, 3, 3 >& invK, unsigned int iCodeSize, unsigned int iMarkerSize, unsigned long long int uiMask, bool useInnerEdgels) {
	// refine corner positions
		const bool bRefine = false;
			if ( refineCorners( img, it ) ) //, pDebugImg ) )
			{
				// if image is top-down, exchange point 2 and 4 to assure counter-clock-wise order for squareHomography
				if ( !img.origin )
				{
					Math::Vector< float, 2 > temp( (it)[ 3 ] );
					(it)[ 3 ] = (it)[ 1 ];
					(it)[ 1 ] = temp;
				}

				// compute homography
				Math::Matrix< float, 3, 3 > H( Calibration::squareHomography( it ) );

				// get marker image & decode
				boost::shared_ptr< Image > pMarker( getMarkerImage( img, H, iMarkerSize ) );
				unsigned long long int nCode = readCode( *pMarker, iCodeSize, uiMask );
				/*
				// show marker image in debug mode
				if ( pDebugImg )
				{
					// Maybe make configurable in the future
					int scale = 5;
				
					int markersPerCol = img.height / ( pMarker->height * scale );
					int xStart = ( iShownMarker / markersPerCol ) * ( pMarker->height * scale );
					int yStart = ( iShownMarker % markersPerCol ) * ( pMarker->height * scale );
					iShownMarker++;
					for ( unsigned int x = 0; x < iMarkerSize; x++ ) {
						for ( unsigned int y = 0; y < iMarkerSize; y++ )
						{
							unsigned c = ( (unsigned char*)pMarker->imageData )[ y * pMarker->widthStep + x ];
							cvRectangle( *pDebugImg, 
								cvPoint( xStart + scale * x, yStart + scale * y ), 
								cvPoint( xStart + scale * (x+1), yStart + scale * (y+1) ),
								nCode ? CV_RGB( c, c, c ) : CV_RGB( c, (c*c)>>8, (c*c)>>8 ), CV_FILLED );
						}
                    }
                    for ( size_t i = 0; i < it->size(); ++i) {
                        cvCircle( *pDebugImg, cvPoint( cvRound( it->at( i )(0) * 16 ), cvRound( it->at( i )(1) * 16 ) ),
                            cvRound( pDebugImg->width / 500.0 * 16 ), CV_RGB( 255, 127, 39 ), -1, CV_AA, 4 );
                    }
                }*/

				// normalize marker code
				int nRotate = 0;
				nCode = normalizeCode( nCode, iCodeSize, nRotate );

				if ( nCode )
				{ 
                    LOG4CPP_TRACE( logger, "found marker with code: 0x" << std::hex << nCode ); 
                    // Draw marker contours for found marker
                    if ( pDebugImg )
                    {
                        for ( size_t i = 0; i < it.size(); ++i) {
                        cvCircle( *pDebugImg, cvPoint( cvRound( it.at( i )(0) * 16 ), cvRound( it.at( i )(1) * 16 ) ),
				            cvRound( pDebugImg->width / 500.0 * 16 ), CV_RGB( 0, 255, 255 ), -1, CV_AA, 4 );
                        }
                    }
                }

				if ( nCode && ( markerInfos.find( nCode ) != markerInfos.end() || markerInfos.find( 0 ) != markerInfos.end() ) )
				{
					// find the info struct for this marker
					if ( markerInfos.find( nCode ) == markerInfos.end() )
						markerInfos[ nCode ] = markerInfos[ 0 ];

					MarkerInfo& info( markerInfos[ nCode ] );
					info.found = info.EFullScanFound;
						
					// create corner list
					info.corners.resize( 4 );
					for ( unsigned i = 0; i < 4; i++ )
						Math::vector_cast_assign( info.corners[ i ], Math::Vector< double, 2 >( (it)[ ( i + nRotate ) % 4 ]( 0 ), (it)[ ( i + nRotate ) % 4 ]( 1 ) ) );

					// origin correction
					if ( !img.origin )
						for ( unsigned i = 0; i < 4; i++ )
							info.corners[ i ]( 1 ) = img.height - 1 - info.corners[ i ]( 1 );

                    if ( pDebugImg )
                    {
                        for ( size_t i = 0; i < it.size(); ++i) {
                        cvCircle( *pDebugImg, cvPoint( cvRound( it.at( i )(0) * 16 ), cvRound( it.at( i )(1) * 16 ) ),
				            cvRound( pDebugImg->width / 500.0 * 16 ), CV_RGB( 0, 255, 0 ), -1, CV_AA, 4 );
                        }
                    }

					if ( info.refinement >= MarkerInfo::EInitialPose )
					{
						// scale homography with marker size
						ublas::column( H, 2 ) *= info.fSize;						
						LOG4CPP_DEBUG( logger, "Homography: " << H  );
						// compute pose
						Math::Pose initialPose( Calibration::poseFromHomography( H, invK ) );						
						LOG4CPP_DEBUG( logger, "initialPose: " << initialPose  );
						// rotate pose to account for different order of corner points
						const double fSqrtHalf = 0.70710678118654752440084436210485;
						static const double csMap[ 4 ][ 2 ] =
							{ { 1.0f, 0.0f }, { fSqrtHalf, fSqrtHalf }, { 0.0f, 1.0f }, { fSqrtHalf, -fSqrtHalf } };
						initialPose = Math::Pose( initialPose.rotation() * Math::Quaternion( 0.0, 0.0, csMap[ nRotate ][ 1 ], csMap[ nRotate ][ 0 ] ), initialPose.translation() );
						LOG4CPP_DEBUG( logger, "initial pose: " << initialPose << ", marker size: " << info.fSize );

						// create list of 3D marker points
						std::vector< Math::Vector< float, 3 > > p3D;
							
						const float fF = 0.5f * info.fSize;
						for ( unsigned i = 0; i < 4; i++ )
						{
							unsigned j = ( i + 4 - nRotate ) % 4;
							p3D.push_back( Math::Vector< float, 3 >( fF * std2dPoints[ j ][ 0 ], fF * std2dPoints[ j ][ 1 ], 0.0f ) );
						}

						Math::Pose pose( initialPose );

						// perform some LM iterations on the edge points to get a more stable initialization
						double optRes = Calibration::optimizePose( pose, it, p3D, K, 4 );
							
						// try optimization with the rotation from the last pose
						if ( info.bUseInitialPose )
						{
							Math::Pose testPose = Math::Pose( info.pose.rotation(), initialPose.translation() );
							double testOptRes = Calibration::optimizePose( testPose, it, p3D, K, 4 );
							if ( testOptRes < optRes * 9 ) // prefer last pose rotation
								pose = testPose;
						}
							
						// refine the pose
						if ( info.refinement >= MarkerInfo::EEdgeRefinedPose )
						{
							pose = edgeBasedRefinement( pose, nCode, iCodeSize, iMarkerSize, info, K, img, it, pDebugImg, bRefine, uiMask, useInnerEdgels );

							// Compute refined corners, based on the refined pose
							// This is only necessary here and not with optimizePose() because here, also the inner edgelets are considered for pose estimation
							LOG4CPP_TRACE( logger, "Unrefined corner positions: [" << info.corners[0] << ", " << info.corners[1] << ", " << info.corners[2] << ", " << info.corners[3] << "]" );

							updateCorners( K, pose, info );
							// Origin correction
							if ( !img.origin )
								for ( unsigned i = 0; i < 4; i++ )
									info.corners[ i ]( 1 ) = img.height - 1 - info.corners[ i ]( 1 );

							LOG4CPP_TRACE( logger, "Refined corner positions: [" << info.corners[0] << ", " << info.corners[1] << ", " << info.corners[2] << ", " << info.corners[3] << "]" );
						}
						else
						{
							// more iterations on the edge points
							Calibration::optimizePose( pose, it, p3D, K, 7 );
						}
						
						// remember pose
						LOG4CPP_DEBUG( logger, "optimized pose: " << pose << ", marker size: " << info.fSize );
									
						// also send ErrorPose if anybody is connected
						if ( info.bCalculateCovariance )
							// FIXME: make pixel variance configurable
							info.covariance = Calibration::singleCameraPoseError( pose, p3D, K, 0.04f * 0.04f );
						
						// in debug mode, draw a nice cube onto the marker
						if ( pDebugImg ) {
							drawCube( *pDebugImg, pose, K, info.fSize, CV_RGB( 255, 255, 0 ), info.fResidual );
						}
									
						// check whether need to switch
						if ( info.nPrevPoseValidator == 0 )
						{	
							info.prevPose = pose;
							info.pose = pose;
						}
						else
						{	
							info.prevPose = info.pose;
							info.pose = pose;
						}
						info.nPrevPoseValidator++;
						
						// calculate projection buffer
						Math::Vector< int, 2 > topLeft;
						Math::Vector< int, 2 > botRight;
						
						calculateMarkerBoundingRectangle( img, info.corners, topLeft, botRight, 0.2 );

						// Peter Keitler: This call crashes on some images or image sequences and therefore has been commented out!
						//info.pFlow.calcProjectionBuffer( img, topLeft, botRight );
					}
				}
				else
				{ LOG4CPP_TRACE( logger, "no marker info found" ); }
			}
}

void markerCalculationsRefine(unsigned long long int markerId,  const Image& img, Vision::Image* pDebugImg,MarkerInfoMap& markerInfos, 
	const Math::Matrix< float, 3, 3 >& K,const Math::Matrix< float, 3, 3 >& invK, unsigned int iCodeSize, unsigned int iMarkerSize, unsigned long long int uiMask, bool useInnerEdgels){
		int nDiff, nVis;
		float fRes;
		bool bDraw = true;				
		Math::Pose pose;
		Math::Pose aprPose;
		Math::Pose flipPose;
		Math::Quaternion quat;
		Math::Vector< double, 3 > oldTrans, newTrans;
		Math::Vector< int, 2 > res;
		MarkerInfo info(markerInfos[markerId]);
			
			// Calculate the approximate pose via pixel flow
			if ( info.bEnablePixelFlow )
			{	
				info.pFlow.computeFlow( img, res, nDiff, pDebugImg );

				quat = info.pose.rotation();
				oldTrans = info.pose.translation();
				
				// calculating the translations for new pose
				newTrans[2] = oldTrans[2];
				newTrans[0] = oldTrans[0] + res(0)*oldTrans[2]/K(0,0)*K(2,2);
				newTrans[1] = oldTrans[1] + res(1)*oldTrans[2]/K(1,1)*K(2,2);

				// creation of new aproximate pose
				LOG4CPP_DEBUG( logger, "Pixel flow: " << res );
				LOG4CPP_DEBUG( logger, "old: " << oldTrans << ", new: " << newTrans );
				aprPose = Math::Pose( quat, newTrans );
			}
			else if ( info.bEnableInterpolation )
				// interpolating the current pose with previous one
				aprPose = Math::linearInterpolate( info.prevPose, info.pose, 2 );
			else
				aprPose = info.pose;

			// refine the pose
			try
			{
				pose = edgeBasedRefinement( aprPose, markerId, iCodeSize, iMarkerSize, info, K, img, info.corners, pDebugImg, true, uiMask, useInnerEdgels );
				fRes = info.fResidual;
				nVis = info.nVisibility;
			}
			catch ( std::exception& )
			{
				info.nVisibility = 0;
			}

			if ( info.bEnableFlipCheck && info.nVisibility > 0 ) // should be same as bUseInitial Pose?
			{
				flipPose = alternateMarkerPose( aprPose );
				flipPose = edgeBasedRefinement( flipPose, markerId, iCodeSize, iMarkerSize, info, K, img, info.corners, pDebugImg, true, uiMask, useInnerEdgels );
				
				LOG4CPP_DEBUG( logger, "old res: " << fRes << ", flipped res: " << info.fResidual );
				LOG4CPP_DEBUG( logger, "old vis: " << nVis << ", flipped vis: " << info.nVisibility );
				
				if ( info.fResidual < fRes * 0.6 )
				//if ( info.nVisibility > nVis + 10 )
					pose = flipPose;
				else
				{
					info.fResidual = fRes;
					info.nVisibility = nVis;
				}
			}
				
			if ( !checkRefinedMarker( K, pose, info, img, markerId, pDebugImg, iMarkerSize, iCodeSize ) )
				info.nVisibility = 0;
			else
				info.found = info.ERefinementFound;

			if ( info.nVisibility < 40 ) // TODO: magic number
			{	
				info.found = info.bEnablePixelFlow ? MarkerInfo::EPixelFlowFound : MarkerInfo::ENotFound;
				bDraw = false;
				pose = aprPose;
			}
			
			// update corners list
			updateCorners( K, pose, info );

			// calculating the projection buffer
			Math::Vector< int, 2 > topLeft;
			Math::Vector< int, 2 > botRight;
			
			// additional steps needed for pixel flow
			if ( info.bEnablePixelFlow )
			{
				calculateMarkerBoundingRectangle(img,info.corners, topLeft, botRight, 0.2);
				info.pFlow.calcProjectionBuffer(img,topLeft,botRight);
			}

			// remember pose
			LOG4CPP_DEBUG( logger, "optimized pose: " << pose );
			info.prevPose = info.pose;
			info.pose = pose;
			
			// in debug mode, draw a nice cube onto the marker
			if ( pDebugImg && bDraw )
				drawCube( *pDebugImg, pose, K, info.fSize, CV_RGB( 0, 0, 255 ) );
}

void detectMarkers( const Image& img, MarkerInfoMap& markerInfos, 
	const Math::Matrix< float, 3, 3 >& _K,  Image* pDebugImg, bool bRefine, 
	unsigned int iCodeSize, unsigned int iMarkerSize, unsigned long long int uiMask, bool useInnerEdgels )
{
	assert( (iMarkerSize - iCodeSize) / 2.0 == 1.0 || (iMarkerSize - iCodeSize) / 2.0 == 2.0 );
	
	
	
	Math::Matrix< float, 3, 3 > K( _K );
		
	// flip image coordinates if origin==0 
	LOG4CPP_DEBUG( logger, "image origin flag = " << img.origin );
	Calibration::correctOrigin( K, img.origin, img.height );

	// compute inverse camera matrix
	Math::Matrix< float, 3, 3 > invK( invert_matrix( K ) );
	
	// initialize found state of markerInfos
	BOOST_FOREACH( MarkerInfoMap::value_type& mapEl, markerInfos )
		mapEl.second.found = MarkerInfo::ENotFound;
	
	if ( !bRefine )
	{
		LOG4CPP_TRACE( logger, "detectMarkers(): detection/refinement" );	

		// threshold image
		Image thresholded( img.width, img.height, 1 );
		{
			#ifdef DO_TIMING
			UBITRACK_TIME( g_blockTimer1 );
			#endif
			// The source image is copied as OpenCV from beta 5 on destroys it
			cvAdaptiveThreshold( *img.Clone(), thresholded, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY,
			( img.height / 48 ) | 1 , 4.0 );
		}

		
		MarkerList markers;
		{
			#ifdef DO_TIMING 
			UBITRACK_TIME( g_blockTimer2 );
			#endif
			// find markers in the image
			markers = findQuadrangles( thresholded, pDebugImg);
		}
		{
			#ifdef DO_TIMING
			UBITRACK_TIME( g_blockTimer3 );
			#endif
#ifdef HAVE_TBB		
			parallel_for(blocked_range<size_t>(0, markers.size(), 4 ), TBBMarkerCalculations(&markers, &img, pDebugImg, &markerInfos, &K, &invK, iCodeSize, iMarkerSize, uiMask, useInnerEdgels ) );
#else
			int iShownMarker = 0;
			for ( MarkerList::iterator it = markers.begin(); it != markers.end(); it++ )
			{
				markerCalculations(*it, img, pDebugImg, markerInfos, K, invK, iCodeSize, iMarkerSize, uiMask, useInnerEdgels);
			}
#endif
		}
	}
	else
	{	
		LOG4CPP_TRACE( logger, "detectMarkers(): refinement only" );	
		#ifdef DO_TIMING
		UBITRACK_TIME( g_blockTimer4 );
		#endif
		// declaration of variables
		std::map<unsigned long long int, MarkerInfo>::iterator it;
#ifdef HAVE_TBB		
		std::vector<unsigned long long int> _markerIds;
		for ( it = markerInfos.begin(); it != markerInfos.end(); it++ )
		{	
			_markerIds.push_back(it->first);			
		}
		parallel_for(blocked_range<size_t>(0, _markerIds.size(), 4 ), TBBMarkerCalculationsRefine(&_markerIds, &img, pDebugImg, &markerInfos, &K, &invK, iCodeSize, iMarkerSize, uiMask, useInnerEdgels ));
#else				
		for ( it = markerInfos.begin(); it != markerInfos.end(); it++ )
		{	
			markerCalculationsRefine(it->first, img, pDebugImg, markerInfos, K, invK, iCodeSize, iMarkerSize, uiMask, useInnerEdgels);
		}
#endif
	}
	#ifdef DO_TIMING	
	LOG4CPP_INFO( logger, g_blockTimer1);				
	LOG4CPP_INFO( logger, g_blockTimer2);				
	LOG4CPP_INFO( logger, g_blockTimer3);				
	LOG4CPP_INFO( logger, g_blockTimer4);				
	
	#endif
}
#endif // HAVE_LAPACK


/** extracts the marker from the image and checks how many pixels are correct */
bool checkRefinedMarker(const Math::Matrix< float, 3, 3 >& K, Math::Pose checkPose, MarkerInfo& info, const Image& img, unsigned long long int nCode, Image* pDebugImg, unsigned int iMarkerSize, unsigned int iCodeSize )
{
	Math::Matrix< double, 3, 4 > Rt( checkPose );
	Math::Matrix< double, 3, 4 > KRt( ublas::prod( K, Rt ) );

	// scale to make marker coordinates from -1 to +1
	Math::Matrix< double, 4, 3 > Sr( Math::Matrix< double, 4, 3 >::zeros( ) );
	Sr( 0, 0 ) = Sr( 1, 1 ) = info.fSize;
	Sr( 3, 2 ) = 1;
	Math::Matrix< double, 3, 3 > h( ublas::prod( KRt, Sr ) );

	Math::Vector< double, 3 > test = ublas::prod( h, Math::Vector< double, 3 >( -1, -1, 1 ) );
	test /= test( 2 );

	Math::Vector< double, 3 > test2 = ublas::prod( h, Math::Vector< double, 3 >( 1, 1, 1 ) );
	test2 /= test2( 2 );

	boost::shared_ptr< Image > pMarkerImg = getMarkerImage(img, h, iMarkerSize);
	const unsigned char* pData = reinterpret_cast< const unsigned char* >( pMarkerImg->imageData ); 

	// show marker image in debug mode
	if ( pDebugImg )
	{
		int xStart = img.width - 1 - ( pMarkerImg->height * (iMarkerSize-1) );
		int yStart = img.height - 1 - ( pMarkerImg->height * (iMarkerSize-1) );
		for ( unsigned int x = 0; x < iMarkerSize; x++ )
			for ( unsigned int y = 0; y < iMarkerSize; y++ )
			{
				unsigned c = ( (unsigned char*)pMarkerImg->imageData )[ y * pMarkerImg->widthStep + x ];
				cvRectangle( *pDebugImg, 
					cvPoint( xStart + (iMarkerSize-1) * x, yStart + (iMarkerSize-1) * y ), 
					cvPoint( xStart + (iMarkerSize-1) * (x+1), yStart + (iMarkerSize-1) * (y+1) ),
					CV_RGB( (c*c)>>8, (c*c)>>8, c ), CV_FILLED );
			}
	}

	int avg = 0;
	int whiteCellsBorder =0;
	
	// compute average pixel
	for(int x  = 0; x < pMarkerImg->width; x++)
	{
		for(int y = 0; y < pMarkerImg->height; y++)
		{
			int t = pData[y*pMarkerImg->widthStep + x];
			avg += t;  
		}
	}
	avg = int(avg/(pMarkerImg->width * pMarkerImg->height));
	
	// find the black and white points
	unsigned int nCorrectBoxes = 0;
	unsigned int nBorderSize = (iMarkerSize - iCodeSize) / 2;
	for ( unsigned int y = nBorderSize; y < iMarkerSize - nBorderSize; y++ )	
		for ( unsigned int x = nBorderSize; x < iMarkerSize - nBorderSize; x++ )
		{
			if ( nCode & ( ((unsigned long long int)1) << ( ( iMarkerSize - 2*nBorderSize - y ) * ( iMarkerSize - 2*nBorderSize ) + ( iMarkerSize - 2*nBorderSize - x ) ) ))
			{
				if (pData[ y * pMarkerImg->widthStep + x ] <= avg )
					nCorrectBoxes++;
			}
			else
			{
				if (pData[ y * pMarkerImg->widthStep + x ] > avg )
					nCorrectBoxes++;
			}
		}

	for (unsigned int x = 0; x < iMarkerSize ; x ++)
		for(unsigned int y = 0; y < iMarkerSize; y++)
		{
			if( ( 0 <= x && x < nBorderSize || (iMarkerSize-nBorderSize) <= x && x < iMarkerSize
					|| 0 <= y && y < nBorderSize || (iMarkerSize-nBorderSize) <= y && y < iMarkerSize) 
					&& pData[ y * pMarkerImg->widthStep + x ] <= avg )
				nCorrectBoxes++;
		}	

	if ( nCorrectBoxes < iMarkerSize*iMarkerSize ) {
		LOG4CPP_INFO( logger, "Wrong boxes detected" );
		return false;
	}
	else
		return true;
}


MarkerList findQuadrangles( Image& img, Image * pDbgImg, cv::Point offset)
{
	// initialize return value
	MarkerList ret;

	// create memory storage that will contain all the dynamic data
	CvMemStorage* pStorage = cvCreateMemStorage( 0 );

	// Create dynamic structure and sequence.
	CvSeq* pContours = cvCreateSeq( CV_SEQ_ELTYPE_POINT, sizeof( CvSeq ), sizeof( CvPoint ), pStorage );

	// Find all contours.
	cvFindContours( img, pStorage, &pContours, sizeof( CvContour ), 
		CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, offset );

	// approximate all contours by polygons and check size and corner number
	for ( CvSeq* pContour = pContours; pContour; pContour = pContour->h_next ) 
	{
		CvRect boundingRect = cvBoundingRect( pContour );
		float fDiameter = sqrtf( static_cast< float >( boundingRect.height * boundingRect.height + boundingRect.width * boundingRect.width ) );

		// discard too small contours
		if ( fDiameter < 30 )
			continue;

		// discard too large contours
		if ( fDiameter > 1000 )
			continue;
	
		// approximate contour by few corners
		CvSeq* pApproxChain = cvApproxPoly( pContour, sizeof( CvContour ), pStorage,
			CV_POLY_APPROX_DP, fDiameter * 0.03f + 2.0f, 0 );

		int nCorners = pApproxChain->total; // This is number point in contour

		// discard non-quadrangles
		if ( nCorners != 4 )
			continue;

		// Alloc memory for contour point set. 
		boost::scoped_array< CvPoint > pPointArray( new CvPoint[ nCorners ] );

		// Get contour point set and append to list
		cvCvtSeqToArray( pApproxChain, pPointArray.get(), CV_WHOLE_SEQ );

		CornerList rect( 4 );
		for ( int i = 0; i < 4; i++ ) 
		{
			rect[ i ][ 0 ] = static_cast< float >( pPointArray[ i ].x );
			rect[ i ][ 1 ] = static_cast< float >( pPointArray[ i ].y );
		}

		if (pDbgImg) {
			cvDrawContours( pDbgImg, pApproxChain, CV_RGB ( 255, 0, 255), CV_RGB( 255, 0, 255 ), -1);
		}

		ret.push_back( rect );
	}

	// cleanup	     
	cvReleaseMemStorage( &pStorage );

	return ret;
}


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
int refineLinePoints( CvPoint2D32f* pPoints, int* pStrengths, int nPoints, const Image& greyImage,
	const Math::Vector< float, 2 >& point1, const Math::Vector< float, 2 >& point2, int nSearchPixels, 
	Image* pDebugImg = NULL )
{
	int iPoint = 0;

	// compute normalized direction vector of input line
	Math::Vector< float, 2 > direction( point2 - point1 );
	float fLength = boost::numeric::ublas::norm_2( direction );
	direction /= fLength;

	// compute normal of input line
	Math::Vector< float, 2 > normal( -direction( 1 ), direction( 0 ) );

	// create buffer
	boost::scoped_array< int > pBuffer( new int[ nSearchPixels ] );

	// refine at some points on input line
	for ( int i = 0; i < nPoints; i++ )
	{
		// sample image perpendicular to input line with sobel operator
		float fPos = ( i + 2 ) * fLength / ( nPoints + 3 );
		Math::Vector< float, 2 > startPoint( point1 + fPos * direction );

		int nMax;
		float fMaxSubPix = findEdge< FindEdgePositiveMaximum, ExtractLineSobel>
		//float fMaxSubPix = findEdge< FindEdgePositiveMaximum, ExtractLineSimpleGradient>
			( greyImage, startPoint, normal, nSearchPixels >> 1, nMax );

		if ( nMax >= g_nMinimalEdgeStrength )
		{
			// compute subpixel edge point
			pPoints[ iPoint ].x = startPoint( 0 ) + fMaxSubPix * normal( 0 );
			pPoints[ iPoint ].y = startPoint( 1 ) + fMaxSubPix * normal( 1 );
			pStrengths[ iPoint ] = nMax;

			iPoint++;
		}

		// debugging
		if ( pDebugImg )
			cvCircle( *pDebugImg, cvPoint( cvRound( pPoints[ iPoint ].x * 16 ), cvRound( pPoints[ iPoint ].y * 16 ) ),
				cvRound( pDebugImg->width / 1600.0 * 16 ), nMax < g_nMinimalEdgeStrength ? CV_RGB( 255, 0, 0 ) : CV_RGB( 0, 255, 0 ), -1, CV_AA, 4 );
	}

	return iPoint;
}


bool refineCorners( const Image& img, CornerList& corners, Image* pDebugImg )
{
	int nAvgEdgeStrength = 0;

	// compute bounding box
	CvPoint2D32f boundMin = cvPoint2D32f( corners[ 0 ][ 0 ], corners[ 0 ][ 1 ] );
	CvPoint2D32f boundMax = cvPoint2D32f( corners[ 0 ][ 0 ], corners[ 0 ][ 1 ] );
	for ( unsigned i = 1; i < corners.size(); i++ )
	{
		boundMin.x = std::min( boundMin.x, corners[ i ][ 0 ] );
		boundMin.y = std::min( boundMin.y, corners[ i ][ 1 ] );
		boundMax.x = std::max( boundMax.x, corners[ i ][ 0 ] );
		boundMax.y = std::max( boundMax.y, corners[ i ][ 1 ] );
	}

	// localize edges
	boost::scoped_array< float > lineParams( new float[ 4 * corners.size() ] );
	for ( unsigned i = 0; i < corners.size(); i++ )
	{
		unsigned iNext = ( i + 1 ) % corners.size();
		int nSearchPixels;
		if ( fabsf( corners[ i ][ 0 ] - corners[ iNext ][ 0 ] ) >
			fabsf( corners[ i ][ 1 ] - corners[ iNext ][ 1 ] ) )
			nSearchPixels = static_cast< int >( ( boundMax.y - boundMin.y ) * g_fEdgeSearchArea + 2 );
		else
			nSearchPixels = static_cast< int >( ( boundMax.x - boundMin.x ) * g_fEdgeSearchArea + 2 );

		// make sure this is an odd number
		nSearchPixels |= 1;

		// get some subpixel points along edge
		CvPoint2D32f points[ g_nEdgePoints ];
		int strengths[ g_nEdgePoints ];
		int nPoints = refineLinePoints( points, strengths, g_nEdgePoints, img,
			corners[ i ], corners[ iNext ],	nSearchPixels, pDebugImg );

		// update average edge strength (not detected edges get 0 weight)
		int nStrengthSum = 0;
		for ( int j = 0; j < nPoints; j++ )
			nStrengthSum += strengths[ j ];
		nAvgEdgeStrength += nStrengthSum / g_nEdgePoints;

		// fit line trough these points
		if ( nPoints >= g_nFitMinEdgePoints )
		{
			// TODO: here one could experiment with different fitting distances to make this more robust
			CvMat mat = cvMat( 1, nPoints, CV_32FC2, points );
			cvFitLine( &mat, CV_DIST_L2, 0, 0.01, 0.01, &lineParams[ 4 * i ] );
		}
		else
		{
			// not enough edge points detected -> do not fit to points
			lineParams[ 4 * i + 0 ] = corners[ i ][ 0 ] - corners[ iNext ][ 0 ];
			lineParams[ 4 * i + 1 ] = corners[ i ][ 1 ] - corners[ iNext ][ 1 ];
			lineParams[ 4 * i + 2 ] = corners[ i ][ 0 ];
			lineParams[ 4 * i + 3 ] = corners[ i ][ 1 ];
		}

		// draw refined lines and corners into the image -- is rather annoying
		if ( pDebugImg && false )
			cvLine( *pDebugImg, 
				cvPoint( cvRound( lineParams[ 4 * i + 2 ] + 100 * lineParams[ 4 * i + 0 ] ), cvRound( lineParams[ 4 * i + 3 ] + 100 * lineParams[ 4 * i + 1 ] ) ), 
				cvPoint( cvRound( lineParams[ 4 * i + 2 ] - 100 * lineParams[ 4 * i + 0 ] ), cvRound( lineParams[ 4 * i + 3 ] - 100 * lineParams[ 4 * i + 1 ] ) ), 
				CV_RGB( 0, 255, 255 ), 1, 4, 0 );
	}
		
	// now recompute corners by intersecting the edges
	for ( unsigned i = 0; i < corners.size(); i++ )
	{
		unsigned iNext = ( i + 1 ) % corners.size();
		float* pLine1 = &lineParams[ 4 * i ];
		float* pLine2 = &lineParams[ 4 * iNext ];

		float a = pLine1[ 0 ] * ( pLine2[ 3 ] - pLine1[ 3 ] ) - 
			pLine1[ 1 ] * ( pLine2[ 2 ] - pLine1[ 2 ] );
		float b = pLine2[ 0 ] * pLine1[ 1 ] - pLine1[ 0 ] * pLine2[ 1 ];
		
		// check if parallel
		if ( fabsf( a ) < fabsf( b * 10e6f ) )
		{
			corners[ iNext ][ 0 ] = pLine2[ 0 ] * a / b + pLine2[ 2 ];
			corners[ iNext ][ 1 ] = pLine2[ 1 ] * a / b + pLine2[ 3 ];
		}
	}
	
	if ( pDebugImg )
	{
		// draw corners as circles
		for ( unsigned i = 0; i < corners.size(); i++ ) 
			cvCircle( *pDebugImg, cvPoint( cvRound( corners[ i ][ 0 ] * 16 ), cvRound( corners[ i ][ 1 ] * 16 ) ),
				cvRound( 1.8f * 16 ), CV_RGB( i*80, 0, 0 ), -1, CV_AA, 4 );
	}

	return ( nAvgEdgeStrength / 4 ) > g_nMinimalEdgeStrength;
}
 

boost::shared_ptr< Image > getMarkerImage( const Image& img, const Math::Matrix< float, 3, 3 > homography, int nSize )
	{
	// compute a suitable scaling matrix that maps 
	// for x: -0.5 -> -1 and (nSize-0.5) -> +1
	// for y: -0.5 -> +1 and (nSize-0.5) -> -1
	Math::Matrix< float, 3, 3 > hScale;
	
	float m = 1.0f / nSize;
	hScale( 0, 0 ) = m;
	hScale( 0, 1 ) = 0.0f;
	hScale( 0, 2 ) = 0.5f * m - 0.5f;
	hScale( 1, 0 ) = 0.0f;
	hScale( 1, 1 ) = -hScale( 0, 0 );
	hScale( 1, 2 ) = -hScale( 0, 2 );
	hScale( 2, 0 ) = 0.0f;
	hScale( 2, 1 ) = 0.0f;
	hScale( 2, 2 ) = 1.0f;

	// transpose because of column major representation in Matrix
	Math::Matrix< float, 3, 3 > Htrans = ublas::trans( ublas::prod( homography, hScale ) );	
	CvMat cvH = cvMat( 3, 3, CV_32FC1, Htrans.content() );
	
	// create target image
	boost::shared_ptr< Image > r( new Image( nSize, nSize, 1 ) );
	
	// apply homography
	cvWarpPerspective( img, *r, &cvH,
		CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS + CV_WARP_INVERSE_MAP, cvScalar( 128 ) );

	return r;
	}


unsigned long long int readCode( const Image& markerImage, unsigned int iCodeSize, unsigned long long int uiMask )
{
	// This method only works for square markers...
	assert( markerImage.height == markerImage.width );

	LOG4CPP_TRACE( logger, "readCode(): markerImage.width: " << markerImage.width 
		<< ", markerImage.height: " << markerImage.height  
		<< ", iCodeSize: " << iCodeSize 
		<< ", markerImage.widthStep: " <<  markerImage.widthStep 
		<< ", markerImage.width: " << markerImage.width );

	// find threshold
	int nAvg = 0;
	const unsigned char* pData = reinterpret_cast< const unsigned char* >( markerImage.imageData ); 
	for ( int y = 0; y < markerImage.width; y++ )	
		{
		for ( int x = 0; x < markerImage.width; x++ )	
			nAvg += *pData++;
		pData += markerImage.widthStep - markerImage.width;
		}
		
	nAvg /= markerImage.width * markerImage.height;
	
	// check if border is black
	int nBlackBorderPixels = 0;
	pData = reinterpret_cast< const unsigned char* >( markerImage.imageData ); 
	unsigned int nBorderThickness = (markerImage.width - iCodeSize) / 2;
	// Iterate border thickness
	for ( unsigned int j = 0; j < nBorderThickness; j++ ) {
		// Iterate all horizontal border pixels
		for ( int i = 0; i < markerImage.width; i++ )
		{
			// upper border
			if ( pData[ j * markerImage.widthStep + i ] <= nAvg )
				nBlackBorderPixels++;
			// lower border
			if ( pData[ markerImage.widthStep * ( markerImage.height - 1 - j) + i ] <= nAvg )
				nBlackBorderPixels++;
		}
		// Iterate remaining vertical border pixels
		for ( unsigned int i = nBorderThickness; i < markerImage.height - nBorderThickness; i++ )
		{
			// left
			if ( pData[ i * markerImage.widthStep + j ] <= nAvg )
				nBlackBorderPixels++;
			// right
			if ( pData[ i * markerImage.widthStep + markerImage.width - 1 - j ] <= nAvg )
				nBlackBorderPixels++;
		}
	}
		
	if ( nBlackBorderPixels < 2 * markerImage.width + 2 * ( markerImage.height - 2 ) - g_nMaxWhiteBorderPixels ) {
		LOG4CPP_TRACE( logger, "readCode(): black border corrupt" );
		return 0;
	}
		
	// compute marker code
	unsigned long long int nMarkerCode = 0;
	unsigned int uiBorderWidth = (markerImage.width - iCodeSize) / 2;
	for ( unsigned int y = uiBorderWidth; y < (markerImage.width - uiBorderWidth); y++ )	
		for ( unsigned int x = uiBorderWidth; x < (markerImage.width - uiBorderWidth); x++ )
			if ( pData[ y * markerImage.widthStep + x ] <= nAvg )
				nMarkerCode |= ((unsigned long long int)1) << ( (markerImage.width - uiBorderWidth - y - 1) * iCodeSize + (markerImage.width - uiBorderWidth - x - 1) );
					
	LOG4CPP_TRACE( logger, "readCode(): raw code: 0x" << std::hex << nMarkerCode );

	if ( uiMask != 0 )
		nMarkerCode &= uiMask;

	LOG4CPP_TRACE( logger, "readCode(): masked code: 0x" << std::hex << nMarkerCode );
	
	return nMarkerCode;
}


/** 
 * \internal
 * rotates a marker code 90 degrees counter-clockwise.
 *
 * @param nCode code to rotate
 * @param codeLen side-length of marker code
 * @return rotated marker code
 */
unsigned long long int rotateMarkerCode( unsigned long long int nCode, int codeLen )
{
	LOG4CPP_TRACE( logger, "rotateMarkerCode(): old: 0x" << std::hex << nCode );

	unsigned long long int nRotCode = 0;
	for ( int j = 0; j < codeLen * codeLen; j++ )
		if ( ( nCode & ( ((unsigned long long int)1) << j ) ) != 0 )
			nRotCode |= ((unsigned long long int)1) << ( codeLen * ( j % codeLen ) + codeLen - 1 - ( j / codeLen ) );

	LOG4CPP_TRACE( logger, "rotateMarkerCode(): new: 0x" << std::hex << nRotCode );

	return nRotCode;
}


unsigned long long int normalizeCode( unsigned long long int nCode, int codeSize, int& nRotations )
{
	LOG4CPP_TRACE( logger, "normalizeCode(): old: 0x" << std::hex << nCode );
	// return the code with the smallest numeric representation
	unsigned long long int nMinCode = nCode;
	unsigned long long int rotCode = nCode;
	nRotations = 0;
	for ( int i = 0; i < 3; i++ )
	{
		rotCode = rotateMarkerCode( rotCode, codeSize );
		if ( rotCode < nMinCode )
		{
			nMinCode = rotCode;
			nRotations = i + 1;
		}
	}

	LOG4CPP_TRACE( logger, "normalizeCode(): new: 0x" << std::hex << nMinCode);
	return nMinCode;
}


void computeMarkerEdgels( unsigned long long int markerCode, float markerSize, 
	std::vector< Math::Vector< float, 3 > >& points1, std::vector< Math::Vector< float, 3 > >& points2, 
	unsigned int iCodeSize, unsigned int iMarkerSize, unsigned long long int uiMask, bool computeInnerEdgels )
{
	float halfMarkerSize = (float)iMarkerSize / 2;
	float halfCodeSize = (float)iCodeSize / 2;
	float scale = markerSize / (float)iMarkerSize;
	
	LOG4CPP_TRACE( logger, "computeMarkerEdgels(): scale is: " << scale );

	// start with outer border pixels
	for ( unsigned int i = 0; i < iMarkerSize; i++ )
	{
		float p1 = (float)( -(halfMarkerSize-0.5) + i ) * scale;
		float p2 = (float)( -(halfMarkerSize+0.2) + i ) * scale;

		// left
		points1.push_back( Math::Vector< float, 3 >( -halfMarkerSize * scale, p1, 0 ) );
		points2.push_back( Math::Vector< float, 3 >( -halfMarkerSize * scale, p2, 0 ) );

		// right
		points1.push_back( Math::Vector< float, 3 >( halfMarkerSize * scale, -p1, 0 ) );
		points2.push_back( Math::Vector< float, 3 >( halfMarkerSize * scale, -p2, 0 ) );

		// top
		points1.push_back( Math::Vector< float, 3 >( p1, halfMarkerSize * scale, 0 ) );
		points2.push_back( Math::Vector< float, 3 >( p2, halfMarkerSize* scale, 0 ) );

		// bottom
		points1.push_back( Math::Vector< float, 3 >( -p1, -halfMarkerSize * scale, 0 ) );
		points2.push_back( Math::Vector< float, 3 >( -p2, -halfMarkerSize * scale, 0 ) );
	}
	
	if ( !computeInnerEdgels )
		return;

	// inner border pixels
	for ( unsigned int i = 0; i < iCodeSize; i++ )
	{
		float p = ( -(halfCodeSize-0.5f) + i ) * scale;
		float p1 = ( -(halfCodeSize+0.2f) + i ) * scale;
		float p2 = ( -(halfCodeSize-1.2f) + i ) * scale;

		// left
		if ( ! ( markerCode & ( ((unsigned long long int)1) << ( i * iCodeSize + (iCodeSize-1) ) ) ) )
		{
			points1.push_back( Math::Vector< float, 3 >( -halfCodeSize * scale, p, 0 ) );
			points2.push_back( Math::Vector< float, 3 >( -halfCodeSize * scale, p2, 0 ) );
		}

		// right
		if ( ! ( markerCode & ( ((unsigned long long int)1) << ( i * iCodeSize ) ) ) )
		{
			points1.push_back( Math::Vector< float, 3 >( halfCodeSize * scale, p, 0 ) );
			points2.push_back( Math::Vector< float, 3 >( halfCodeSize * scale, p1, 0 ) );
		}

		// top
		if ( ! ( markerCode & ( ((unsigned long long int)1) << ( i + iCodeSize*iCodeSize - iCodeSize ) ) ) )
		{
			points1.push_back( Math::Vector< float, 3 >( -p, halfCodeSize * scale, 0 ) );
			points2.push_back( Math::Vector< float, 3 >( -p1, halfCodeSize * scale, 0 ) );
		}

		// bottom
		if ( ! ( markerCode & ( ((unsigned long long int)1) << i ) ) )
		{
			points1.push_back( Math::Vector< float, 3 >( -p, -halfCodeSize * scale, 0 ) );
			points2.push_back( Math::Vector< float, 3 >( -p2, -halfCodeSize * scale, 0 ) );
		}
	}

	// inner edges
	for ( unsigned int y = 0; y < iCodeSize; y++ ) 
	{
		for ( unsigned int x = 0; x < iCodeSize; x++ )
		{
			// Ignore masked bits
			if ( ! ( uiMask & ( ((unsigned long long int)1) << ( y * iCodeSize + x ) ) ) )
				continue;
		
			bool bSet = ( markerCode & ( ((unsigned long long int)1) << ( y * iCodeSize + x ) ) ) != 0;

			// left edge, for all except leftmost column, ignore masked neighbor bits
			if ( ( x < (iCodeSize - 1) ) && ( uiMask & ( ((unsigned long long int)1) << ( y * iCodeSize + (x + 1) ) ) ) )
			{
				bool bLeftSet = ( markerCode & ( ((unsigned long long int)1) << ( y * iCodeSize + (x + 1) ) ) ) != 0;
				if ( !bSet && bLeftSet )
				{
					points1.push_back( Math::Vector< float, 3 >( (halfCodeSize - x -1.0f) * scale, ( -(halfCodeSize-0.5f) + y ) * scale, 0 ) );
					points2.push_back( Math::Vector< float, 3 >( (halfCodeSize - x -1.0f) * scale, ( -(halfCodeSize-1.2f) + y ) * scale, 0 ) );
				}
				else if ( bSet && !bLeftSet )
				{
					points1.push_back( Math::Vector< float, 3 >( (halfCodeSize - x -1.0f) * scale, ( -(halfCodeSize-0.5f) + y ) * scale, 0 ) );
					points2.push_back( Math::Vector< float, 3 >( (halfCodeSize - x -1.0f) * scale, ( -(halfCodeSize+0.2f) + y ) * scale, 0 ) );
				}
			}
			// upside edge, for all except highest row, ignore masked neighbor bits
			if ( ( y < (iCodeSize - 1) ) && ( uiMask & ( ((unsigned long long int)1) << ( (y + 1) * iCodeSize + x ) ) ) )
			{
				bool bUpSet = ( markerCode & ( ((unsigned long long int)1) << ( (y + 1) * iCodeSize + x ) ) ) != 0;
				if ( !bSet && bUpSet )
				{
					points1.push_back( Math::Vector< float, 3 >( ( (halfCodeSize-0.5f) - x ) * scale, ( y - halfCodeSize + 1.0f) * scale, 0 ) );
					points2.push_back( Math::Vector< float, 3 >( ( (halfCodeSize+0.2f) - x ) * scale, ( y - halfCodeSize + 1.0f) * scale, 0 ) );
				}
				else if ( bSet && !bUpSet )
				{
					points1.push_back( Math::Vector< float, 3 >( ( (halfCodeSize-0.5f) - x ) * scale, ( y - halfCodeSize + 1.0f) * scale, 0 ) );
					points2.push_back( Math::Vector< float, 3 >( ( (halfCodeSize-1.2f) - x ) * scale, ( y - halfCodeSize + 1.0f) * scale, 0 ) );
				}
			}
		}
	}
}


Math::Pose alternateMarkerPose( const Math::Pose& p )
{
	namespace ublas = boost::numeric::ublas;

	// compute normal and viewing direction
	Math::Matrix< double, 3, 3 > rot( p.rotation() );
	rot( 0, 2 ) *= -1;
	rot( 1, 2 ) *= -1;
	rot( 2, 0 ) *= -1;
	rot( 2, 1 ) *= -1;

	// add rotation
	return Math::Pose( Math::Quaternion( rot ), p.translation() );
}





void drawCube( Vision::Image& img, const Math::Pose& pose, const Math::Matrix< float, 3, 3 >& K, double markerSize, CvScalar color, double error )
{
	// the corner points
	static float points3D[ 8 ][ 3 ] = {
		{ 0.5f, 0.5f, 0.0f }, { -0.5f, 0.5f, 0.0f }, { -0.5f, -0.5f, 0.0f }, { 0.5f, -0.5f, 0.0f },
		{ 0.5f, 0.5f, 1.0f }, { -0.5f, 0.5f, 1.0f }, { -0.5f, -0.5f, 1.0f }, { 0.5f, -0.5f, 1.0f }
	};

	// project points
	Math::Vector< double, 3 > p2D[ 8 ];
	for ( int i = 0; i < 8; i++ )
	{
		p2D[ i ] = ublas::prod( K, pose * Math::Vector< double, 3 >( markerSize * Math::Vector< float, 3 >( points3D[ i ] ) ) );
		p2D[ i ] /= p2D[ i ][ 2 ];
	}

	// draw some lines
	for ( int c = 0; c < 8; c += 4 )
		for ( int i = 0; i < 4; i++ )
			cvLine( img, cvPoint( cvRound( p2D[ c + i ][ 0 ] * 16 ), cvRound( p2D[ c + i ][ 1 ] * 16 ) ),
				cvPoint( cvRound( p2D[ c + ( i + 1 ) % 4 ][ 0 ] * 16 ), cvRound( p2D[ c + ( i + 1 ) % 4 ][ 1 ] * 16 ) ),
				color, 1, CV_AA, 4 );
	for ( int i = 0; i < 4; i++ )
		cvLine( img, cvPoint( cvRound( p2D[ i ][ 0 ] * 16 ), cvRound( p2D[ i ][ 1 ] * 16 ) ),
			cvPoint( cvRound( p2D[ i + 4 ][ 0 ] * 16 ), cvRound( p2D[ i + 4 ][ 1 ] * 16 ) ),
			i == 0 ? CV_RGB( 0, 0, 0 ) : color, 1, CV_AA, 4 );

	if (error > -0.5) {
		// draw line representing error value
		CvScalar colorE = getGradientRampColor (error, 0.0, 100.0);
		cvLine( img, cvPoint( cvRound( p2D[ 4 ][ 0 ] * 16 ), cvRound( p2D[ 4 ][ 1 ] * 16 ) ),
			cvPoint( cvRound( p2D[ 6 ][ 0 ] * 16 ), cvRound( p2D[ 6 ][ 1 ] * 16 ) ),
			colorE, 2, CV_AA, 4 );
		cvLine( img, cvPoint( cvRound( p2D[ 5 ][ 0 ] * 16 ), cvRound( p2D[ 5 ][ 1 ] * 16 ) ),
			cvPoint( cvRound( p2D[ 7 ][ 0 ] * 16 ), cvRound( p2D[ 7 ][ 1 ] * 16 ) ),
			colorE, 2, CV_AA, 4 );
	}

}


} } } // namespace Ubitrack::Vision::Markers
