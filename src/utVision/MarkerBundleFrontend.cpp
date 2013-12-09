#include "MarkerBundle.h"

int main( int, char** )
{
	try
	{		
		
		Util::initLogging();

		// parse configuration file
		get_config().parseMBConf( "markerbundle.conf" );
		
		
		
		// load intrinsics
		Vision::Undistortion undistorter( get_config().sMatrixFile, get_config().sDistortionFile );

		Math::Matrix< float, 3, 3 > intrinsics;
		Math::matrix_cast_assign( intrinsics, undistorter.getIntrinsics() );
		
		// find image files in directories
		std::vector< std::string > imageNames;
		
		//The image list should be provided as a parameter
		createImageList( imageNames );
		
		// open each file and search for markers
		BAInfo baInfo( intrinsics, undistorter.getRadialCoeffs() );
		for ( std::vector< std::string >::iterator itImage = imageNames.begin(); itImage != imageNames.end(); itImage++ )
		{
			const std::size_t camId( baInfo.cameras.size() );
			baInfo.cameras.push_back( BACameraInfo() );
			baInfo.cameras.back().name = *itImage;
			baInfo.imageToCam[ *itImage ] = camId;
			
			// load image
			boost::shared_ptr< Vision::Image > pImage( new Vision::Image( cvLoadImage( itImage->c_str(), CV_LOAD_IMAGE_GRAYSCALE ) ) );

			// undistort
			pImage = undistorter.undistort( pImage );
			
			// find markers
			std::map< unsigned long long int, Markers::MarkerInfo > markerMap( get_config().markers );
			
			Markers::detectMarkers( *pImage, markerMap, intrinsics, NULL, false, 8, 12 );
			
			// erase not-seen markers from map
			for( std::map< unsigned long long int, Markers::MarkerInfo >::iterator itMarker = markerMap.begin(); itMarker != markerMap.end();  )
				if ( itMarker->second.found != Vision::Markers::MarkerInfo::ENotFound )
				{
					baInfo.markers[ itMarker->first ].fSize = itMarker->second.fSize;
					baInfo.markers[ itMarker->first ].cameras.insert( camId );
					std::cout << "Found marker " << std::hex << itMarker->first << " in " << *itImage << " (camera " << std::dec << camId << ")" << std::endl;
					itMarker++;
				}
				else
					markerMap.erase( itMarker++ );
			
			// copy remaining marker infos
			baInfo.cameras[ camId ].measMarker.swap( markerMap );
		}
		
		// initialize poses
		baInfo.initMarkers();
		
		// do bundle adjustment
		baInfo.bundleAdjustment( false );
		baInfo.printConfiguration();
		baInfo.printResiduals();

		if ( !get_config().refPoints.empty() )
		{
			// add information from reference points
			baInfo.initRefPoints();
			baInfo.bundleAdjustment( true );
			baInfo.printConfiguration();
			baInfo.printResiduals();
		}

		std::ofstream outFile( get_config().sResultFile.c_str() );
		//baInfo.writeUTQL( outFile );
	}
	catch ( const std::string& s )
	{ std::cout << "Error: " << s << std::endl; }
	catch ( const std::runtime_error& e )
	{ std::cout << "Error: " << e.what() << std::endl; }


}
