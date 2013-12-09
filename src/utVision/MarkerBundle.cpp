//#define INSTANTIATE_SCONFIG

#include "MarkerBundle.h"

static log4cpp::Category& logger( log4cpp::Category::getInstance( "MarkerBundle" ) );

static const Math::Vector< double, 3 > g_unitCorners[ 4 ] = 
{
	Math::Vector< double, 3 >( -0.5,  0.5, 0.0 ),
	Math::Vector< double, 3 >( -0.5, -0.5, 0.0 ),
	Math::Vector< double, 3 >(  0.5, -0.5, 0.0 ),
	Math::Vector< double, 3 >(  0.5,  0.5, 0.0 ) 
};

void SConfig::setResultFile(std::string re)
{
	sResultFile = re;
}
void SConfig::setMatrixFile(std::string mat)
{
	sMatrixFile = mat;
}
void SConfig::setDistortionFile(std::string dis)
{
	sDistortionFile = dis;
}
void SConfig::setMarkersInfo(unsigned long long int code, float size)
{
	markers[ code ].fSize =  size;
}
void SConfig::setRefPositions(std::string id, double x, double y, double z)
{
	refPoints[ id ].pos = Math::Vector< double, 3 >( x , y , z );
}
void SConfig::setRefPoints(std::string id, RefPoint::Meas measurement)
{
	refPoints[ id ].measurements.push_back( measurement );
}
std::string SConfig::getResultFile()
{
	return sResultFile;
}
std::string SConfig::getMatrixFile()
{
	return sMatrixFile;
}
std::string SConfig::getDistortionFile()
{
	return sDistortionFile;
}
std::map< unsigned long long int, Markers::MarkerInfo > SConfig::getMarkers()
{
	return markers;
}


std::size_t BAInfo::size() const
{
	std::size_t s = 0;
	
	// 8 measurements for each marker seen
	for ( CameraList::const_iterator it = cameras.begin(); it != cameras.end(); it++ )
		s += 8 * it->measMarker.size();

	if ( m_bUseRefPoints )
		// add the reference points measurements
		for ( SConfig::RefPointMap::iterator it = get_config().refPoints.begin(); it != get_config().refPoints.end(); it++ )
			s += 2 * it->second.measurements.size();

	else
		// 6 measurements to define the origin
		s += 6;
	
	return s;
}

std::ostringstream out;
double stdDev;

void SConfig::parseMBConf(std::string conFile)
{
	// init regular expressions
	std::string sComment( "\\s*(#.*)?" );
	boost::regex reMarker( "\\s*marker\\s+(0x)?([0-9a-fA-F]+)\\s+([.0-9e]+)" + sComment );
	boost::regex reMatrix( "\\s*matrixFile\\s+([^# \\t]+)" + sComment );
	boost::regex reDistortion( "\\s*distortionFile\\s+([^# \\t]+)" + sComment );
	boost::regex reResult( "\\s*resultFile\\s+([^# \\t]+)" + sComment );
	boost::regex reRefPointPos( "\\s*refPointPos\\s+([0-9A-Za-z_]+)\\s+([-+.0-9e]+)\\s+([-+.0-9e]+)\\s+([-+.0-9e]+)" + sComment );
	boost::regex reRefPointMeasurement( "\\s*refPointMeasurement\\s+([0-9A-Za-z_]+)\\s+([^# \\t]+)\\s+([-+.0-9e]+)\\s+([-+.0-9e]+)" + sComment );
	boost::regex reEmpty( "\\s*(#.*)?" );
	
	setResultFile("multiMarker.utql");
	
	std::ifstream f( conFile.c_str() );
	
	
	if ( f.fail() )
		throw std::runtime_error( "unable to open markerbundle.conf" );

	while ( !f.fail() && !f.eof() )
	{
		std::string buf;
		std::getline( f, buf );

		char* dummy; // needed for strtoX
		boost::smatch match;		
		
		if ( boost::regex_match( buf, match, reEmpty ) )
			; // do nothing
		else if ( boost::regex_match( buf, match, reMarker ) )
		{	
			//###
			std::stringstream ss(match[2].str());
			unsigned long long int code;
			ss >> std::hex >> code;
			
			//### unsigned long long int code = strtoul( match[ 2 ].str().c_str(), &dummy, 16 );
			double size = strtod( match[ 3 ].str().c_str(), &dummy );
			setMarkersInfo(code, float(size));
			
			std::cout << "Looking for marker " << std::hex << code << " of size " << size << std::endl;
			out << "Looking for marker " << std::hex << code << " of size " << size << std::endl;
		}
		else if ( boost::regex_match( buf, match, reMatrix ) )
		{
			setMatrixFile( match[ 1 ] );
			std::cout << "Using intrinsic matrix from " << sMatrixFile << std::endl;
			
			out << "Using intrinsic matrix from " << sMatrixFile << std::endl;
		}
		else if ( boost::regex_match( buf, match, reDistortion ) )
		{
			setDistortionFile( match[ 1 ] );
			
			std::cout << "Using distortion coefficients from " << sDistortionFile << std::endl;
			std::cout << "Checking the source of the error" << std::endl;
			
			out << "Using distortion coefficients from " << sDistortionFile << std::endl;
			out << "Checking the source of the error" << std::endl;
		}
		else if ( boost::regex_match( buf, match, reResult ) )
		{
			setResultFile( match[ 1 ] );
			std::cout << "Writing result to " << sResultFile << std::endl;
			
			out << "Writing result to " << sResultFile << std::endl;
		}
		else if ( boost::regex_match( buf, match, reRefPointPos ) )
		{
			setRefPositions( match [ 1 ], strtod( match[ 2 ].str().c_str(), &dummy ),
			strtod( match[ 3 ].str().c_str(), &dummy ),strtod( match[ 4 ].str().c_str(), &dummy ));
		}
		else if ( boost::regex_match( buf, match, reRefPointMeasurement ) )
		{
			setRefPoints( match[ 1 ] , RefPoint::Meas( match[ 2 ].str(), 
			Math::Vector< double, 2 >( strtod( match[ 3 ].str().c_str(), &dummy ), strtod( match[ 4 ].str().c_str(), &dummy ))));
		}
		else
			std::cerr << "unknown configuration string: " << buf << std::endl;
	}

	if ( markers.empty() )
		throw std::runtime_error( "No marker configurations found" );
}


void BAInfo::printConfiguration()
{
	for ( std::size_t iCam = 0; iCam < cameras.size(); iCam++ )
	{
		std::cout << "Camera " << iCam << ": ~" << ~cameras[ iCam ].pose << std::endl;
		LOG4CPP_TRACE( logger, "Camera " << iCam << ": ~" << ~cameras[ iCam ].pose);
		out << "Camera " << iCam << ": ~" << ~cameras[ iCam ].pose << std::endl;
	}
		
	for ( MarkerMap::iterator it = markers.begin(); it != markers.end(); it++ )
	{
		std::cout << "Marker " << std::hex << it->first << ": " << it->second.pose << std::endl;
		LOG4CPP_TRACE( logger, "Marker " << std::hex << it->first << ": " << it->second.pose);
		out << "Marker " << std::hex << it->first << ": " << it->second.pose << std::endl;
	}
	std::cout << std::endl;
}

void BAInfo::printResiduals()
{
	Math::Vector< double > parameters( parameterSize() );
	genParameterVector( parameters );

	Math::Vector< double > measurements( size() );
	Math::Matrix< double, 0, 0 > J( size(), parameterSize() );
	evaluateWithJacobian( measurements, parameters, J );

	//std::cout << "\nIndividual residuals:\n";
	LOG4CPP_TRACE( logger,"\nIndividual residuals:\n");
	out << "\nIndividual residuals:\n";
	
	std::size_t iMeasurement( 0 );
	stdDev  = 0;
	for ( std::size_t iCam = 0; iCam < cameras.size(); iCam++ )
		for ( BACameraInfo::MarkerMeasMap::iterator it = cameras[ iCam ].measMarker.begin(); 
			it != cameras[ iCam ].measMarker.end(); it++ )
		{
			double r = 0;
			for ( std::size_t i( 0 ); i < 4; i++ )
			{
				double dx = measurements( iMeasurement     ) - it->second.corners[ i ]( 0 );
				double dy = measurements( iMeasurement + 1 ) - it->second.corners[ i ]( 1 );
				stdDev += dx * dx + dy * dy;
				r += sqrt( dx * dx + dy * dy );
				
				
				iMeasurement += 2;
			}
			r /= 4;
			std::cout << r << ": marker " << std::hex << it->first << " in image " << std::dec << iCam << " (" << cameras[ iCam ].name  << ")\n";
			LOG4CPP_TRACE( logger, r << ": marker " << std::hex << it->first << " in image " << std::dec << iCam << " (" << cameras[ iCam ].name  << ")");
			out << r << ": marker " << std::hex << it->first << " in image " << std::dec << iCam << " (" << cameras[ iCam ].name  << ")\n";
		}

	if ( m_bUseRefPoints )
		for ( SConfig::RefPointMap::iterator itP = get_config().refPoints.begin(); itP != get_config().refPoints.end(); itP++ )
			for ( std::vector< SConfig::RefPoint::Meas >::iterator itM = itP->second.measurements.begin();
				itM != itP->second.measurements.end(); itM++ )
			{
				double dx = measurements( iMeasurement     ) - itM->pos( 0 );
				double dy = measurements( iMeasurement + 1 ) - itM->pos( 1 );
				stdDev += dx * dx + dy * dy;
				double r = sqrt( dx * dx + dy * dy );
			
				std::cout << r << ": refpoint " << itP->first << " in image " << std::dec << imageToCam[ itM->image ] << 
					" (" << cameras[ imageToCam[ itM->image ] ].name << ")\n";
				LOG4CPP_TRACE( logger, r << ": refpoint " << itP->first << " in image " << std::dec << imageToCam[ itM->image ] << " (" << cameras[ imageToCam[ itM->image ] ].name << ")");
				out << r << ": refpoint " << itP->first << " in image " << std::dec << imageToCam[ itM->image ] << 
					" (" << cameras[ imageToCam[ itM->image ] ].name << ")\n";
				iMeasurement += 2;
			}

	stdDev /= ( size() - parameterSize() );
	std::cout << "Estimated measurement stddev: " << sqrt( stdDev ) << std::endl;
	out << "Estimated measurement stddev: " << sqrt( stdDev ) << std::endl;
	LOG4CPP_TRACE( logger, "Estimated measurement stddev: " << sqrt( stdDev ));

	// perform backward propagation
	// unclear if this makes sense!
	Math::Matrix< double, 0, 0 > covariance( parameterSize(), parameterSize() );
	if ( m_bUseRefPoints )
		Math::backwardPropagationIdentity( covariance, stdDev, J );
	else
	{
		ublas::matrix_range< typename Math::Matrix< double, 0, 0 >::base_type > subJ( J, ublas::range( 0, J.size1() - 6 ), ublas::range( 0, J.size2() ) );
		Math::backwardPropagationIdentity( covariance, stdDev, subJ );
	}

	std::cout << "Estimated standard deviations (r, t):" << std::endl;
	LOG4CPP_TRACE( logger, "Estimated standard deviations (r, t):" );
	out << "Estimated standard deviations (r, t):" << std::endl;
	
	
	// first cameras
	for ( std::size_t iCam = 0; iCam < cameras.size(); iCam++ )
	{
		std::cout << "camera " << iCam << ": ";
		out << "camera " << iCam << ": ";
		LOG4CPP_TRACE( logger, "camera " << iCam << ": ");
		
		for ( std::size_t i ( 0 ); i < 6 ; i++ )
		{
			std::cout << sqrt( covariance( iCam*6 + i, iCam*6 + i ) ) << ( i < 5 ? ", " : "" );
			LOG4CPP_TRACE( logger, sqrt( covariance( iCam*6 + i, iCam*6 + i ) ) << ( i < 5 ? ", " : "" ));
			out << sqrt( covariance( iCam*6 + i, iCam*6 + i ) ) << ( i < 5 ? ", " : "" );
		}
		std::cout << std::endl;
		out << std::endl;
	}
	
	// then markers
	std::size_t iMarkersStart ( 6 * cameras.size() );
	for ( MarkerMap::iterator it = markers.begin(); it != markers.end(); it++ )
	{
		std::size_t iStart( iMarkersStart + 6 * it->second.index );
		std::cout << "marker " << std::hex << it->first << ": ";
		LOG4CPP_TRACE( logger,"marker " << std::hex << it->first << ": ");
		for ( std::size_t i( 0 ); i < 6 ; i++ )
		{
			std::cout << sqrt( covariance( iStart + i, iStart + i ) ) << ( i < 5 ? ", " : "" );
			LOG4CPP_TRACE( logger, sqrt( covariance( iStart + i, iStart + i ) ) << ( i < 5 ? ", " : "" ));
			out << sqrt( covariance( iStart + i, iStart + i ) ) << ( i < 5 ? ", " : "" );
		}
		std::cout << std::endl;
	}

	std::cout << std::endl;
	out << std::endl;
}

std::ostringstream &getStream()
{
	return out;
}

double getStdDev()
{
	return stdDev ;
}

void BAInfo::initMarkers()
{
	// number markers
	std::size_t iMarker ( 0 );
	for ( MarkerMap::iterator it = markers.begin(); it != markers.end(); it++ )
		it->second.index = iMarker++;

	
	// initialize poses
	std::deque< unsigned long long int> markerQueue;
	std::deque< std::size_t > cameraQueue;

	
	
	const unsigned long long int startMarker = markers.begin()->first;	
	
	
	markers[ startMarker ].pose = Math::Pose( Math::Quaternion( 0, 0, 0, 1 ), Math::Vector< double, 3 >( 0, 0, 0 ) );	
	markers[ startMarker ].bPoseComputed = true;
	markerQueue.push_back( startMarker );
	
	
	// kind of a breadth-first search over markers and cameras 
	while ( !markerQueue.empty() || !cameraQueue.empty() )
	{
		if ( !markerQueue.empty() )
		{
			// take one marker out of the queue
			unsigned long long int id = markerQueue.front();
			markerQueue.pop_front();
			BAMarkerInfo& info( markers[ id ] );
			
			// iterate over cameras that saw the marker
			for ( std::set< std::size_t >::iterator it = info.cameras.begin(); it != info.cameras.end(); it++ )
				if ( !cameras[ *it ].bPoseComputed )
				{
					cameras[ *it ].bPoseComputed = true;
					cameras[ *it ].pose = cameras[ *it ].measMarker[ id ].pose * ~info.pose;
					cameraQueue.push_back( *it );
				}
		}
		
		if ( !cameraQueue.empty() )
		{
			// take one camera out of the queue
			const std::size_t id( cameraQueue.front() );
			cameraQueue.pop_front();
			BACameraInfo& info( cameras[ id ] );
			
			// iterate over markers seen by the camera
			for ( BACameraInfo::MarkerMeasMap::iterator it = info.measMarker.begin(); it != info.measMarker.end(); it++ )
				if ( !markers[ it->first ].bPoseComputed )
				{
					markers[ it->first ].bPoseComputed = true;
					markers[ it->first ].pose = ~info.pose * it->second.pose;
					markerQueue.push_back( it->first );
				}
		}
	}
	
}

void BAInfo::initRefPoints(bool undistorted)
{
	// undistort all reference point measurements
	if(!undistorted)
		for ( SConfig::RefPointMap::iterator it = get_config().refPoints.begin(); it != get_config().refPoints.end(); it++ )
			for ( std::vector< SConfig::RefPoint::Meas >::iterator itM = it->second.measurements.begin();
				itM != it->second.measurements.end(); itM++ )
				itM->pos = Calibration::lensUnDistort( itM->pos, radialCoeffs, intrinsicMatrix );
				

	// find reference points with at least two measurements
	std::cout << std::endl << "Initializing reference points:" << std::endl;
    getStream() << std::endl << "Initializing reference points:" << std::endl;
	std::vector< Math::Vector< double, 3 > > refPointsCam;
	std::vector< Math::Vector< double, 3 > > refPointsRoom;

	for ( SConfig::RefPointMap::iterator it = get_config().refPoints.begin(); it != get_config().refPoints.end(); it++ )
		if ( it->second.measurements.size() >= 2 )
		{
			SConfig::RefPoint::Meas& m1( *(it->second.measurements.begin() ) );
			SConfig::RefPoint::Meas& m2( *(it->second.measurements.rbegin()) );

			// triangulate position in camera coordinates
			Math::Matrix< double, 3, 4 > P1( cameras[ imageToCam[ m1.image ] ].pose );
			P1 = ublas::prod( intrinsicMatrix, P1 );
			Math::Matrix< double, 3, 4 > P2( cameras[ imageToCam[ m2.image ] ].pose );
			P2 = ublas::prod( intrinsicMatrix, P2 );
			Math::Vector< double, 3 > p3d = Calibration::get3DPosition( P1, P2, m1.pos, m2.pos );

			refPointsCam.push_back( p3d );
			refPointsRoom.push_back( it->second.pos );
			
			

			//std::cout << "Point " << it->first << " from {" << m1.image << ", " << m2.image << "}: " << p3d << std::endl;
			LOG4CPP_TRACE( logger, "Point " << it->first << " from {" << m1.image << ", " << m2.image << "}: " << p3d );
			//std::cout << "      world: " << it->second.pos << std::endl;
			LOG4CPP_TRACE( logger, " world: " << it->second.pos );
		}
		
	//LOG4CPP_TRACE( logger, "Just Checking " << refPointsCam.size());
	if ( refPointsCam.size() < 3 )
		throw std::runtime_error( "You need at least three reference points which are seen by at least two cameras" );
	else
	{
		Math::Pose t = Calibration::calculateAbsoluteOrientation( refPointsCam, refPointsRoom );
		//std::cout << "Room transformation " << t << " / " << ~t << std::endl;
		LOG4CPP_TRACE( logger, "Room transformation " << t << " / " << ~t );

		// multiply the t to all poses
		for ( std::size_t iCam = 0; iCam < cameras.size(); iCam++ )
			cameras[ iCam ].pose = cameras[ iCam ].pose * ~t;

		for ( MarkerMap::iterator it = markers.begin(); it != markers.end(); it++ )
			it->second.pose = t * it->second.pose;
	}
}

void BAInfo::writeUTQL( std::ostream& of )
{
	
	of << "<UTQLResponse>\n";
	of << "\n";
    of << "<Pattern name=\"DirectShowFrameGrabber\" id=\"Framegrabber\">\n";
	of << "    <!-- replace with your favourite frame grabber -->\n";
    of << "   <Output>\n";
    of << "        <Node name=\"Camera\"/>\n";
    of << "        <Node name=\"ImagePlane\"/>\n";
    of << "        <Edge name=\"Intrinsics\" source=\"Camera\" destination=\"ImagePlane\">\n";
    of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"3x3Matrix\"/>\n";
    of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"pull\"/>\n";
    of << "        </Edge>\n";
    of << "        <Edge name=\"Output\" source=\"Camera\" destination=\"ImagePlane\">\n";
    of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"Image\"/>\n";
    of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"push\"/>\n";
    of << "        </Edge>\n";
    of << "    </Output>\n";
    of << "    <DataflowConfiguration>\n";
    of << "        <UbitrackLib class=\"DirectShowFrameGrabber\"/>\n";
    of << "        <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"imageWidth\" value=\"320\"/>\n";
    of << "        <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"timeOffset\" value=\"0\"/>\n";
    of << "        <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"intrinsicMatrixFile\" value=\"CamMatrix.calib\" />\n";
    of << "        <!--Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"cameraName\" value=\"\" /-->\n";
    of << "        <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"distortionFile\" value=\"CamCoeffs.calib\" />\n";
    of << "        <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"divisor\" value=\"1\" />\n";
    of << "        <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"imageHeight\" value=\"240\" />\n";
    of << "    </DataflowConfiguration>\n";
    of << "</Pattern>\n";
	of << "\n";

	// for each marker write a marker tracker pattern
	for ( MarkerMap::iterator it = markers.begin(); it != markers.end(); it++ )
	{
		of << "<Pattern name=\"MarkerTracker\" id=\"Marker" << std::hex << it->first << "\">\n";
		of << "    <Input>\n";
		of << "        <Node name=\"Camera\" id=\"node_1\"/>\n";
		of << "        <Node name=\"ImagePlane\" id=\"node_2\"/>\n";
		of << "        <Node name=\"Marker\" id=\"node_3\">\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"markerId\" value=\"0x" << std::hex << it->first << "\" />\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"markerSize\" value=\"" << it->second.fSize << "\" />\n";
		of << "        </Node>\n";
		of << "        <Node name=\"MarkerPoints\" id=\"node_4\"/>\n";
		of << "        <Edge name=\"Image\" source=\"Camera\" destination=\"ImagePlane\" pattern-ref=\"Framegrabber\" edge-ref=\"Output\">\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"Image\" />\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"push\" />\n";
		of << "        </Edge>\n";
		of << "    </Input>\n";
		of << "    <Output>\n";
		of << "        <Edge name=\"Corners\" source=\"ImagePlane\" destination=\"MarkerPoints\">\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"3DPositionList\" />\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"push\" />\n";
		of << "        </Edge>\n";
		of << "    </Output>\n";
		of << "    <DataflowConfiguration>\n";
		of << "        <UbitrackLib class=\"MarkerTracker\"/>\n";
		of << "    </DataflowConfiguration>\n";
    	of << "</Pattern>\n";
		of << "\n";
		of << "<Pattern name=\"StaticPositionList\" id=\"StaticCorners" << std::hex << it->first << "\">\n";
    	of << "    <Output>\n";
    	of << "        <Node name=\"A\" id=\"node_3\"/>\n";
    	of << "        <Node name=\"B\" id=\"node_4\"/>\n";
    	of << "        <Edge name=\"AB\" source=\"A\" destination=\"B\">\n";
    	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"3DPositionList\" />\n";
    	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"pull\" />\n";
    	of << "            <Attribute xsi:type=\"utql:ListAttributeType\" name=\"staticPositionList\" >\n";
    	of << "                <Value xsi:type=\"utql:ListOfPrimitiveValueType\">\n";

		//###
		std::vector< Math::Vector< double, 3 > > markerCorners;		

		for ( std::size_t i( 0 ); i < 4; i++ )
		{
			
			Math::Vector< double, 3 > p = it->second.pose * Math::Vector< double, 3 >( g_unitCorners[ i ] * it->second.fSize );
	    	of << "                    <Attribute name=\"staticPosition\" value=\"" << p( 0 ) << " " << p( 1 ) << " " << p( 2 ) << "\"/>\n";
			
			//###
			markerCorners.push_back( p );
		}
		
		//###
		std::ostringstream cornerFileName(std::ostringstream::out);
		cornerFileName << "marker" << std::hex << it->first << ".cal";
		
		Util::writeCalibFile( cornerFileName.str(), Measurement::PositionList( Measurement::now(), markerCorners ) );
		

		of << "                </Value>\n";
    	of << "            </Attribute>\n";
    	of << "        </Edge>\n";
    	of << "    </Output>\n";
    	of << "    <DataflowConfiguration>\n";
   		of << "        <UbitrackLib class=\"StaticPositionList\"/>\n";
   		of << "     </DataflowConfiguration>\n";
    	of << "</Pattern>\n";
		of << "\n";
	}

	// build the pose estimation component
	of << "<Pattern name=\"PoseEstimation\" id=\"PoseEstimation\">\n";
	of << "    <Input>\n";
	of << "        <Node name=\"Camera\" id=\"node_1\"/>\n";
	of << "        <Node name=\"ImagePlane\" id=\"node_2\"/>\n";
	of << "        <Node name=\"Marker\" id=\"node_3\"/>\n";
	of << "        <Node name=\"MarkerPoints\" id=\"node_4\"/>\n";
	of << "        <Edge name=\"Intrinsics\" source=\"Camera\" destination=\"ImagePlane\" pattern-ref=\"Framegrabber\" edge-ref=\"Intrinsics\">\n";
	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"3x3Matrix\" />\n";
	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"pull\" />\n";
	of << "        </Edge>\n";
	of << "        <Edge name=\"Input2d\" source=\"ImagePlane\" destination=\"MarkerPoints\">\n";
	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"2DPositionList\" />\n";
	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"push\" />\n";
	of << "        </Edge>\n";
	of << "        <Edge name=\"Input3d\" source=\"Marker\" destination=\"MarkerPoints\">\n";
	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"3DPositionList\" />\n";
	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"pull\" />\n";
	of << "        </Edge>\n";

	for ( MarkerMap::iterator it = markers.begin(); it != markers.end(); it++ )
	{
		of << "        <Edge name=\"Input2d" << it->first << "\" source=\"ImagePlane\" destination=\"MarkerPoints\" "
			              "pattern-ref=\"Marker" << std::hex << it->first << "\" edge-ref=\"Corners\">\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"2DPositionList\" />\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"push\" />\n";
		of << "        </Edge>\n";
		of << "        <Edge name=\"Input3d" << it->first << "\" source=\"Marker\" destination=\"MarkerPoints\" "
			              "pattern-ref=\"StaticCorners" << std::hex << it->first << "\" edge-ref=\"AB\">\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"3DPositionList\" />\n";
		of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"pull\" />\n";
		of << "        </Edge>\n";
	}

	of << "    </Input>\n";
	of << "    <Output>\n";
	of << "        <Edge name=\"Output\" source=\"Camera\" destination=\"Marker\">\n";
	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"type\" value=\"6D\" />\n";
	of << "            <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"mode\" value=\"push\" />\n";
	of << "        </Edge>\n";
	of << "    </Output>\n";
	of << "    <DataflowConfiguration>\n";
	of << "        <UbitrackLib class=\"2D3DPoseEstimation\"/>\n";
	of << "        <Attribute xsi:type=\"utql:PrimitiveAttributeType\" name=\"expansion\" value=\"space\" />\n";
	of << "    </DataflowConfiguration>\n";
	of << "</Pattern>\n";
	of << "\n";
	of << "</UTQLResponse>\n";

}

template< class VT1, class VT2, class MT1 > 
void BAInfo::evaluateWithJacobian( VT1& result, const VT2& input, MT1& J) const
{
	using namespace Math::Function;
	using namespace Calibration::Function;

	// initialize jacobian
	J =  Math::Matrix< double, 0, 0 >::zeros( J.size1(), J.size2() );

	const std::size_t iMarkersStart( 6 * cameras.size() );
	std::size_t  iMeasurement = 0;
	for ( std::size_t iCam( 0 ); iCam < cameras.size(); iCam++ )
	{
		const std::size_t iCamStart( iCam * 6 );
		
		// marker measurements
		for ( BACameraInfo::MarkerMeasMap::const_iterator it = cameras[ iCam ].measMarker.begin(); 
			it != cameras[ iCam ].measMarker.end(); it++ )
		{
			const std::size_t iMarkerStart = iMarkersStart + 6 * markers.find( it->first )->second.index;
			for ( std::size_t iCorner( 0 ); iCorner < 4; iCorner++ )
			{	
				ublas::vector_range< VT1 > subResult( result, ublas::range( iMeasurement, iMeasurement + 2 ) );
				ublas::matrix_range< MT1 > subJ( J, ublas::range( iMeasurement, iMeasurement + 2 ), ublas::range( 0, J.size2() ) );

				( CameraIntrinsicsMultiplication() <<
					fixedParameterRef< 5 >( intrinsics ) << 
					( Dehomogenization< 3 >() << 
						( Addition< 3 >() << 
							parameter< 3 >( iCamStart + 3 ) <<
							( LieRotation() <<
								parameter< 3 >( iCamStart ) <<
								( Addition< 3 >() <<
									parameter< 3 >( iMarkerStart + 3 ) <<
									( LieRotation() <<
										parameter< 3 >( iMarkerStart ) <<
										fixedParameterCopy< 3 >( it->second.fSize * g_unitCorners[ iCorner ] )
									)
								)
							)
						)
					)
				).evaluateWithJacobian( input, subResult, subJ );
				
				iMeasurement += 2;
			}
		}
	}

	if ( m_bUseRefPoints )
	{
		for ( SConfig::RefPointMap::const_iterator itP = get_config().refPoints.begin(); itP != get_config().refPoints.end(); itP++ )
			for ( std::vector< SConfig::RefPoint::Meas >::const_iterator itM = itP->second.measurements.begin();
				itM != itP->second.measurements.end(); itM++ )
			{
				ublas::vector_range< VT1 > subResult( result, ublas::range( iMeasurement, iMeasurement + 2 ) );
				ublas::matrix_range< MT1 > subJ( J, ublas::range( iMeasurement, iMeasurement + 2 ), ublas::range( 0, J.size2() ) );

				const std::size_t iCamStart( 6 * imageToCam.find( itM->image )->second );

				( CameraIntrinsicsMultiplication() <<
					fixedParameterRef< 5 >( intrinsics ) << 
					( Dehomogenization< 3 >() << 
						( Addition< 3 >() << 
							parameter< 3 >( iCamStart + 3 ) <<
							( LieRotation() <<
								parameter< 3 >( iCamStart ) <<
								fixedParameterRef< 3 >( itP->second.pos )
							)
						)
					)
				).evaluateWithJacobian( input, subResult, subJ);
				
				iMeasurement += 2;
			}
	}
	else
	{
		// add measurement of origin (first marker)
		ublas::vector_range< VT1 > subROrigin( result, ublas::range( iMeasurement, iMeasurement + 6 ) );
		subROrigin = ublas::subrange( input, iMarkersStart, iMarkersStart + 6 );

		ublas::matrix_range< MT1 > subJOrigin( J, ublas::range( iMeasurement, iMeasurement + 6 ), ublas::range( iMarkersStart, iMarkersStart + 6 ) );
		subJOrigin = Math::Matrix< double, 6, 6 >::identity();
	}

	//LOG4CPP_TRACE( logger, "J: " << J );
}

template< class VT >
void BAInfo::genTargetVector( VT& v )
{
	std::size_t iMeasurement = 0;
	for ( std::size_t iCam ( 0 ); iCam < cameras.size(); iCam++ )
		for ( BACameraInfo::MarkerMeasMap::iterator it = cameras[ iCam ].measMarker.begin(); 
			it != cameras[ iCam ].measMarker.end(); it++ )
			for ( std::size_t i( 0 ); i < 4; i++ )
			{
				ublas::vector_range< VT > subMeas( v, ublas::range( iMeasurement, iMeasurement + 2 ) );
				subMeas = it->second.corners[ i ];
				iMeasurement += 2;
			}

	if ( m_bUseRefPoints )
	{
		// add reference point measurements
		for ( SConfig::RefPointMap::iterator itP = get_config().refPoints.begin(); itP != get_config().refPoints.end(); itP++ )
			for ( std::vector< SConfig::RefPoint::Meas >::iterator itM = itP->second.measurements.begin();
				itM != itP->second.measurements.end(); itM++ )
			{
				ublas::vector_range< VT > subMeas( v, ublas::range( iMeasurement, iMeasurement + 2 ) );
				subMeas = itM->pos;
				iMeasurement += 2;
			}
	}
	else
	{
		// add origin measurement
		ublas::vector_range< VT > subOrigin( v, ublas::range( iMeasurement, iMeasurement + 6 ) );
		subOrigin = Math::Vector< double, 6 >::zeros();
	}
}

template< class VT >
void BAInfo::genParameterVector( VT& v )
{
	// first cameras
	for ( std::size_t iCam ( 0 ); iCam < cameras.size(); iCam++ )
	{
		ublas::vector_range< VT > subRot( v, ublas::range( iCam * 6, iCam * 6 + 3 ) );
		subRot = cameras[ iCam ].pose.rotation().toLogarithm();
		ublas::vector_range< VT > subTrans( v, ublas::range( iCam * 6 + 3, iCam * 6 + 6 ) );
		subTrans = cameras[ iCam ].pose.translation();
	}
	
	// then markers
	const std::size_t iMarkersStart ( 6 * cameras.size() );
	for ( MarkerMap::iterator it = markers.begin(); it != markers.end(); it++ )
	{
		const std::size_t iStart( iMarkersStart + 6 * it->second.index );
		ublas::vector_range< VT > subRot( v, ublas::range( iStart, iStart + 3 ) );
		subRot = it->second.pose.rotation().toLogarithm();
		ublas::vector_range< VT > subTrans( v, ublas::range( iStart + 3, iStart + 6 ) );
		subTrans = it->second.pose.translation();
	}
}

template< class VT >
void BAInfo::updateParameters( VT& v )
{
	// first cameras
	for ( std::size_t iCam = 0; iCam < cameras.size(); iCam++ )
	{
		cameras[ iCam ].pose = Math::Pose( 
			Math::Quaternion::fromLogarithm( ublas::subrange( v, iCam * 6, iCam * 6 + 3 ) ),
			ublas::subrange( v, iCam * 6 + 3, iCam * 6 + 6 ) );
	}
	
	// then markers
	const std::size_t iMarkersStart( 6 * cameras.size() );
	for ( MarkerMap::iterator it = markers.begin(); it != markers.end(); it++ )
	{
		const std::size_t iStart ( iMarkersStart + 6 * it->second.index );
		it->second.pose = Math::Pose( 
			Math::Quaternion::fromLogarithm( ublas::subrange( v, iStart, iStart + 3 ) ),
			ublas::subrange( v, iStart + 3, iStart + 6 ) );
	}
}

void BAInfo::bundleAdjustment( bool bUseRefPoints )
{	
	m_bUseRefPoints = bUseRefPoints;

	Math::Vector< double > measurements( size() );
	genTargetVector( measurements );
	
	LOG4CPP_TRACE( logger, "Starting Bundle Adjustment" );
	
	Math::Vector< double > parameters( parameterSize() );
	genParameterVector( parameters );

	LOG4CPP_DEBUG( logger, "original parameters: " << parameters );
	LOG4CPP_DEBUG( logger, "original measurements: " << measurements);
	Math::levenbergMarquardt( *this, parameters, measurements, Math::OptTerminate( 200, 1e-6 ), Math::OptNoNormalize() );
	LOG4CPP_DEBUG( logger, "improved parameters: " << parameters );
	
	updateParameters( parameters );
	LOG4CPP_TRACE( logger, "Ending Bundle Adjustment" );
}


void createImageList( std::vector< std::string >& l )
{
	using namespace boost::filesystem;
	
	boost::regex ext( ".*\\.(jpg|JPG|png|PNG|bmp|BMP)" );

#if BOOST_FILESYSTEM_VERSION == 3	
	path compPath( ".");
#else
	path compPath( ".", native );
#endif
	if ( !exists( compPath ) )
		throw std::string( "path does not exist" );
	
	// iterate directory
	directory_iterator dirEnd;
	for ( directory_iterator it( compPath ); it != dirEnd; it++ )
	{
#ifdef BOOST_FILESYSTEM_I18N
		path p( it->path() );
#else
		path p( *it );
#endif

		std::string suffix = ".jpg";
		
		// check if file of suitable extension
#if BOOST_FILESYSTEM_VERSION == 3
		if ( exists( p ) && !is_directory( p ) && regex_match( p.leaf().string(), ext ) )
			l.push_back( p.leaf().string() );
#else
		if ( exists( p ) && !is_directory( p ) && regex_match( p.leaf(), ext ) )
			l.push_back( p.leaf() );
#endif
	}

}


SConfig &get_config()
{
	static SConfig g_config;
	return g_config;
}
