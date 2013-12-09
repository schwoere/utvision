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
 * Pixel Flow routines.
 *
 * @author Artashes Mkhitaryan <artashm@gmail.com>
 */


#include "PixelFlow.h"



namespace Ubitrack { namespace Vision {

	PixelFlow::PixelFlow()
	{
	}
	
	void PixelFlow::calcProjectionBuffer( const Image& image, const Math::Vector< int, 2 >& topLeft, const Math::Vector< int, 2 >& bottomRight )
	{
		//clearing previous data
		x.clear();
		y.clear();
		topL.clear();
		bottomR.clear();

		//decliaring variables
		int width = image.widthStep;
		int hight = image.height;
		int xSize = bottomRight(0) - topLeft(0) + 1;
		int ySize = bottomRight(1) - topLeft(1) + 1;
			
		//intializing the variables
		topL.push_back(topLeft(0));
		topL.push_back(topLeft(1));
		bottomR.push_back(bottomRight(0));
		bottomR.push_back(bottomRight(1));
		
		x.resize( xSize, 0 );
		y.resize( ySize, 0 );
		
		//calculating the buffer
		for (int i = 0; i < xSize; i++)
			for(int j = 0; j < ySize; j++)
			{	
				int temp = (topLeft(1) + j)*image.widthStep + topLeft(0)+ i;
				x[i] += ((unsigned char*)image.imageData)[temp];
				y[j] += ((unsigned char*)image.imageData)[temp];
			}
		for(int i = 0; i < xSize; i++)
			x[i] = x[i]/ySize;
		
		for(int i = 0; i < ySize; i++)
			y[i] = y[i]/xSize;

	}

	void PixelFlow::calculateIncrisedCoordiants(int width, int height,double incr, std::vector<int> topL, std::vector<int> bottomR,
			Math::Vector< int, 4 >& xRec, Math::Vector< int, 4 >& yRec)
	{
		int incrX = int((bottomR[0] - topL[0]+1)*incr);	
		int incrY = int((bottomR[1] - topL[1]+1)*incr);

		xRec(0) = std::max(topL[0] - incrX,0);
		xRec(1) = topL[1];
		xRec(2) = std::min(bottomR[0] + incrX, width);
		xRec(3) = bottomR[1];

		yRec(0) = topL[0];
		yRec(1) = std::max(topL[1] - incrY, 0);
		yRec(2) = bottomR[0];
		yRec(3) = std::min(bottomR[1] + incrY,height);
	}

	void PixelFlow::calculateErrorBuffer( const std::vector< int >& v,const std::vector< int >& newV, std::vector< int >& ErrorBuf, int nCut)
	{
		int temp,t,nSize;
		
		if(nCut == 0)
		{
			nSize = int(ErrorBuf.size());
		}
		else
		if(nCut == -1)
		{
			nSize = int(ErrorBuf.size()- v.size()/2);
		}
		else
		if(nCut == 1 || nCut == 2)
		{
			nSize = int(ErrorBuf.size() - v.size()/2);
		}
		
		//the case whne we dont have any shifts we just pass theough buffer
		for (int shift = 0 ; shift < nSize; shift++ )
			{	
			temp = 0;
			for(int i = 0; i < int(v.size()); i++)
			{
				t = v[i] - newV[shift + i];
				temp += t*t;
			}
			ErrorBuf[shift]= temp/int(v.size());
		}
		
		//in case if there is a cut from right side
		if(nCut == 1 || nCut == 2)
		{
			for(int shift = 1; shift <= int(v.size()/2); shift++)
			{
				temp = 0;
				for(int i = 0; i < int(v.size()) - shift; i++)
				{
					t = v[i] - newV[int(newV.size() - v.size()) + shift + i];
					temp += t*t;
				}
				ErrorBuf[int(ErrorBuf.size()) - int(v.size()/2) + shift -1] =  temp/int(v.size() - shift);;
			}
		}
		
		//in case if there is a cut from left side
		if(nCut == -1 || nCut == 2)
		{
			for (int shift = 1;  shift <= int(v.size()/2); shift++)
			{
				temp = 0;
				for(int i = 0; i < int(v.size())-shift; i++)
				{
					t = v[i+shift] - newV[i];
					temp += t*t;
				}
				ErrorBuf.insert(ErrorBuf.begin(),temp/int(v.size() - shift));
			}
		}
	}

	void PixelFlow::computeFlow( const Image& image, Math::Vector< int, 2 >& result, int& difference, Image* pDebugImg)
	{
		//decliaring variables
		int xCut = 0;
		int yCut = 0;
		int temp = 0;
		int xIndex = 0;
		int yIndex = 0;
		int xBufSize = 0;
		int yBufSize = 0;
		int width= image.widthStep;
		int hight = image.height; 
		int xSize = int(x.size());
		int ySize = int(y.size());
		Math::Vector< int, 4 > xRec;
		Math::Vector< int, 4 > yRec;

		calculateIncrisedCoordiants(width, hight, 0.5, topL, bottomR, xRec,  yRec);

		
		std::vector<int> newX( xRec(2) - xRec(0) + 1, 0 );
		std::vector<int> newY( yRec(3) - yRec(1) + 1, 0 );
		
		if(xRec(2) != width)
		{	
			if(((int)x.size())%2 != 0)
				xBufSize = int(x.size());
			else
				xBufSize = int(x.size())+1;
		}
		else
		{	
			xBufSize = int(newX.size()) - int(x.size()/2);
		}
		
		if(yRec(3) != hight)
		{	
			if(((int)y.size())%2 != 0)
				yBufSize = int(y.size());
			else
				yBufSize = int(y.size())+1;
		}
		else
		{	
			yBufSize = int(newY.size()) - int(y.size()/2) ;
		}
		
		std::vector<int> xErrorBuf(xBufSize, 255);
		std::vector<int> yErrorBuf(yBufSize, 255);
						
		//calculating the updated buffer
		for (int i = 0; i < xRec(2) - xRec(0); i++)
			for(int j = 0; j < xRec(3) - xRec(1); j++)
			{	
				int temp = (xRec(1)  + j)*image.width +  xRec(0) + i;
				newX[i] += ((unsigned char*)image.imageData)[temp];
			}
		
		for(int i = 0; i < int(newX.size()); i++)
			newX[i] /= xRec(3) - xRec(1);

		for (int i = 0; i < yRec(2) - yRec(0); i++)
			for(int j = 0; j < yRec(3) - yRec(1); j++)
			{	
				int temp = (yRec(1)  + j)*image.width +  yRec(0) + i;
				newY[j] += ((unsigned char*)image.imageData)[temp];
			}

		for(int i = 0; i < int(newY.size()); i++)
			newY[i] /= yRec(2) - yRec(0);

		//check whether there was a cut
		if(xRec(2) == width)
			xCut = 1;
		if(xRec(0)== 0)
			if(xCut != 1)
				xCut = -1;
			else
				xCut = 2;
			
		//calculating the error buffer
		calculateErrorBuffer(x,newX, xErrorBuf,xCut);

		//check whether there was a cut
		if(yRec(3) == hight)
			yCut = 1;
		if(yRec(1)== 0)
			if(yCut != 1)
				yCut = -1;
			else
				yCut = 2;

		//calculating the error buffer
		calculateErrorBuffer(y,newY, yErrorBuf,yCut);
		
		//finding the best shift in x direction
		temp = xErrorBuf[0];

		for (int i = 1; i <int(xErrorBuf.size()); i++)
			if(temp > xErrorBuf[i])
			{
				temp = xErrorBuf[i];
				xIndex = i;	
			}
		
		//finding the best shift in y direcion
		temp = yErrorBuf[0];
		
		for (int i = 1; i < int(yErrorBuf.size()); i++)
			if(temp > yErrorBuf[i])
			{
				temp = yErrorBuf[i];
				yIndex = i;
			}
		
		//if the nuew buffer is cut from left, the error buffer is shifting on xdif, or ydif
		int xdif, ydif;

		xdif = int(newX.size()) - 2*int(x.size()/2) - int(x.size());
		ydif = int(newY.size()) - 2*int(y.size()/2) - int(y.size());
		
		//returning the reslt
		if(xCut == 0 || xCut == 1)	
		{	
			result(0) = xIndex- int(x.size()/2);
		}
		else
		if(xCut = -1)
		{
			result(0) = xIndex - 2*int(x.size()/2)-xdif;
		}

		if(yCut == 0 || yCut == 1)	
		{	
			result(1) = yIndex - int(y.size()/2) ;
		}
		else
		if(yCut = -1)
		{
			result(1) = yIndex - 2*int(y.size()/2) -ydif;
		}
		
		//TO BE MODIFIED::
		difference = 0;

		//visualizing the error buffer
		if(pDebugImg)
		{
			CvPoint p1,p2;
				
			if(int(xErrorBuf.size()) < width -50)
			{	
				p1.y = 10;
				p2.y = 20;
				
				for (int i = 0; i < int(xErrorBuf.size()); i++)
				{	
					p1.x = width - int(xErrorBuf.size()) - 50 + i ;
					p2.x = width - int(xErrorBuf.size()) - 50 + i ;
					int color = std::min( int(xErrorBuf[i]) + 50, 255 );
					cvLine(pDebugImg,p1,p2,cvScalar(0,0,color),10,CV_AA,0);
				}

				p1.x = width -10;
				p2.x = width -20;
				
				for (int i = 0; i < int(yErrorBuf.size()); i++)
				{	
					p1.y = i + 50;
					p2.y = i + 50;
					int color = std::min( int(yErrorBuf[i]) + 50, 255 );
					cvLine(pDebugImg,p1,p2,cvScalar(0,0,color),10,CV_AA,0);
				}
			}
		}
	}

	PixelFlow::~PixelFlow()
	{
	}	
}}
