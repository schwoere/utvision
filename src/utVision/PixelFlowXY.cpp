#include "PixelFlowXY.h"

namespace Ubitrack { namespace Vision {

PixelFlowXY::PixelFlowXY(int width, int height, int maxShift) : AbstractPixelFlow(width,height,maxShift)
{
	sumx = new unsigned int[width];
	sumx_old = new unsigned int[width];

	sumy = new unsigned int[height];
	sumy_old = new unsigned int[height];
}

PixelFlowXY::~PixelFlowXY(void)
{
	delete[] sumx;
	delete[] sumx_old;
	delete[] sumy;
	delete[] sumy_old;
}

void PixelFlowXY::update(PixelFlowXY *other_flow)
{
	updateWithoutFlow(other_flow);
	updateFlow();	
}

void PixelFlowXY::update(IplImage* image)
{
	updateWithoutFlow(image);
	updateFlow();	
}

void PixelFlowXY::updateWithoutFlow(IplImage* image)
{
	unsigned char* imageData = (unsigned char*) image->imageData;

	memset(sumx,0,width*sizeof(unsigned int));
	memset(sumy,0,height*sizeof(unsigned int));
	
	for(int i=0;i<height;i++)
	{
		int index = i*width;
		for(int j=0;j<width;j++)
		{
			unsigned char value = imageData[index+j];
			sumx[j] += value;			
			sumy[i] += value;				
		}
	}
	/*
	if( image->origin == IPL_ORIGIN_BL)
	{
		for(int i=0;i<height;i++)
		{
			int index = i*width;
			for(int j=0;j<width;j++)
			{
				unsigned char value = imageData[index+j];
				sumx[j] += value;			
				sumy[i] += value;				
			}
		}
	} else 
	{	
		for(int i=1;i<=height;i++)
		{
			int index = i*width;
			for(int j=0;j<width;j++)
			{
				unsigned char value = imageData[index+j];
				sumx[j] += value;			
				sumy[height - i] += value;				
			}
		}
	}*/

	
	
	

	
	for(int i=0;i<height;i++)
	{
		sumy[i] /= width;						
	}

	for(int j=0;j<width;j++)
	{		
		sumx[j] /= height;			
	}

	unsigned int* tmp = sumx_old;
	sumx_old = sumx;
	sumx = tmp;
	
	tmp = sumy_old;
	sumy_old = sumy;
	sumy = tmp;	
}
void PixelFlowXY::updateWithoutFlow(PixelFlowXY *other_flow)
{
	memset(sumx,0,width*sizeof(unsigned int));
	memset(sumy,0,height*sizeof(unsigned int));

	for(int i=0;i<height;i++)
	{
		sumy[i] = (other_flow->sumy_old[i*2] + other_flow->sumy_old[i*2+1])/2;
		sumy_old[i] = (other_flow->sumy[i*2] + other_flow->sumy[i*2+1])/2;	
	}
	
	for(int i=0;i<width;i++)
	{
		sumx[i] = (other_flow->sumx_old[i*2] + other_flow->sumx_old[i*2+1])/2;
		sumx_old[i] = (other_flow->sumx[i*2] + other_flow->sumx[i*2+1])/2;				
	}	

	unsigned int* tmp = sumx_old;
	sumx_old = sumx;
	sumx = tmp;
	
	tmp = sumy_old;
	sumy_old = sumy;
	sumy = tmp;	
}
void PixelFlowXY::updateFlow()
{
	computeShift(sumx_old,sumx,width,shiftX,badnessX);
	maximumBadnessX = maximumBadness;
	computeShift(sumy_old,sumy,height,shiftY,badnessY);
	maximumBadnessY = maximumBadness;
	
	
}

void PixelFlowXY::updateFlow(const int startIndexX, const int startIndexY, const int maxShift)
{
	computeShift(sumx_old,sumx,width,shiftX,badnessX,maxShift, startIndexX);
	maximumBadnessX = maximumBadness;
	computeShift(sumy_old,sumy,height,shiftY,badnessY,maxShift, startIndexY);
	maximumBadnessY = maximumBadness;
}

void PixelFlowXY::getFlowXY(int &shiftX, int &shiftY, unsigned int &badnessX, unsigned int &badnessY)
{
	shiftX = this->shiftX;
	badnessX = this->badnessX;
	shiftY = this->shiftY;
	badnessY = this->badnessY;
	
}

unsigned int PixelFlowXY::getMaximumBadnessX()
{
	return maximumBadnessX;
}

unsigned int PixelFlowXY::getMaximumBadnessY()
{
	return maximumBadnessY;
}

}
}