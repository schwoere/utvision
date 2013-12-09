#include "AbstractPixelFlow.h"

#include <algorithm>



using namespace std;

namespace Ubitrack { namespace Vision {

AbstractPixelFlow::AbstractPixelFlow(int width, int height, int maxShift) : width(width), height(height), maxShift(maxShift)
{
	maximumBadness = 0;
}

AbstractPixelFlow::~AbstractPixelFlow(void)
{
}

void AbstractPixelFlow::computeShift(unsigned int *sum, unsigned int *sum_old, int length, int &shift, unsigned int &badness)
{
	computeShift(sum,sum_old,length,shift,badness,maxShift,0);
}

void AbstractPixelFlow::computeShift(unsigned int *sum, unsigned int *sum_old, int length, int &shift, unsigned int &badness, const int maximumShift, const int startingPoint)
{
	maximumBadness = 0;

	unsigned int gbadness=((unsigned int)~((unsigned int)0));
	int gshift=0;

	for(int i=-maximumShift;i<=maximumShift;i++)
	{
		int s = startingPoint+i;
		unsigned int localbadness = computeBadness(sum, sum_old, length, s);		
		maximumBadness = max(maximumBadness ,localbadness);
		if(localbadness < gbadness)
		{
			gbadness = localbadness;
			gshift = s;
		}
	}

	badness = gbadness;
	shift = gshift;
}

unsigned int AbstractPixelFlow::computeBadness(unsigned int *sum, unsigned int *sum_old, int length, int shift)
{

	// start index im alten array
	int t;
	int start;
	t = shift;
	// wenn shift negativ
	if(t < 0)
	{
		// starte am unteren ende des alten arrays
		start = 0;
	} else
	{
		// sonst starte am unteren ende des neuen arrays
		start = t;
	}
	// ende index im alten array
	int end;
	t = length + shift;
	// wenn ende hinter dem ende des alten arrays
	if(t > length)
	{
		// gehe bis zum ende des alten arrays
		end = length;
	} else 
	{
		// sonst bis zum ende des neuen arrays
		end = t;
	}
		
	unsigned int badness=0;
	// für alle punkte bei denen es einträge in beiden arrays gibt
	for(int i=start;i<end;i++)
	{
		int value = sum_old[i]-sum[i-shift];
		badness += value*value;
	}

	int countOfElements = (end - start);
	badness = badness / countOfElements;

	return badness;	
}

unsigned int AbstractPixelFlow::computeBadness2(unsigned int *sum, unsigned int *sum_old, int length, int shift)
{		
	unsigned int badness=0;
	
	for(int i=0;i<length;i++)
	{
		int index = i - shift;
		if(index < 0) 
			index += length;
		else if(index >= length)
		{
			index -= length;
		}
		int value = sum_old[i]-sum[index];
		badness += value*value;
	}
	
	badness = badness / length;

	return badness;	
}

unsigned int AbstractPixelFlow::getMaximumBadness()
{
	return maximumBadness;
}

}
}