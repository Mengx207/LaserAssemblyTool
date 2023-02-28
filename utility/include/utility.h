#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <pylon/InstantCamera.h>
#include <pylon/PylonIncludes.h>
#include <GenApi/IEnumeration.h>
#include <pylon/EnumParameter.h>
#include <Base/GCString.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include <stdio.h>
#include <ctime>

namespace laserdot
{
int PixelCounter(Mat img, int count)
{
	Mat img_nominal;
	img.convertTo(img_nominal, CV_32F);
	for(int i=0; i<img_nominal.rows; i++)
	{
		for(int j=0; j<img_nominal.cols; j++)
		{
			if(img_nominal.at<float>(i,j)<=0.0)
			{
				count++;
				//cout<<img_nominal.at<float>(i,j)<<endl;
			}
		}
	}
	return count;
}

int SizeAverage (int count, int size_avg, int size_array[], int center_total)
{
	for(int i=9; i>0; i--)
	{
		size_array[i] = size_array[i-1];
	}
	size_array[0] = count;

	int size_sum = 0;
	for(int i=0; i<10; i++)
	{
		size_sum = size_sum + size_array[i];
	}

	if(size_array[9]!=0 && center_total > 50)
	{
		size_avg = size_sum/10;
		return size_avg;
	}
}
}