/*
*   OctomapPlanner
*
*   Copyright (C) 2018  ArduPilot
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*   Author Ayush Gaud <ayush.gaud[at]gmail.com>
*/

#include "stereo_matcher.h"
#include <iostream>

StereoMatcher::StereoMatcher(const std::string &parameters_filename)
{
	if( !parameters_filename.empty() )
	{
	    // reading intrinsic parameters
	    cv::FileStorage fs(parameters_filename, cv::FileStorage::READ);
	    if(!fs.isOpened())
	    {
	        printf("Failed to open file %s\n", parameters_filename.c_str());
	        throw -1;
	    }
	    try
	    {
		    fs["M1"] >> M1;
		    fs["D1"] >> D1;
		    fs["M2"] >> M2;
		    fs["D2"] >> D2;
		    fs["R"] >> R;
		    fs["T"] >> T;
    	    
  	    int PreFilterCap = (int)fs["PreFilterCap"];
  	    int sgbmWinSize = (int)fs["sgbmWinSize"];
  	    int MinDisparity = (int)fs["MinDisparity"];
  	    int UniquenessRatio = (int)fs["UniquenessRatio"];
  	    int SpeckleWindowSize =  (int)fs["SpeckleWindowSize"];
  	    int SpeckleRange = (int)fs["SpeckleRange"];
  	    int Disp12MaxDiff = (int)fs["Disp12MaxDiff"];
  	    int TextureThreshold = (int)fs["TextureThreshold"];
  	    int Width = (int)fs["Width"];
  	    int Height = (int)fs["Height"];
  	    scale = (float)fs["Scale"];
  	    
  	    fs.release();

  	    numberOfDisparities = ((Width/8) + 15) & -16;

  	    sgbm = cv::StereoSGBM::create(0,16,3);
  	    sgbm->setPreFilterCap(PreFilterCap);
  	    sgbm->setBlockSize(sgbmWinSize);

  	    int cn = 1; //Number of channels

  	    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
  	    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
  	    sgbm->setMinDisparity(MinDisparity);
  	    sgbm->setNumDisparities(numberOfDisparities);
  	    sgbm->setUniquenessRatio(UniquenessRatio);
  	    sgbm->setSpeckleWindowSize(SpeckleWindowSize);
  	    sgbm->setSpeckleRange(SpeckleRange);
  	    sgbm->setDisp12MaxDiff(Disp12MaxDiff);
        sgbm->setMode(cv::StereoSGBM::MODE_HH);
        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
        // sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

  			img_size = cv::Size(Width, Height);
  			cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q);

  			initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
  			initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
    	}
    	catch(cv::Exception& e )
    	{
    		const char* err_msg = e.what();
		    std::cout << "exception caught: " << err_msg << std::endl;
    	}
	}
}

cv::Mat StereoMatcher::matchPair(cv::Mat img1, cv::Mat img2)
{

	if(img1.type() != 0)
	{
		cv::cvtColor(img1, img1, CV_BGR2GRAY);
		cv::cvtColor(img2, img2, CV_BGR2GRAY);
	}
	remap(img1, img1r, map11, map12, cv::INTER_LINEAR);
	remap(img2, img2r, map21, map22, cv::INTER_LINEAR);

	resize(img1r, img1r, cv::Size(img_size.width*scale, img_size.height*scale));
	resize(img2r, img2r, cv::Size(img_size.width*scale, img_size.height*scale));

	sgbm->compute(img1r, img2r, disp);
	resize(disp, disp, cv::Size(img_size.width, img_size.height));
	disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
	// normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
	return disp8;
}

cv::Mat StereoMatcher::getPointcloud(void)
{
	reprojectImageTo3D(disp, points, Q, true);
	return points;
}