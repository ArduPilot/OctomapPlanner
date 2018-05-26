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

		    numberOfDisparities = ((640/8) + 15) & -16;

		    sgbm = cv::StereoSGBM::create(0,16,3);
		    sgbm->setPreFilterCap(10);
		    int sgbmWinSize = 9;
		    sgbm->setBlockSize(sgbmWinSize);

		    int cn = 3; //Number of channels

		    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
		    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
		    sgbm->setMinDisparity(0);
		    sgbm->setNumDisparities(numberOfDisparities);
		    sgbm->setUniquenessRatio(10);
		    sgbm->setSpeckleWindowSize(100);
		    sgbm->setSpeckleRange(32);
		    sgbm->setDisp12MaxDiff(1);
	        // sgbm->setMode(cv::StereoSGBM::MODE_HH);
	        sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
	        // sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
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
	cv::Size img_size = img1.size();
	cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q);

	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

	cv::cvtColor(img1, img1, CV_BGR2GRAY);
	cv::cvtColor(img2, img2, CV_BGR2GRAY);
	remap(img1, img1r, map11, map12, cv::INTER_LINEAR);
	remap(img2, img2r, map21, map22, cv::INTER_LINEAR);


	sgbm->compute(img1r, img2r, disp);
	disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
	// normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

	return disp8;
}

cv::Mat StereoMatcher::getPointcloud(void)
{
	reprojectImageTo3D(disp, points, Q, true);
	return points;
}
