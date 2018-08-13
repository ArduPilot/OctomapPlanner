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

#ifndef STEREO_MATCHER_HH_
#define STEREO_MATCHER_HH_

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

class StereoMatcher
{
	public:
		StereoMatcher(const std::string&);
		cv::Mat matchPair(cv::Mat img1, cv::Mat img2);
		cv::Mat getPointcloud(void);
	private:

		int SADWindowSize, numberOfDisparities;
		bool no_display;
		float scale;

		cv::Ptr<cv::StereoSGBM> sgbm;

		cv::Size img_size;
		cv::Mat M1, D1, M2, D2, R, T, R1, P1, R2, P2, map11, map12, map21, map22, img1r, img2r, disp, disp8, Q, points;

};

#endif