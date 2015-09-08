//
//  estimateRigidTransform.h
//  Footage_Manipulation
//
//  Created by Bryan Anenberg on 8/24/15.
//  Copyright (c) 2015 Bryan Anenberg. All rights reserved.
//
//  Reimplementation cv::estimateRigidTransform

#ifndef __ert_h
#define __ert_h

#include <opencv2/opencv.hpp>

cv::Mat estimateRigidTransformRansac( cv::InputArray src1,
                                     cv::InputArray src2,
                                     bool fullAffine, int ransac_max_iters, double ransac_good_ratio);


#endif
