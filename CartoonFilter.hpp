/**
*******************************************************************************
* @file <CartoonFilter.hpp>
* @brief Show interops by using cartoon filter. Class declaration.
*******************************************************************************
*/

#define NOMINMAX
#include <CL/cl.hpp>

#include <stdio.h>
#include <iostream>
#include <fstream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/operations.hpp>

// for OPENCV-CL
#include "opencv2/ocl/ocl.hpp"

using namespace cv;
using namespace cv::ocl;
using namespace std;

/*!<OpenCL kernel file name.*/
const char *kernelFile = "CartoonFilter_Kernels.cl";

cv::ocl::ProgramSource program("OverlapingEdges",NULL, NULL);

/**
* CartoonFilter
* To show the various interops between OpenCV, OpenCL and OpenCV-CL.
*/
class CartoonFilter
{
        cv::Mat cvDstMat;    /*!<The output cv::Mat obtained after filtering.*/
        cv::Mat cvCannyDstMat;  /*!<The output canny matrix in cv::Mat.*/

        cv::ocl::oclMat oclSrcMat;  /*!<The input image in cv::ocl::oclMat format.*/
        cv::ocl::oclMat oclCannyMat;/*!<The Canny input mat in
                               *  cv::ocl::oclMat format*/

    public:

        /**
        * Constructor
        * Initialize OpenCVCommandArgs and SDKTimer objects, default image name
        */
        CartoonFilter() {}

        /** Destructor
        */
        ~CartoonFilter() {}

        void overlapingEdges(const oclMat &oclSrc, oclMat &oclDst, int pixelBuff);

        /**
        ***************************************************************************
        * @fn run
        * @brief Run the filter.
        *
        * @return int SDK_SUCCESS on success and nonzero on failure.
        ***************************************************************************
        */
        void run();
};
