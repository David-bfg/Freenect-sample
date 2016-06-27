/**********************************************************************
Copyright ©2015 Advanced Micro Devices, Inc. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

.   Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
.   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

/**
*******************************************************************************
* @file <CartoonFilter.cpp>
* @brief Show interops by using cartoon filter. Implementation file.
*******************************************************************************
*/

#include "CartoonFilter.hpp"
#include "libfreenect.hpp"
//#include <GL/glut.h>
//#include <pthread.h>

using namespace std;


class myMutex {
	public:
		myMutex() {
			pthread_mutex_init( &m_mutex, NULL );
		}
		void lock() {
			pthread_mutex_lock( &m_mutex );
		}
		void unlock() {
			pthread_mutex_unlock( &m_mutex );
		}
	private:
		pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_11BIT),
			m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
			m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
			rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
			ownMat(Size(640,480),CV_8UC3,Scalar(0)) {

			for( unsigned int i = 0 ; i < 2048 ; i++) {
				float v = i/2048.0;
				v = std::pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
		}

		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
//			std::cout << "RGB callback" << std::endl;
			m_rgb_mutex.lock();
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			rgbMat.data = rgb;
			m_new_rgb_frame = true;
			m_rgb_mutex.unlock();
		};

		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
//			std::cout << "Depth callback" << std::endl;
			m_depth_mutex.lock();
			uint16_t* depth = static_cast<uint16_t*>(_depth);
			depthMat.data = (uchar*) depth;
			m_new_depth_frame = true;
			m_depth_mutex.unlock();
		}

		bool getVideo(Mat& output) {
			m_rgb_mutex.lock();
			if(m_new_rgb_frame) {
				cv::cvtColor(rgbMat, output, CV_RGB2BGR);
				m_new_rgb_frame = false;
				m_rgb_mutex.unlock();
				return true;
			} else {
				m_rgb_mutex.unlock();
				return false;
			}
		}

		bool getDepth(Mat& output) {
				m_depth_mutex.lock();
				if(m_new_depth_frame) {
					uint16_t* depth = (uint16_t*) depthMat.data;
					for( unsigned int i = 0 ; i < 640*480 ; i++) {
						int pval = m_gamma[depth[i]];
						int lb = pval & 0xff;
						switch (pval>>8) {
						case 0:
							output.data[3*i+0] = 255-lb;
							output.data[3*i+1] = 255-lb;
							output.data[3*i+2] = 255;
							break;
						case 1:
							output.data[3*i+0] = 0;
							output.data[3*i+1] = lb;
							output.data[3*i+2] = 255;
							break;
						case 2:
							output.data[3*i+0] = 0;
							output.data[3*i+1] = 255;
							output.data[3*i+2] = 255-lb;
							break;
						case 3:
							output.data[3*i+0] = lb;
							output.data[3*i+1] = 255;
							output.data[3*i+2] = 0;
							break;
						case 4:
							output.data[3*i+0] = 255;
							output.data[3*i+1] = 255-lb;
							output.data[3*i+2] = 0;
							break;
						case 5:
							output.data[3*i+0] = 255-lb;
							output.data[3*i+1] = 0;
							output.data[3*i+2] = 0;
							break;
						default:
							output.data[3*i+0] = 0;
							output.data[3*i+1] = 0;
							output.data[3*i+2] = 0;
							break;
						}
					}
					m_new_depth_frame = false;
					m_depth_mutex.unlock();
					return true;
				} else {
					m_depth_mutex.unlock();
					return false;
				}
			}
	private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		Mat depthMat;
		Mat rgbMat;
		Mat ownMat;
		myMutex m_rgb_mutex;
		myMutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};

/**
*******************************************************************************
* @fn convertToString
* @brief Convert character array to std::string.
*
* @param filename The character array.
* @param str std::string.
*******************************************************************************
*/
int convertToString(
        const char *filename, 
        std::string& str);

/**
*******************************************************************************
* Implementation of convertToString                                           *
******************************************************************************/
int convertToString(const char *filename, std::string& s)
{
    size_t size;
    char*  str;

    // create a file stream object by filename
    std::fstream f(filename, (std::fstream::in | std::fstream::binary));


    if(!f.is_open())
    {
     	return SDK_FAILURE;   
    }
    else
    {
        size_t fileSize;
        f.seekg(0, std::fstream::end);
        size = fileSize = (size_t)f.tellg();
        f.seekg(0, std::fstream::beg);

        str = new char[size+1];
        if(!str)
        {
            f.close();
            return SDK_FAILURE;
        }

        f.read(str, fileSize);
        f.close();
        str[size] = '\0';

        s = str;
        delete[] str;
        return SDK_SUCCESS;
    }
}

/******************************************************************************
* Implementation of CartoonFilter::RGB2Gray_caller()                          *
******************************************************************************/
void CartoonFilter::RGB2Gray_caller(const oclMat &oclSrc, oclMat &oclDst,
                                    int bidx)
{
    int channels = oclSrc.oclchannels();
    size_t globalThreads[3] = {oclSrc.cols, oclSrc.rows, 1};
    size_t localThreads[3] = {16, 16, 1};
    char build_options[50];
    sprintf (build_options, "-DDEPTH_%d", oclSrc.depth());

    // args
    vector<pair<size_t, const void *> > args;
    args.push_back(make_pair(sizeof(cl_int), (void *)&oclSrc.cols));
    args.push_back(make_pair(sizeof(cl_int), (void *)&oclSrc.rows));
    args.push_back(make_pair(sizeof(cl_int), (void *)&oclSrc.step));
    args.push_back(make_pair(sizeof(cl_int), (void *)&oclDst.step));
    args.push_back(make_pair(sizeof(cl_int), (void *)&channels));
    args.push_back(make_pair(sizeof(cl_int), (void *)&bidx));
    args.push_back(make_pair(sizeof(cl_mem), (void *)&oclSrc.data));
    args.push_back(make_pair(sizeof(cl_mem), (void *)&oclDst.data));

   
	// convert kernel file into string

	string sourceStr;
	int status = convertToString(kernelFile, sourceStr);
	const char *source = sourceStr.c_str();

	program_src1.programStr = source;

	openCLExecuteKernelInterop(oclSrc.clCxt, program_src1, "RGB2Gray", globalThreads, localThreads, args, -1, -1, build_options);
}

/******************************************************************************
* Implementation of CartoonFilter::Gray2RGB_caller()                          *
******************************************************************************/
void CartoonFilter::Gray2RGB_caller(const oclMat &oclSrc, oclMat &oclDst)
{
    size_t globalThreads[3] = {oclSrc.cols, oclSrc.rows, 1};
    size_t localThreads[3] = {16, 16, 1};
    char build_options[50];
    sprintf (build_options, "-DDEPTH_%d", oclSrc.depth());

    // args
    vector<pair<size_t, const void *> > args;
    args.push_back( make_pair( sizeof(cl_int) , (void *)&oclSrc.cols));
    args.push_back( make_pair( sizeof(cl_int) , (void *)&oclSrc.rows));
    args.push_back( make_pair( sizeof(cl_int) , (void *)&oclSrc.step));
    args.push_back( make_pair( sizeof(cl_int) , (void *)&oclDst.step));
    args.push_back( make_pair( sizeof(cl_mem) , (void *)&oclSrc.data));
    args.push_back( make_pair( sizeof(cl_mem) , (void *)&oclDst.data));
  
	// convert kernel file into string
	string sourceStr;
	int status = convertToString(kernelFile, sourceStr);
	const char *source = sourceStr.c_str();

	program_src2.programStr = source; 


	openCLExecuteKernelInterop(oclSrc.clCxt, program_src2,  "Gray2RGB",
                               globalThreads, localThreads, args, -1, -1, build_options);
}

/******************************************************************************
* Implementation of CartoonFilter::myCvtColor()                               *
******************************************************************************/
void CartoonFilter::myCvtColor(const oclMat &oclSrc, oclMat &oclDst, int code)
{
    Size size = oclSrc.size();
    int depth = oclSrc.depth();
    int channels = oclSrc.oclchannels();

    CV_Assert(depth == CV_8U || depth == CV_16U || depth == CV_32F);

    switch(code)
    {
    case CV_BGR2GRAY:
    case CV_BGRA2GRAY:
    {
        CV_Assert(channels == 3 || channels == 4);
        int bidx = ((code == CV_BGR2GRAY || CV_BGRA2GRAY) ? 0 : 2);
        oclDst.create(size, CV_MAKETYPE(depth, 1));
        RGB2Gray_caller(oclSrc, oclDst, bidx);
        break;
    }
    case CV_GRAY2BGR:
    case CV_GRAY2BGRA:
    {
        int dcn = (code == CV_GRAY2BGRA ? 4 : 3);
        oclDst.create(size, CV_MAKETYPE(depth, dcn));
        Gray2RGB_caller(oclSrc, oclDst);
        break;
    }
    default:
        CV_Error(SDK_FAILURE, "No convert rule");
        return;
    }
}

/******************************************************************************
* Implementation of CartoonFilter::overlapingEdges()                          *
******************************************************************************/
void CartoonFilter::overlapingEdges(const oclMat &oclSrc, oclMat &oclDst, int pixelBuff)
{
    Size size = oclSrc.size();
    int depth = oclSrc.depth();
    int channels = oclSrc.oclchannels();

    CV_Assert(depth == CV_8U || depth == CV_16U || depth == CV_32F);

	CV_Assert(channels == 1);
	oclDst.create(size, CV_MAKETYPE(depth, 1));

	size_t globalThreads[3] = {oclSrc.cols, oclSrc.rows, 1};
	size_t localThreads[3] = {16, 16, 1};
	char build_options[50];
	sprintf (build_options, "-DDEPTH_%d", oclSrc.depth());

	// args
	vector<pair<size_t, const void *> > args;
	args.push_back(make_pair(sizeof(cl_int), (void *)&oclSrc.cols));
	args.push_back(make_pair(sizeof(cl_int), (void *)&oclSrc.rows));
	args.push_back(make_pair(sizeof(cl_int), (void *)&oclSrc.step));
	args.push_back(make_pair(sizeof(cl_int), (void *)&oclDst.step));
	args.push_back(make_pair(sizeof(cl_int), (void *)&channels));
	args.push_back(make_pair(sizeof(cl_int), (void *)&pixelBuff));
	args.push_back(make_pair(sizeof(cl_mem), (void *)&oclSrc.data));
	args.push_back(make_pair(sizeof(cl_mem), (void *)&oclDst.data));


	// convert kernel file into string

	string sourceStr;
	int status = convertToString(kernelFile, sourceStr);
	const char *source = sourceStr.c_str();

	program_src1.programStr = source;

	openCLExecuteKernelInterop(oclSrc.clCxt, program_src1, "OverlapingEdges", globalThreads, localThreads, args, -1, -1, build_options);
}

/******************************************************************************
* Implementation of CartoonFilter::run()                                      *
******************************************************************************/
int CartoonFilter::run()
{

	bool die(false);
	string filename("snapshot");
	string suffix(".png");
	int i_snap(0),iter(0);

	Mat depthMat(Size(640,480),CV_8UC3);
	Mat depthf(Size(640,480),CV_8UC3);
	cv::ocl::oclMat oclDepthMat(Size(640,480),CV_8UC1);
	cv::ocl::oclMat scale(Size(615,461),CV_8UC1);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
	Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

	// The next two lines must be changed as Freenect::Freenect
	// isn't a template but the method createDevice:
	// Freenect::Freenect<MyFreenectDevice> freenect;
	// MyFreenectDevice& device = freenect.createDevice(0);
	// by these two lines:

	Freenect::Freenect freenect;
	MyFreenectDevice& kinect = freenect.createDevice<MyFreenectDevice>(0);

	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);
	kinect.startVideo();
	kinect.startDepth();
	int x = 6, y = 43;
	while (!die) {
		kinect.getVideo(rgbMat);
		kinect.getDepth(depthMat);
	    cv::ocl::oclMat oclCloneMat;

	    /****************************************************************************
	    * Mean shift filter.                                                        *
	    ****************************************************************************/
//	    cv::cvtColor(rgbMat, cvSrcMat, CV_BGRA2BGR);
	    oclSrcMat = rgbMat;  // Interop: OpenCV to OpenCV-CL.

	    oclCloneMat = oclSrcMat.clone();

	    cv::ocl::meanShiftFiltering(oclCloneMat, oclSrcMat, 10, 30);
	    cvDstMat = oclSrcMat;  // Interop: OpenCV-CL to OpenCV.

	    /****************************************************************************
	    * Canny.                                                                    *
	    ****************************************************************************/
	    oclCannyMat = rgbMat;
	    // Interop: OpenCV to Raw OpenCL kernel.
	    cv::ocl::cvtColor(oclCannyMat.clone(), oclCannyMat, CV_BGRA2GRAY);

	    oclCloneMat = oclCannyMat.clone();// Interop: Raw OpenCL kernel to OpenCV-CL.
	    cv::ocl::Canny(oclCloneMat, oclCannyMat, 150, 150);

	    // diff
//	    cvDstMat = cvDstMat - cvCannyDstMat;
		cv::imshow("rgb", cvDstMat);

		oclDepthMat = depthMat;
	    // Interop: OpenCV to Raw OpenCL kernel.
	    cv::ocl::cvtColor(oclDepthMat.clone(), oclDepthMat, CV_BGRA2GRAY);

	    oclCloneMat = oclDepthMat.clone();// Interop: Raw OpenCL kernel to OpenCV-CL.
	    cv::ocl::Canny(oclCloneMat, oclDepthMat, 120, 200);

	    int cuty;
	    cv::ocl::resize(oclDepthMat, scale, Size(592,444));
	    cuty = oclDepthMat.rows - scale.rows - y;
	    cuty = cuty < 0 ? cuty :0 ;
	    scale = scale(cv::Rect(0, 0, scale.cols, scale.rows + cuty));
	    oclDepthMat.setTo(Scalar(0));
	    cv::ocl::oclMat rect = oclDepthMat(cv::Rect(x, y, scale.cols, scale.rows));
	    scale.copyTo(rect);

	    overlapingEdges(oclDepthMat, oclCannyMat, 3);

	    // Interop: OpenCV-CL to Raw OpenCL kernel.
	    cv::ocl::cvtColor(oclCannyMat.clone(), oclCannyMat, CV_GRAY2BGRA);
	    cvCannyDstMat = oclCannyMat;  // Interop: Raw OpenCL kernel to OpenCV.

	    // Interop: OpenCV-CL to Raw OpenCL kernel.
	    cv::ocl::cvtColor(oclDepthMat.clone(), oclDepthMat, CV_GRAY2BGRA);
	    depthf = oclDepthMat;  // Interop: Raw OpenCL kernel to OpenCV.
	    cvDstMat = cvDstMat - cvCannyDstMat;
		cv::imshow("rgb", cvDstMat);
		cv::imshow("depth", cvCannyDstMat);
		char k = cvWaitKey(5);
		if( k == 27 ){
			cvDestroyWindow("rgb");
			cvDestroyWindow("depth");
			break;
		}
//		if( k == 81 ){
//			if(x > 0)
//				x--;
//		}
//		if( k == 82 ){
//			y--;
//		}
//		if( k == 83 ){
//			x++;
//		}
//		if( k == 84 ){
//			y++;
//		}
		if( k == 8 ) {
			std::ostringstream file;
			file << filename << i_snap << suffix;
			cv::imwrite(file.str(),rgbMat);
			i_snap++;
		}
		iter++;
	}

	kinect.stopVideo();
	kinect.stopDepth();
    return SDK_SUCCESS;
}

int main( int argc, char** argv )
{
    try
    {
        CartoonFilter Itrp;

        if (Itrp.sampleArgs->initialize() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if (Itrp.sampleArgs->parseCommandLine(argc, argv) != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }

        if (Itrp.run() != SDK_SUCCESS)
        {
            return SDK_FAILURE;
        }
    }
    catch(cv::Exception &e)
    {
        const char *errMsg = e.what();
        cout << "Exception encountered with message: " << errMsg << endl;
    }

    return SDK_SUCCESS;
}
