#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpPoint.h>
#include <visp/vpSubColVector.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpIoTools.h>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>

#include <vvs.h>
#include <grid_tracker.h>
#include <perspective_camera.h>
#include <distortion_camera.h>
#include <cb_tracker.h>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::stringstream;
using cv::waitKey;
using namespace covis;

int main()
{


    cv::VideoCapture cap("../images/video1.mp4");

    while(!cap.isOpened())
        std::cout<<"Error happened"<<std::endl;
        return -1;

    cv::Mat frame;
    while(1){
        cap>>frame;
        cv::imshow("On air", frame);
    }



//    while(true)
//    {
//        cv::Mat frame;
//        cap>>frame;


//        stringstream ss;
//        ss << prefix << patterns.size() << ".jpg";
//        std::ifstream testfile(base + ss.str());
//        if(testfile.good())
//        {

//            testfile.close();
//            Pattern pat;
//            pat.im =  cv::imread(base + ss.str());
//            tracker.detect(pat.im, pat.point);
//            pat.window = ss.str();
//            // draw extraction results
//            drawSeq(pat.window, pat.im, pat.point);
//            patterns.push_back(pat);
//            waitKey(0);

//        }
//        else
//            break;


//    }
//    cout << "Found " << patterns.size() << " images" << endl;

//    //this part just sets an initial parameter such that px, py
//    const double pxy=0.5*(patterns.front().im.rows+patterns.front().im.cols);
//    PerspectiveCamera cam(pxy,pxy,0.5*patterns.front().im.cols,0.5*patterns.front().im.rows);


//    // create a camera model (Perspective or Distortion)
//    // default parameters should be guessed from image dimensions
//    //PerspectiveCamera cam(1,1,1,1);   // not a very good guess

//      cout << "clear 1" << endl;
//    // initiate virtual visual servoing with inter-point distance and pattern dimensions
//    VVS vvs(cam, 0.03, 8, 6);
//cout << "clear 2" << endl;
//    // calibrate from all images
//    vvs.calibrate(patterns);
//cout << "clear 3" << endl;
//    // print results
//    cout << "Final calibration: " << cam.xi_.t() << endl;

    // this will wait for a key pressed to stop the program
    waitKey(0);
}
