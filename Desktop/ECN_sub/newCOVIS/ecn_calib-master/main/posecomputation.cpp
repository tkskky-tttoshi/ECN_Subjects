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

    const string fig="../images/im0.jpg";

    //GridTracker tracker;      // this tracker detects a 6x6 grid of points
    CBTracker tracker(8,6);     // this one is to be given the chessboard dimension (8x6)

    // read images while the corresponding file exists
    // images are displayed to ensure the detection was performed


    std::ifstream testfile(fig);
    Pattern pat;
    if(testfile.good())
    {

        testfile.close();

        pat.im =  cv::imread(fig);
        tracker.detect(pat.im, pat.point);
        pat.window = "im0.jpg";
        // draw extraction results
        drawSeq(pat.window, pat.im, pat.point);
        waitKey(0);

    }

    const double pxy=0.5*(pat.im.rows+pat.im.cols);
    bool _reset=true;
    PerspectiveCamera cam(pxy,pxy,0.5*pat.im.cols,0.5*pat.im.rows);
    vpHomogeneousMatrix M;
    M.buildFrom(
        0,0.,0.5,                                                                                   // translation
        0,0,atan2(pat.point[5].y-pat.point[0].y, pat.point[5].x-pat.point[0].x));   // rotation

    cout << "clear 1" << endl;
    // initiate virtual visual servoing with inter-point distance and pattern dimensions
    VVS vvs(cam, 0.03, 8, 6);
    cout << "clear 2" << endl;
    vvs.computePose(pat,M,_reset);
    cout << "clear 3" << endl;
        // print results
    cout << "Final calibration: " << cam.xi_.t() << endl;

    // this will wait for a key pressed to stop the program
    waitKey(0);
}
