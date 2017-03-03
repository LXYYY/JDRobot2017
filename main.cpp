// ConsoleApplication1.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "CV/CVClass.h"
#include "Threads/Threads.h"

#define Calibrate
#define ShowUndistored
using namespace cv;
using namespace std;

CVClass cvClass;
Communication communicator;


int main() {
//    createThread();
    cvClass.camInit(false);
//#ifdef Calibrate
////    cvClass.camCalib();
////    return 0;
////    cvClass.stereoCalib();
//#endif
//
    if (!cvClass.camParamInit()) {
        cout << "camParamInit failed" << endl;
        getchar();
        return 0;
    }
    cvClass.aprilTags.setTagCodes("16h5");
    if (!cvClass.aprilTags.setup()) {
        cout << "apriltags set failed" << endl;
        return 0;
    }
    while (1) {
        cvClass.getImage();
////#ifdef ShowUndistored
////        cvClass.undistortFrame();
////        cvClass.showUndistortedImage();
////#endif
        if (!cvClass.worldCSInited) {
            cvClass.worldCSInited = cvClass.worldCSInit();
            continue;
        }

        vector<CVClass::objBoxImg> objBoxL;
        vector<CVClass::objBoxImg> objBoxR;
        bool foundL = cvClass.processSingle(cvClass.frameL, false, objBoxL);
        bool foundR = cvClass.processSingle(cvClass.frameR, true, objBoxR);
        if (foundL && foundR) {
            vector<Mat> pts3d;
            vector<Point> pts2dL, pts2dR;

            objBoxL.at(0).sortPts();
            objBoxR.at(0).sortPts();
            pts2dL.push_back(objBoxL.at(0).pts.at(0));
            pts2dL.push_back(objBoxL.at(0).pts.at(objBoxL.at(0).pts.size() - 1));

            pts2dR.push_back(objBoxR.at(0).pts.at(0));
            pts2dR.push_back(objBoxR.at(0).pts.at(objBoxR.at(0).pts.size() - 1));

            vector<Mat> ptsW;
//            cout<<"Points"<<(Mat)pts2dL<<(Mat)pts2dR<<endl;
            cvClass.getPoint3d(pts2dL, pts2dR, pts3d);
            cvClass.getPointWorld(pts3d, ptsW);
            for (size_t i = 0; i < pts3d.size(); i++) {
                cout << ptsW.at(i) << endl;
                cout << "test" << pts3d.at(i) << endl;
                cout << "dist=" << norm(pts3d.at(i) - cvClass.camParam.T2W) << endl;
                cout << "dist to origin=" << norm(ptsW.at(i)) << endl;
                bool repeated = false;
                for (size_t j = 0; j < communicator.PtsToSend.size(); j++) {
                    if (communicator.PtsToSend.at(j).first == i) {
                        repeated = true;
                    }
                }
                if (!repeated) {
                    communicator.PtsToSend.push_back(
                            pair<int, Point3d>
                                    (i, Point3d(ptsW.at(i).at<double>(0),
                                                ptsW.at(i).at<double>(1),
                                                ptsW.at(i).at<double>(1))));
                }
            }
//            cout<<"dist="<<norm(pts3d.at(0)-pts3d.at(1))<<endl;
        }

        cvClass.showImage();
        char c = cvClass.waitKeyProc(1);
        if (c == 'q') break;
    }
    while (1);
    return 0;
}

