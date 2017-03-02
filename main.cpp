// ConsoleApplication1.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "CV/CVClass.h"

#define Calibrate
//#define ShowUndistored
using namespace cv;
using namespace std;

CVClass cvClass;


int main() {
    cvClass.camInit(false);
#ifdef Calibrate
//    cvClass.camCalib();
    //return 0;
//    cvClass.stereoCalib();
#endif
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
        if (!cvClass.worldCSInited) {
            cvClass.worldCSInited = cvClass.worldCSInit();
            continue;
        }
#ifdef ShowUndistored
        cvClass.undistortFrame();
        cvClass.showUndistortedImage();
#endif
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

            pts2dR.push_back(objBoxR.at(0).pts.at(0));

            vector<Mat> ptsW;
            cout<<"Points"<<(Mat)pts2dL<<(Mat)pts2dR<<endl;
            cvClass.getPoint3d(pts2dL, pts2dR, pts3d);
            cvClass.getPointWorld(pts3d, ptsW);
            for (size_t i = 0; i < ptsW.size(); i++) {
                cout << ptsW.at(i) << endl;
//                cout <<"test"<< pts3d.at(i) << endl;
            }
        }
        cvClass.showImage();
        char c = waitKey(1);
        if (c == 'q') break;
    }
    return 0;
}

