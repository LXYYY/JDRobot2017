// ConsoleApplication1.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include "CV/CVClass.h"

using namespace cv;
using namespace std;

CVClass cvClass;


int main()
{
	cvClass.camInit(true);
//    while(1)
//    {
//        double time=static_cast<double>(getTickCount());
//
//        cvClass.camL.getImage();
//        //imshow("test",cvClass.frameL);
////        waitKey(1);
//        time=static_cast<double>(getTickCount()-time)/getTickFrequency();
//        cout<<"time="<<time<<endl;
//    }
//    if (cvClass.camParamInit())
//	{
//		cout << "camParamInit failed" << endl;
//		getchar();
//		return 0;
//	}
	bool worldCSInited = false;

    while (1)
	{
//		cvClass.getImage();
        cvClass.camL.getImage();
        //cvClass.showImage();
        waitKey(1);
        continue;
        if (!worldCSInited)
		{
			worldCSInited = cvClass.worldCSInit();
			continue;
		}
		cvClass.undistortFrame();
		cvClass.showImage();
		cvClass.showUndistortedImage();
		vector<CVClass::objBoxImg> objBoxL;
		vector<CVClass::objBoxImg> objBoxR;
		if (!cvClass.processSingle(cvClass.rFrameL, false, objBoxL)
			&& !cvClass.processSingle(cvClass.rFrameR, true, objBoxR))
		{
			//if (!cvClass.processSingle(cvClass.frameL, false, objBoxL)
			//	&& !cvClass.processSingle(cvClass.frameR, true, objBoxR))
			//{
			//cout << "objBoxL.size()=" << objBoxL.size() << endl;
			//for (int i = 0;i < objBoxL.size();i++)
			//{
			//	objBoxL.at(i).print();
			//}
			//for (int i = 0;i < objBoxR.size();i++)
			//{
			//	objBoxR.at(i).print();
			//}
			vector<Point> pts3d;
			vector<Point> pts2dL, pts2dR;
			//if (objBoxL.size() == objBoxR.size())
			//{
			//	for (int i = 0;i < objBoxL.size();i++)
			//	{
			//		objBoxL.at(i).sortPts();
			//		objBoxR.at(i).sortPts();
			//		pts2dL.push_back(objBoxL.at(i).pts.at(0));
			//		pts2dR.push_back(objBoxR.at(i).pts.at(0));
			//	}
			//}

			objBoxL.at(0).sortPts();
			objBoxR.at(0).sortPts();
			pts2dL.push_back(objBoxL.at(0).pts.at(0));
			pts2dL.push_back(objBoxL.at(0).pts.at(objBoxL.at(0).pts.size() - 1));

			pts2dR.push_back(objBoxR.at(0).pts.at(0));
			pts2dR.push_back(objBoxR.at(0).pts.at(objBoxR.at(0).pts.size() - 1));

			cvClass.getPoint3d(pts2dL, pts2dR, pts3d);

		}
		waitKey(1);
	}
	return 0;
}

