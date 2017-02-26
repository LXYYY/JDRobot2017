#include "CVClass.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
using namespace std;
using namespace cv;

bool CVClass::camParamInit(void)
{
	//intrinsics
	FileStorage paramL("paramL.yaml", FileStorage::READ);
	FileStorage paramR("paramR.yaml", FileStorage::READ);

	if (!paramL.isOpened())
	{
		cout << "paramL.xml not available" << endl;
		return 0;
	}
	if (!paramR.isOpened())
	{
		cout << "paramR.xml not available" << endl;
		return 0;
	}

	cout << (int)paramL["nframes"] << endl;

	paramL["camera_matrix"] >> camParam.cameraMatrix[0];
	paramL["distortion_coefficients"] >> camParam.distCoeffs[0];

	paramR["camera_matrix"] >> camParam.cameraMatrix[1];
	paramR["distortion_coefficients"] >> camParam.distCoeffs[1];

	paramL.release();
	paramR.release();

	getImage();
	if (frameL.size() != frameR.size())
	{
		cout << "imgSizes not equal" << endl;
		return true;
	}
	else
		camParam.imgSize = frameL.size();

	try
	{
		initUndistortRectifyMap(camParam.cameraMatrix[0], camParam.distCoeffs[0], Mat(),
			getOptimalNewCameraMatrix(camParam.cameraMatrix[0], camParam.distCoeffs[0], camParam.imgSize, 1, camParam.imgSize, 0),
			camParam.imgSize, CV_16SC2, camParam.map1[0], camParam.map2[0]);
		initUndistortRectifyMap(camParam.cameraMatrix[1], camParam.distCoeffs[1], Mat(),
			getOptimalNewCameraMatrix(camParam.cameraMatrix[1], camParam.distCoeffs[1], camParam.imgSize, 1, camParam.imgSize, 0),
			camParam.imgSize, CV_16SC2, camParam.map1[1], camParam.map2[1]);
	}
	catch (...)
	{
		cout << "initUndistortedRectifyMap failed " << endl;
		return true;
	}

	//extrinsics
	FileStorage extrinsics("extrinsics.yml", FileStorage::READ);
	if (!extrinsics.isOpened())
	{
		cout << "extrinsics.xml not available" << endl;
		return true;
	}

	extrinsics["R"] >> camParam.R;
	extrinsics["T"] >> camParam.T;
	extrinsics["R1"] >> camParam.R1;
	extrinsics["R2"] >> camParam.R2;
	extrinsics["P1"] >> camParam.P1;
	extrinsics["P2"] >> camParam.P2;
	extrinsics["Q"] >> camParam.Q;

	extrinsics.release();

	return false;
}

bool CVClass::undistortFrame(void)
{
	try
	{
		remap(frameL, rFrameL, camParam.map1[0], camParam.map2[0], INTER_LINEAR);
		remap(frameR, rFrameR, camParam.map1[1], camParam.map2[1], INTER_LINEAR);
	}
	catch (...) 
	{
		cout << "remap failed" << endl;
		return true;
	}
	return false;
}

Mat_<double> LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
	Matx34d P,       //camera 1 matrix
	Point3d u1,      //homogenous image point in 2nd camera
	Matx34d P1       //camera 2 matrix
)
{
	//build matrix A for homogenous equation system Ax = 0
	//assume X = (x,y,z,1), for Linear-LS method
	//which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
	Matx43d A(u.x*P(2, 0) - P(0, 0), u.x*P(2, 1) - P(0, 1), u.x*P(2, 2) - P(0, 2),
		u.y*P(2, 0) - P(1, 0), u.y*P(2, 1) - P(1, 1), u.y*P(2, 2) - P(1, 2),
		u1.x*P1(2, 0) - P1(0, 0), u1.x*P1(2, 1) - P1(0, 1), u1.x*P1(2, 2) - P1(0, 2),
		u1.y*P1(2, 0) - P1(1, 0), u1.y*P1(2, 1) - P1(1, 1), u1.y*P1(2, 2) - P1(1, 2)
	);
	Mat_<double> B = (Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)),
		-(u.y*P(2, 3) - P(1, 3)),
		-(u1.x*P1(2, 3) - P1(0, 3)),
		-(u1.y*P1(2, 3) - P1(1, 3)));

	Mat_<double> X;
	solve(A, B, X, DECOMP_SVD);

	return X;
}

#define EPSILON 0.005
Mat_<double> IterativeLinearLSTriangulation(Point3d u,    //homogenous image point (u,v,1)
	Matx34d P,          //camera 1 matrix
	Point3d u1,         //homogenous image point in 2nd camera
	Matx34d P1          //camera 2 matrix
) {

	double wi = 1, wi1 = 1;
	Mat_<double> X(4, 1);

	for (int i = 0; i < 10; i++) { //Hartley suggests 10 iterations at most
		Mat_<double> X_ = LinearLSTriangulation(u, P, u1, P1);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
		//recalculate weights
		double p2x = Mat_<double>(Mat_<double>(P).row(2)*X)(0);
		double p2x1 = Mat_<double>(Mat_<double>(P1).row(2)*X)(0);

		//breaking point
		if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

		wi = p2x;
		wi1 = p2x1;

		//reweight equations and solve
		Matx43d A((u.x*P(2, 0) - P(0, 0)) / wi, (u.x*P(2, 1) - P(0, 1)) / wi, (u.x*P(2, 2) - P(0, 2)) / wi,
			(u.y*P(2, 0) - P(1, 0)) / wi, (u.y*P(2, 1) - P(1, 1)) / wi, (u.y*P(2, 2) - P(1, 2)) / wi,
			(u1.x*P1(2, 0) - P1(0, 0)) / wi1, (u1.x*P1(2, 1) - P1(0, 1)) / wi1, (u1.x*P1(2, 2) - P1(0, 2)) / wi1,
			(u1.y*P1(2, 0) - P1(1, 0)) / wi1, (u1.y*P1(2, 1) - P1(1, 1)) / wi1, (u1.y*P1(2, 2) - P1(1, 2)) / wi1
		);
		Mat_<double> B = (Mat_<double>(4, 1) << -(u.x*P(2, 3) - P(0, 3)) / wi,
			-(u.y*P(2, 3) - P(1, 3)) / wi,
			-(u1.x*P1(2, 3) - P1(0, 3)) / wi1,
			-(u1.y*P1(2, 3) - P1(1, 3)) / wi1
			);

		solve(A, B, X_, DECOMP_SVD);
		X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
	}

	return X;
}



bool CVClass::getPoint3d(vector<Point> pts2dL, vector<Point> pts2dR, vector<Point>& pts3d)
{
	double fxL = camParam.cameraMatrix[0].at<double>(0, 0);
	double fyL = camParam.cameraMatrix[0].at<double>(1, 1);
	double cxL = camParam.cameraMatrix[0].at<double>(0, 2);
	double cyL = camParam.cameraMatrix[0].at<double>(1, 2);

	double fxR = camParam.cameraMatrix[1].at<double>(0, 0);
	double fyR = camParam.cameraMatrix[1].at<double>(1, 1);
	double cxR = camParam.cameraMatrix[1].at<double>(0, 2);
	double cyR = camParam.cameraMatrix[1].at<double>(1, 2);
	if (pts2dL.size() != pts2dR.size())
	{
		cout << "pt2dL.size() != pt2dR.size()" << endl;
	}
	//for (int i = 0;i < pts2dL.size();i++)
	//{
	//	undistortPoints()
	//}
	pts3d.clear();
	cv::Mat pts3dMat(1, pts2dL.size(), CV_64FC4);
	cout << pts2dL << endl << pts2dR << endl;
	//cout << (Mat_<double>)pts3dMat/pts3dMat.at<long>(0, 3)  << endl;
	//Vec4i val= pts3dMat.at<long>(0, 0);
	//	cout << pts3dMat.at<long>(0, 3) << endl;

	//Mat ptsLM(1, 1, CV_64FC2);
	//Mat ptsRM(1, 1, CV_64FC2);	
	//Mat undistPtsLM(1, 1, CV_64FC2);
	//Mat undistPtsRM(1, 1, CV_64FC2);
	//ptsLM.at<Vec2d>(0, 0)[0] = pts2dL.at(0).x;
	//ptsLM.at<Vec2d>(0, 0)[1] = pts2dL.at(0).y;	
	//ptsRM.at<Vec2d>(0, 0)[0] = pts2dR.at(0).x;
	//ptsRM.at<Vec2d>(0, 0)[1] = pts2dR.at(0).y;
	//cout << ptsLM << endl << ptsRM << endl;

	//undistortPoints(ptsLM, undistPtsLM, camParam.cameraMatrix[0], camParam.distCoeffs[0]);
	//undistortPoints(ptsRM, undistPtsRM, camParam.cameraMatrix[1], camParam.distCoeffs[1]);

	//Point3d ptL = Point3d(undistPtsLM.at<Vec2d>(0, 0)[0]*fxL+cxL, undistPtsLM.at<Vec2d>(0, 0)[1]*fyL+cyL, 1);
	//Point3d ptR = Point3d(undistPtsRM.at<Vec2d>(0, 0)[0]*fxR+cxR, undistPtsRM.at<Vec2d>(0, 0)[1]*fyR+cyR, 1);
	//cout <<"undistorted"<< ptL << endl << ptR << endl;
	int npts = pts2dL.size();
	vector<Mat> pt3d(npts);
	vector<Point3f> tPts3dW;
	for (int i = 0;i < npts;i++)
	{
		Point3d ptL = Point3d(pts2dL.at(i).x, pts2dL.at(i).y, 1);
		Point3d ptR = Point3d(pts2dR.at(i).x, pts2dR.at(i).y, 1);
		pt3d.at(i) = IterativeLinearLSTriangulation(ptL, camParam.P1, ptR, camParam.P2);
		tPts3dW.push_back(Point3f(pt3d.at(i).at<double>(0, 0), pt3d.at(i).at<double>(1, 0), pt3d.at(i).at<double>(2, 0)));
	}
	if (npts >= 2)
	{
		cout << pt3d.at(0) << endl;
		cout << pt3d.at(1) << endl;
		cout << "dist=" << norm(pt3d.at(0) - pt3d.at(1)) << endl;;
	}

	vector<Point3f> Pts3dW(tPts3dW.size());

	perspectiveTransform(tPts3dW, Pts3dW, camParam.m);
	cout << (Mat)Pts3dW << endl;
	//Mat showDist(Size(1000, 500), CV_8UC3);
	//char dist[20] = "test";
	//sprintf(dist, "%8.4f", norm(pt3d));
	//putText(showDist, dist, Point(0, 250), FONT_HERSHEY_PLAIN, 10, Scalar(0, 0, 255));
	//imshow("dist", showDist);
	//waitKey(1);

	return false;
}
void CVClass::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
	corners.resize(0);

	switch (patternType)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float(j*squareSize),
					float(i*squareSize), 0));
		break;

	case ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float((2 * j + i % 2)*squareSize),
					float(i*squareSize), 0));
		break;

	default:
		CV_Error(Error::StsBadArg, "Unknown pattern type\n");
	}
}
bool CVClass::worldCSInit(void)
{
	float squareSize = 35.5;
	Pattern patternType = CHESSBOARD;
	vector<Point2f> pointbuf;
	Size boardSize = Size(5, 4);
	bool found;
	Mat frameGrey;
	cvtColor(frameL, frameGrey, COLOR_BGR2GRAY);
	switch (patternType)
	{
	case CHESSBOARD:
		found = findChessboardCorners(frameL, boardSize, pointbuf,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		break;
	case CIRCLES_GRID:
		found = findCirclesGrid(frameL, boardSize, pointbuf);
		break;
	case ASYMMETRIC_CIRCLES_GRID:
		found = findCirclesGrid(frameL, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
		break;
	default:
		return fprintf(stderr, "Unknown pattern type\n"), -1;
	}
	if (patternType == CHESSBOARD && found) cornerSubPix(frameGrey, pointbuf, Size(11, 11),
		Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

	if (found)
		drawChessboardCorners(frameL, boardSize, Mat(pointbuf), found);
	imshow("worldCS", frameL);
	char c = waitKey(1);
		if (c == 'w')
		{
			if (found)
			{
				vector<vector<Point3f> >  objectPoints(1);
				calcChessboardCorners(boardSize, squareSize, objectPoints.at(0), patternType);
				vector<vector<Point2f> >  imagePoints;
				imagePoints.push_back(pointbuf);


				calibrateCamera(objectPoints, imagePoints,frameL.size(), camParam.cameraMatrix[0], camParam.distCoeffs[0], camParam.rvec, camParam.tvec,CALIB_USE_INTRINSIC_GUESS);
				Matx33d r;
				Rodrigues(camParam.rvec, r);
				camParam.m << r(0, 0), r(0, 1), r(0, 2), camParam.tvec.at<double>(0),
					r(1, 0), r(1, 1), r(1, 2), camParam.tvec.at<double>(1),
					r(2, 0), r(2, 1), r(2, 2), camParam.tvec.at<double>(2),
					0, 0, 0, 1;
				cout << "m:" << camParam.m << endl;
				getchar();
				return true;
			}
			else
			{
				cout << "no circles grid found" << endl;
				return false;
			}
		}
	return false;
}