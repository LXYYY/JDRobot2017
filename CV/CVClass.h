#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "linux/videodev2.h"
#include "stdio.h"
#include "unistd.h"
#include "error.h"
#include "string.h"
#include "fcntl.h"
using namespace std;
using namespace cv;
class CVClass
{
private:
	VideoCapture camL;
	VideoCapture camR;
	enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
	void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType);
    int fd;
    char *camLDevice="/dev/video0";
    char *camRDevice="/dev/video1";

    char *qctl_name=NULL;
    int qctl_value=0;
    struct v4l2_capability cap;
    struct v4l2_queryctrl  qctrl;
    void print_qctrl(struct v4l2_queryctrl *qctrl);
    int set_qctrl(struct v4l2_queryctrl *qctrl);

public:
	CVClass();
	~CVClass();

	enum ColorE
	{
		YELLOW = 0,
		BLUE,
		GREEN
	};
	class objBoxImg
	{
	public:
		ColorE color;
		int id;
		objBoxImg();
		objBoxImg(vector<Point> tPts, ColorE tColor, int tId);
		~objBoxImg();
		vector<Point> pts;
		bool print(void);
		bool sortPts(void);
	};
	class objBox3D
	{
	public:
		objBox3D();
		~objBox3D();
		ColorE color;
		int id;
		vector<Point3f> pts3D;
	};
	struct CamParamS				//0 left ,1 right
	{
		Mat cameraMatrix[2];
		Mat distCoeffs[2];
		Mat R, T;
		Mat R1,R2;
		Mat P1,P2;
		Mat Q;
		Mat map1[2],map2[2];
		Size imgSize;
		Mat rvec, tvec;
		Matx44d m;
	} camParam;
	Mat frame;
	Mat frameL, frameR;
	Mat rFrameL,rFrameR;
	bool camParamInit(void);
	bool undistortFrame(void);
	bool getPoint3d(vector<Point> pt2dL, vector<Point> pt2dR, vector<Point>& pt3d);
	bool camCalib(void);
	bool stereoCalib(void);
	bool worldCSInit(void);
	bool camInit(bool LR);
	bool getImage();
	bool showImage();
	bool showUndistortedImage();
	int findGreen(Mat& img);
	int adjustContrast(Mat& src);
	bool getProjMats(Size imgSize);
	vector<objBox3D> computeBox3D(vector<objBoxImg> objBoxImgL, vector<objBoxImg> objBoxImgR);
	bool processSingle(Mat img,bool LorR, vector<objBoxImg>& objBoxRlt);
	vector<objBox3D> processStereo();
};

