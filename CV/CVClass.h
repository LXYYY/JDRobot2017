#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
using namespace std;
using namespace cv;
class CVClass
{
public:
    class CamClass
    {
    public:
        CamClass(){}
        ~CamClass(){}
        VideoCapture cam;
        Size imgSize=Size(640,480);
//        uchar *buffer;
        int fd;
        int fd_temp;
        char camFile[20]="/dev/video0";
        struct v4l2_streamparm setfps;
        struct v4l2_capability cap;
        struct v4l2_fmtdesc fmtdesc;
        struct v4l2_format fmt,fmtack;
        struct v4l2_requestbuffers req;
//        struct v4l2_buffer buf;
        enum   v4l2_buf_type type;
        bool openCam(int id);
        bool init_v4l2(int id);//deful 1280*720
        bool v4l2_grab(void);
        int set_Video_Fps(__u32 Numerator,__u32 Denominator = 1);
        int get_Video_Parameter(void);
        int set_Video_Parameter(__u32 parameter,__s32 value_of_parameter);
        int get_Video_Parameter(__u32 parameter);
        Mat getImage(void);
    };
    CamClass camL;
//    CamClass camR;
    enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
	void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType);

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
	bool findGreen(Mat& img);
	bool adjustContrast(Mat& src);
	bool getProjMats(Size imgSize);
	vector<objBox3D> computeBox3D(vector<objBoxImg> objBoxImgL, vector<objBoxImg> objBoxImgR);
	bool processSingle(Mat img,bool LorR, vector<objBoxImg>& objBoxRlt);
	vector<objBox3D> processStereo();
};

