#include "CVClass.h"
#include "../Param.h"

int rate = 1000;

//uchar *buffer;
//struct v4l2_buffer buf;
CVClass::CVClass() {
}


CVClass::~CVClass() {
}

bool CVClass::camInit(bool LR) {

    try {
        if (LR) {
            camR.openCam(1);
            camL.openCam(0);
        } else {
            camL.openCam(1);
            camR.openCam(0);
        }
    }
    catch (...) {
        cout << "camera open failed" << endl;
        return false;
    }
#ifdef WIN
    try
    {
        camL.set(CV_CAP_PROP_EXPOSURE, exposure);
        camL.set(CV_CAP_PROP_BRIGHTNESS, brightness);
        camR.set(CV_CAP_PROP_EXPOSURE, exposure);
        camR.set(CV_CAP_PROP_BRIGHTNESS, brightness);
        camL.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
        camL.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
        camR.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
        camR.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    }
    catch (...)
    {
        cout << "camera set failed" << endl;
        return true;
    }
    return false;
#endif

#ifdef LINUX

    try {
        cout<<"test1"<<endl;

        camL.exposure=120;
        camR.exposure=120;

//        camL.set_Video_Fps(100, 600);
//    get_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE);
        camL.set_Video_Parameter(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
        camL.set_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE, camL.exposure);
//        camL.set_Video_Parameter(V4L2_CID_BRIGHTNESS, brightness);

//        camR.set_Video_Fps(100, 600);
//    get_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE);
        camR.set_Video_Parameter(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
        camR.set_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE, camR.exposure);
        cout<<"exposure:"<<camL.exposure<<ends<<camR.exposure<<endl;
        cout<<"test"<<endl;
//        camL.set_Video_Parameter(V4L2_CID_BRIGHTNESS, brightness);

//        camL.set_Video_Parameter(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
//        camL.set_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE, exposure);
//        camL.set_Video_Parameter(V4L2_CID_BRIGHTNESS, brightness);
//        camR.set_Video_Parameter(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
//        camR.set_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE, exposure);
//        camR.set_Video_Parameter(V4L2_CID_BRIGHTNESS, brightness);
    }
    catch (...) {
        cout << "camera set failed" << endl;
        return false;
    }
    return true;
#endif
}

bool CVClass::getImage() {

    try {
#ifdef WIN
        camL.cam >> frameL;
        camR.cam >> frameR;
#endif

#ifdef LINUX
        frameL = camL.getImage();
        frameR = camR.getImage();
//        cout<<frameL.rows<<"*"<<frameL.cols<<endl;
#endif
    }
    catch (...) {
        return false;
    }
    return true;
}

bool CVClass::showImage() {

    try {
        resize(frameL, frameL, Size(), 0.5, 0.5);
        resize(frameR, frameR, Size(), 0.5, 0.5);
        imshow("image left", frameL);
        imshow("image right", frameR);
    }
    catch (...) {
        cout << "imshow failed" << endl;
        return false;
    }
    return true;
}

bool CVClass::showUndistortedImage() {
    try {
        imshow("undistorted image left", rFrameL);
        imshow("undistorted image right", rFrameR);
    }
    catch (...) {
        cout << "undistorted imshow failed" << endl;
        return false;
    }
    return true;
}

vector<CVClass::objBox3D> CVClass::computeBox3D(vector<objBoxImg> objBoxImgL, vector<objBoxImg> objBoxImgR) {

    return vector<objBox3D>();
}


bool CVClass::findGreen(Mat &src) {
    try {
        for (int i = 0; i < src.rows; i++) {
            uchar *data = src.ptr<uchar>(i);
            for (int j = 0; j < src.cols * 3; j += 3) {
                double brightness = data[j] + data[j + 1] + data[j + 2];
                brightness /= (255 * 3);
                //cout <<brightness<<ends<<rate<<ends<< (brightness*(double)rate / 1000.f) << endl;

                if (data[j + 2] < (data[j + 1] - 10) && data[j] < (data[j + 1] - 10)
                    && ((double) (data[j + 1] - data[j]) / (data[j] + 1)) > (brightness * rate / 1000.f) &&
                    ((double) (data[j + 1] - data[j + 2]) / (data[j] + 2)) >
                    (brightness * rate / 1000.f))       //Unreliable
                {
                    //data[j] = 255;
                    //data[j + 1] = 255;
                    //data[j + 2] = 255;
                } else {
                    //cout << ((data[j + 1] - data[j]) / (data[j] + 1)) << ends << (brightness*rate / 1000.f) << endl;

                    data[j] = 0;
                    data[j + 1] = 0;
                    data[j + 2] = 0;
                }
            }
        }
        return true;
    }
    catch (...) {
        cout << "color change error" << endl;
        return false;
    }
}

bool CVClass::adjustContrast(Mat &src) {
    try {
        for (int i = 0; i < src.rows; i++) {
            uchar *data = src.ptr<uchar>(i);
            for (int j = 0; j < src.cols; j++) {
                data[j] = 2 * ((double) (data[j]) / (255)) * 255;
            }
        }
        return true;
    }
    catch (...) {
        cout << "color change error" << endl;
        return false;
    }
}

//get projection mats of both cam
bool CVClass::getProjMats(Size imgSize) {

    return false;
}

int element_shape = MORPH_RECT;
int an = 5;
Mat element = getStructuringElement(element_shape, Size(an * 2 + 1, an * 2 + 1), Point(an, an));

bool CVClass::processSingle(Mat img, bool LorR, vector<objBoxImg> &objBoxRlt) {
    string LR;
    if (!LorR)
        LR = "L";
    else
        LR = "R";
    objBoxRlt.clear();
    Mat src;
    namedWindow("test" + LR);
    createTrackbar("rate", "test" + LR, &rate, 1000);
    vector<Vec4i> hierarchy;
    Mat dst, cdst;

    resize(img, src, Size(), 1, 1);
//    imshow("raw"+LR, src);
    findGreen(src);
//    imshow("green"+LR, src);

    morphologyEx(src, src, MORPH_CLOSE, element);
//    imshow("open"+LR, src);

    Canny(src, dst, 100, 200, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);

    cvtColor(src, src, COLOR_BGR2GRAY);

    //adjustContrast(src);
    vector<vector<Point> > contours0;
    findContours(src, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<Point> > approx(contours0.size());
    vector<vector<Point> > objBoxPts;

    for (int i = 0; i < contours0.size(); i++) {
        if (contourArea(contours0.at(i)) >= 10000) {
            vector<Point> hull;
            convexHull(contours0.at(i), hull, true);
            approxPolyDP(hull, approx.at(i), 1, true);
            //objBoxPts.push_back(approx.at(i));
            objBoxPts.push_back(hull);
        }
    }
    cvtColor(src, src, COLOR_GRAY2BGR);
//    drawContours(cdst, contours0, -1, Scalar(0, 0, 255));
    drawContours(cdst, approx, -1, Scalar(0, 255, 0));
    resize(cdst,cdst,Size(),0.5,0.5);
    imshow("test" + LR, cdst);

    if (objBoxPts.size()) {
        cout << "objBoxPts.size()=" << objBoxPts.size() << endl;
        for (int i = 0; i < objBoxPts.size(); i++) {
            objBoxRlt.push_back(CVClass::objBoxImg(objBoxPts.at(i), CVClass::ColorE::BLUE, i));
        }
    } else return false;

    return true;
}

vector<CVClass::objBox3D> CVClass::processStereo() {
    if (frameL.empty() || frameR.empty()) {
        cout << "img empty" << endl;
        return vector<CVClass::objBox3D>();
    } else {
        vector<objBox3D> objBox;
        vector<objBoxImg> objBoxImgL;
        vector<objBoxImg> objBoxImgR;
        try {
            processSingle(frameL, false, objBoxImgL);
            processSingle(frameR, true, objBoxImgR);
        }
        catch (...) {
            cout << "single imgproc wrong" << endl;
            return vector<CVClass::objBox3D>();
        }
        if (!objBoxImgL.size() || !objBoxImgR.size()) {
            cout << "no box found" << endl;
            return vector<CVClass::objBox3D>();
        } else {
            try {
                objBox = computeBox3D(objBoxImgL, objBoxImgR);
            }
            catch (...) {
                cout << "stereo compute wrong" << endl;
                return vector<CVClass::objBox3D>();
            }
        }
        return objBox;
    }
}

char CVClass::waitKeyProc(int delay) {
    char c = waitKey(delay);
    if (c == 'i') {
        camL.exposure++;
        camL.set_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE, camL.exposure);
        cout << "L exposure=" << camL.exposure << " R exposure=" << camR.exposure << endl;

    } else if (c == 'k') {
        camL.exposure--;
        camL.set_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE, camL.exposure);
        cout << "L exposure=" << camL.exposure << " R exposure=" << camR.exposure << endl;
    } else if (c == 'o') {
        camR.exposure++;
        camR.set_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE, camR.exposure);
        cout << "L exposure=" << camL.exposure << " R exposure=" << camR.exposure << endl;
    } else if (c == 'l') {
        camR.exposure--;
        cout << "L exposure=" << camL.exposure << " R exposure=" << camR.exposure << endl;
        camR.set_Video_Parameter(V4L2_CID_EXPOSURE_ABSOLUTE, camR.exposure);
    }
    return c;
}

bool CVClass::getPointWorld(vector<Mat> &pt3d, vector<Mat> &ptW) {
    ptW.clear();
    try {
        for (size_t i = 0; i < pt3d.size(); i++) {
            ptW.push_back(camParam.R2W.inv() * (pt3d.at(i) - camParam.T2W));
//            cout << "getPointworld" << endl
//                 << "camParam.R2W:" << camParam.R2W << endl
//                 << "camParam.R2W.inv:" << camParam.R2W.inv() << endl
//                 << "camParam.R2W.t:" << camParam.R2W.t() << endl
//                 << "R2W.norm:" << norm(camParam.R2W) << endl
//                 << "R2W.inv.norm:" << norm(camParam.R2W.inv()) << endl
//                 << "r2w*r2w.inv:" << camParam.R2W * camParam.R2W.inv() << endl
//                 << "pt3d.at(i):" << pt3d.at(i) << endl
//                 << "ori dist;" << norm((pt3d.at(i) - camParam.T2W)) << endl
//                 << "dist:" << norm(camParam.R2W.inv() * (pt3d.at(i) - camParam.T2W)) << endl
//                 << camParam.T2W << endl
//                 << "world cordinates:" << ptW.at(i) << endl;
        }
    }
    catch (...) {
        cout << "getPointWorld failed" << endl;
        return false;
    }
    return true;
}


bool CVClass::CamClass::v4l2_grab(void) {
    //struct v4l2_requestbuffers req = {0};
    //4  request for 4 buffers
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1)//resqursting Buffer
    {
        printf("Requesting Buffer error\n");
        return false;
    }
    //5 mmap for buffers
    buffer = (uchar *) malloc(req.count * sizeof(*buffer));
    if (!buffer) {
        printf("Out of memory\n");
        return false;
    }
    unsigned int n_buffers;
    for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
//      struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            printf("Querying Buffer error\n");
            return false;
        }
        buffer = (uchar *) mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (buffer == MAP_FAILED) {
            printf("buffer map error\n");
            return false;
        }
        printf("Length: %d\nAddress: %p\n", buf.length, buffer);
        printf("Image Length: %d\n", buf.bytesused);
    }
    //6 queue
    for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
        buf.index = n_buffers;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_QBUF, &buf)) {
            printf("query buffer error\n");
            return false;
        }
    }
    //7 starting
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        printf("stream on error\n");
        return false;
    }
    return true;
}

bool CVClass::CamClass::init_v4l2(int id) {

//    if ((fd = open(FILE_VIDEO1, O_RDWR|O_NONBLOCK)) == -1)//not block
    sprintf(camFile, "/dev/video%d", id);
    if ((fd = open(camFile, O_RDWR)) == -1) {
        printf("Opening video device error\n");
        return false;
    }
    fd_temp = fd;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        printf("unable Querying Capabilities\n");
        return false;
    }
//   else
/*   {
        printf( "Driver Caps:\n"
                "  Driver: \"%s\"\n"
                "  Card: \"%s\"\n"
                "  Bus: \"%s\"\n"
                "  Version: %d\n"
                "  Capabilities: %x\n",
                cap.driver,
                cap.card,
                cap.bus_info,
                cap.version,
                cap.capabilities);
    }

    if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE)

        printf("Camera device %s: support capture\n",FILE_VIDEO1);
    }
    if((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING)
    {
        printf("Camera device %s: support streaming.\n",FILE_VIDEO1);
    }
*/
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format: \n");
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1)//get the video parameter
    {
        printf("\t%d. %s\n", fmtdesc.index + 1, fmtdesc.description);
        fmtdesc.index++;
    }
    //set fmt
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = (__u32) imgSize.width;
    fmt.fmt.pix.height = (__u32) imgSize.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; //*************************V4L2_PIX_FMT_YUYV****************
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
//    fmt.fmt.pix.priv = 1;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1)//set the video parameter
    {
        printf("Setting Pixel Format error\n");
        return false;
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1) {
        printf("Unable to get format\n");
        return false;
    }
//   else
/*   {
        printf("fmt.type:\t%d\n",fmt.type);
        printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF,(fmt.fmt.pix.pixelformat >> 8) & 0xFF,\
               (fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        printf("pix.height:\t%d\n",fmt.fmt.pix.height);
        printf("pix.field:\t%d\n",fmt.fmt.pix.field);
    }
*/
/*
    setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    setfps.parm.capture.timeperframe.numerator = 100;
    setfps.parm.capture.timeperframe.denominator = 100;
    printf("init %s is OK\n",FILE_VIDEO1);
*/
    return true;
}

int CVClass::CamClass::set_Video_Fps(__u32 Numerator, __u32 Denominator) {
    struct v4l2_streamparm Stream_Parm;
    int io_rel(0);
    get_Video_Parameter();
    memset(&Stream_Parm, 0, sizeof(struct v4l2_streamparm));
    Stream_Parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    Stream_Parm.parm.capture.timeperframe.denominator = Denominator;
    Stream_Parm.parm.capture.timeperframe.numerator = Numerator;
    io_rel = ioctl(fd, VIDIOC_S_PARM, &Stream_Parm);
    get_Video_Parameter();
    if (io_rel != 0) {
        perror("Unable to set the value of fps");
        return -1;
    }
    return 0;
}

int CVClass::CamClass::get_Video_Parameter(void) {
    int io_rel;
    struct v4l2_streamparm Stream_Parm;
    memset(&Stream_Parm, 0, sizeof(struct v4l2_streamparm));
    Stream_Parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    io_rel = ioctl(fd, VIDIOC_G_PARM, &Stream_Parm);
    if (io_rel != 0) {
        perror("Unable to set the value of fps");
        return -1;
    }
    return 0;
}

int CVClass::CamClass::get_Video_Parameter(__u32 parameter) {
    int io_rel(0);
    struct v4l2_control cemara_ctrl;
    memset(&cemara_ctrl, 0, sizeof(struct v4l2_control));
    cemara_ctrl.id = parameter;
    io_rel = ioctl(fd, VIDIOC_G_CTRL, &cemara_ctrl);
    if (io_rel != 0) {
        perror("Unable to get the value of parameter");
        return -1;
    }
    return 0;
}

int CVClass::CamClass::set_Video_Parameter(__u32 parameter, __s32 value_of_parameter) {
    int io_rel(0);
    struct v4l2_control cemara_ctrl;
    memset(&cemara_ctrl, 0, sizeof(struct v4l2_control));
    cemara_ctrl.id = parameter;
    cemara_ctrl.value = value_of_parameter;
    io_rel = ioctl(fd, VIDIOC_S_CTRL, &cemara_ctrl);
    if (io_rel != 0) {
        perror("Unable to set the value of parameter");
        return -1;
    }
    return 0;
}

bool CVClass::CamClass::openCam(int id) {
#ifdef WIN
    cam.open(id);
#endif

#ifdef LINUX
    if (!init_v4l2(id)) {
        printf("Init fail~~\n");
        return false;
    }
    //    printf("second~~\n");
    if (!v4l2_grab()) {
        printf("grab fail~~\n");
        return false;
    }
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
#endif
    return true;
}

Mat CVClass::CamClass::getImage(void) {
    Mat timg;
    ioctl(fd, VIDIOC_DQBUF, &buf);
    buf.index = 0;
    timg = imdecode(cv::Mat(imgSize, CV_8UC3, (void *) buffer), CV_LOAD_IMAGE_COLOR);
//    cout<<timg.empty()<<endl;
    ioctl(fd, VIDIOC_QBUF, &buf);
//    imshow("test",timg);
//    waitKey(1);
    return timg;
}


CVClass::objBoxImg::objBoxImg() {
}

CVClass::objBoxImg::objBoxImg(vector<Point> tPts, ColorE tColor, int tId) {
    pts = tPts;
    color = tColor;
    id = tId;
}

CVClass::objBoxImg::~objBoxImg() {
}

bool CVClass::objBoxImg::print(void) {
    cout << (Mat) pts << endl;
    return false;
}

bool CVClass::objBoxImg::sortPts(void) {
    Point tPt;
    for (int i = 0; i < pts.size(); i++) {
        for (int j = i + 1; j < pts.size(); j++) {
            if (pts.at(j).y < pts.at(i).y) {
                tPt = pts.at(j);
                pts.at(j) = pts.at(i);
                pts.at(i) = tPt;
            }
        }
    }
    return false;
}

CVClass::objBox3D::objBox3D() {
}

CVClass::objBox3D::~objBox3D() {
}
