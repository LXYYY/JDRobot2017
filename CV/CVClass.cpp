#include "CVClass.h"
#include "../Param.h"

int rate = 1000;

CVClass::CVClass() {
}


CVClass::~CVClass() {
}

bool CVClass::camInit(bool LR) {
#ifdef WIN
    try {
        if (LR) {
            camR.open(1);
            camL.open(0);
        }
        else {
            camL.open(1);
            camR.open(0);
        }
    }
    catch (...)
    {
        cout << "camera open failed" << endl;
        return true;
    }
    int exposure = -6;
    int brightness = 0;
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
        if (LR) {
            camR.open(1);
            camL.open(0);
        } else {
            camL.open(1);
            camR.open(0);
        }
    }
    catch (...) {
        cout << "camera open failed" << endl;
        return true;
    }
    int exposure = -6;
    int brightness = 0;
    try {
        camL.set(CV_CAP_PROP_EXPOSURE, exposure);
        camL.set(CV_CAP_PROP_BRIGHTNESS, brightness);
        camR.set(CV_CAP_PROP_EXPOSURE, exposure);
        camR.set(CV_CAP_PROP_BRIGHTNESS, brightness);
        camL.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
        camL.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
        camR.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
        camR.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

    }
    catch (...) {
        cout << "camera set failed" << endl;
        return true;
    }
    return false;
#endif
}

bool CVClass::getImage() {
    try {
        camL >> frameL;
        camR >> frameR;
    }
    catch (...) {
        return true;
    }
    return false;
}

bool CVClass::showImage() {
    try {
        imshow("image left", frameL);
        imshow("image right", frameR);
    }
    catch (...) {
        cout << "imshow failed" << endl;
        return true;
    }
    return false;
}

bool CVClass::showUndistortedImage() {
    try {
        imshow("undistorted image left", rFrameL);
        imshow("undistorted image right", rFrameR);
    }
    catch (...) {
        cout << "undistorted imshow failed" << endl;
        return true;
    }
    return false;
}

vector<CVClass::objBox3D> CVClass::computeBox3D(vector<objBoxImg> objBoxImgL, vector<objBoxImg> objBoxImgR) {

    return vector<objBox3D>();
}


int CVClass::findGreen(Mat &src) {
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
        return 0;
    }
    catch (...) {
        cout << "color change error" << endl;
        return 1;
    }
}

int CVClass::adjustContrast(Mat &src) {
    try {
        for (int i = 0; i < src.rows; i++) {
            uchar *data = src.ptr<uchar>(i);
            for (int j = 0; j < src.cols; j++) {
                data[j] = 2 * ((double) (data[j]) / (255)) * 255;
            }
        }
        return 0;
    }
    catch (...) {
        cout << "color change error" << endl;
        return 1;
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
    //imshow("raw"+LR, src);
    findGreen(src);
    //imshow("green"+LR, src);

    morphologyEx(src, src, MORPH_CLOSE, element);
    //imshow("open"+LR, src);

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
    //drawContours(cdst, contours0, -1, Scalar(0, 0, 255));
    drawContours(cdst, approx, -1, Scalar(0, 255, 0));
    imshow("test" + LR, cdst);

    if (objBoxPts.size()) {
        cout << "objBoxPts.size()=" << objBoxPts.size() << endl;
        for (int i = 0; i < objBoxPts.size(); i++) {
            objBoxRlt.push_back(CVClass::objBoxImg(objBoxPts.at(i), CVClass::ColorE::BLUE, i));
        }
    } else return true;

    return false;
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
