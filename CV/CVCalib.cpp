#include "CVClass.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

bool CVClass::camParamInit(void) {
    //intrinsics
    FileStorage paramL("/home/jdrobot/Desktop/JDRobot/paramL.yaml", FileStorage::READ);
    FileStorage paramR("/home/jdrobot/Desktop/JDRobot/paramR.yaml", FileStorage::READ);

    if (!paramL.isOpened()) {
        cout << "paramL.xml not available" << endl;
        getchar();
        return false;
    }
    if (!paramR.isOpened()) {
        cout << "paramR.xml not available" << endl;
        return false;
    }

    cout << (int) paramL["nframes"] << endl;

    paramL["camera_matrix"] >> camParam.cameraMatrix[0];
    paramL["distortion_coefficients"] >> camParam.distCoeffs[0];

    paramR["camera_matrix"] >> camParam.cameraMatrix[1];
    paramR["distortion_coefficients"] >> camParam.distCoeffs[1];

    paramL.release();
    paramR.release();

    getImage();
    if (frameL.size() != frameR.size()) {
        cout << "imgSizes not equal" << endl;
        return false;
    } else
        camParam.imgSize = frameL.size();

    try {
        initUndistortRectifyMap(camParam.cameraMatrix[0], camParam.distCoeffs[0], Mat(),
                                getOptimalNewCameraMatrix(camParam.cameraMatrix[0], camParam.distCoeffs[0],
                                                          camParam.imgSize, 1, camParam.imgSize, 0),
                                camParam.imgSize, CV_16SC2, camParam.map1[0], camParam.map2[0]);
        initUndistortRectifyMap(camParam.cameraMatrix[1], camParam.distCoeffs[1], Mat(),
                                getOptimalNewCameraMatrix(camParam.cameraMatrix[1], camParam.distCoeffs[1],
                                                          camParam.imgSize, 1, camParam.imgSize, 0),
                                camParam.imgSize, CV_16SC2, camParam.map1[1], camParam.map2[1]);
    }
    catch (...) {
        cout << "initUndistortedRectifyMap failed " << endl;
        return false;
    }

    //extrinsics
    FileStorage extrinsics("/home/jdrobot/Desktop/JDRobot/extrinsics.yml", FileStorage::READ);
    if (!extrinsics.isOpened()) {
        cout << "extrinsics.xml not available" << endl;
        return false;
    }

    extrinsics["R"] >> camParam.R;
    extrinsics["T"] >> camParam.T;
    extrinsics["R1"] >> camParam.R1;
    extrinsics["R2"] >> camParam.R2;
    extrinsics["P1"] >> camParam.P1;
    extrinsics["P2"] >> camParam.P2;
    extrinsics["Q"] >> camParam.Q;

    extrinsics.release();

    return true;
}

bool CVClass::undistortFrame(void) {
    try {
        remap(frameL, rFrameL, camParam.map1[0], camParam.map2[0], INTER_LINEAR);
        remap(frameR, rFrameR, camParam.map1[1], camParam.map2[1], INTER_LINEAR);
    }
    catch (...) {
        cout << "remap failed" << endl;
        return true;
    }
    return false;
}

Mat_<double> LinearLSTriangulation(Point3d u,       //homogenous image point (u,v,1)
                                   Matx34d P,       //camera 1 matrix
                                   Point3d u1,      //homogenous image point in 2nd camera
                                   Matx34d P1       //camera 2 matrix
) {
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    Matx43d A(u.x * P(2, 0) - P(0, 0), u.x * P(2, 1) - P(0, 1), u.x * P(2, 2) - P(0, 2),
              u.y * P(2, 0) - P(1, 0), u.y * P(2, 1) - P(1, 1), u.y * P(2, 2) - P(1, 2),
              u1.x * P1(2, 0) - P1(0, 0), u1.x * P1(2, 1) - P1(0, 1), u1.x * P1(2, 2) - P1(0, 2),
              u1.y * P1(2, 0) - P1(1, 0), u1.y * P1(2, 1) - P1(1, 1), u1.y * P1(2, 2) - P1(1, 2)
    );
    Mat_<double> B = (Mat_<double>(4, 1) << -(u.x * P(2, 3) - P(0, 3)),
            -(u.y * P(2, 3) - P(1, 3)),
            -(u1.x * P1(2, 3) - P1(0, 3)),
            -(u1.y * P1(2, 3) - P1(1, 3)));

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
        X(0) = X_(0);
        X(1) = X_(1);
        X(2) = X_(2);
        X(3) = 1.0;
        //recalculate weights
        double p2x = Mat_<double>(Mat_<double>(P).row(2) * X)(0);
        double p2x1 = Mat_<double>(Mat_<double>(P1).row(2) * X)(0);

        //breaking point
        if (fabsf(wi - p2x) <= EPSILON && fabsf(wi1 - p2x1) <= EPSILON) break;

        wi = p2x;
        wi1 = p2x1;

        //reweight equations and solve
        Matx43d A((u.x * P(2, 0) - P(0, 0)) / wi, (u.x * P(2, 1) - P(0, 1)) / wi, (u.x * P(2, 2) - P(0, 2)) / wi,
                  (u.y * P(2, 0) - P(1, 0)) / wi, (u.y * P(2, 1) - P(1, 1)) / wi, (u.y * P(2, 2) - P(1, 2)) / wi,
                  (u1.x * P1(2, 0) - P1(0, 0)) / wi1, (u1.x * P1(2, 1) - P1(0, 1)) / wi1,
                  (u1.x * P1(2, 2) - P1(0, 2)) / wi1,
                  (u1.y * P1(2, 0) - P1(1, 0)) / wi1, (u1.y * P1(2, 1) - P1(1, 1)) / wi1,
                  (u1.y * P1(2, 2) - P1(1, 2)) / wi1
        );
        Mat_<double> B = (Mat_<double>(4, 1) << -(u.x * P(2, 3) - P(0, 3)) / wi,
                -(u.y * P(2, 3) - P(1, 3)) / wi,
                -(u1.x * P1(2, 3) - P1(0, 3)) / wi1,
                -(u1.y * P1(2, 3) - P1(1, 3)) / wi1
        );

        solve(A, B, X_, DECOMP_SVD);
        X(0) = X_(0);
        X(1) = X_(1);
        X(2) = X_(2);
        X(3) = 1.0;
    }

    return X;
}


bool CVClass::getPoint3d(vector<Point> pts2dL, vector<Point> pts2dR, vector<Point3f> &pts3d) {
    double fxL = camParam.cameraMatrix[0].at<double>(0, 0);
    double fyL = camParam.cameraMatrix[0].at<double>(1, 1);
    double cxL = camParam.cameraMatrix[0].at<double>(0, 2);
    double cyL = camParam.cameraMatrix[0].at<double>(1, 2);

    double fxR = camParam.cameraMatrix[1].at<double>(0, 0);
    double fyR = camParam.cameraMatrix[1].at<double>(1, 1);
    double cxR = camParam.cameraMatrix[1].at<double>(0, 2);
    double cyR = camParam.cameraMatrix[1].at<double>(1, 2);
    pts3d.clear();
    if (pts2dL.size() != pts2dR.size()) {
        cout << "pt2dL.size() != pt2dR.size()" << endl;
        return false;
    }
    try {
        pts3d.clear();

        size_t npts = pts2dL.size();
        vector<Mat> pt3dMat(npts);
        for (size_t i = 0; i < npts; i++) {
            Point3d ptL = Point3d(pts2dL.at(i).x, pts2dL.at(i).y, 1);
            Point3d ptR = Point3d(pts2dR.at(i).x, pts2dR.at(i).y, 1);
            pt3dMat.at(i) = IterativeLinearLSTriangulation(ptL, camParam.P1,
                                                           ptR, camParam.P2);
        }
        for (size_t i = 0; i < pt3dMat.size(); i++) {
            pts3d.push_back(Point3f(pt3dMat.at(i).at<double>(0),
                                    pt3dMat.at(i).at<double>(1),
                                    pt3dMat.at(i).at<double>(2)));
//            cout<<pt3dMat.at(i)<<endl;
        }
    }
    catch (...) {
        cout << "compute 3d points failed" << endl;
        return false;
    }
    return true;
}

void CVClass::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f> &corners,
                                    Pattern patternType = CHESSBOARD) {
    corners.resize(0);

    switch (patternType) {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(Point3f(float(j * squareSize),
                                              float(i * squareSize), 0));
            break;

        case ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(Point3f(float((2 * j + i % 2) * squareSize),
                                              float(i * squareSize), 0));
            break;

        default:
            CV_Error(Error::StsBadArg, "Unknown pattern type\n");
    }
}

bool CVClass::worldCSInit(void) {
    bool foundTagsL;
    bool foundTagsR;
    foundTagsL = aprilTags.processImage(frameL, aprilTags.tagsL);
    foundTagsR = aprilTags.processImage(frameR, aprilTags.tagsR);

    namedWindow("tags left");
    namedWindow("tags right");

    Mat tImgL, tImgR;
    frameL.copyTo(tImgL);
    frameR.copyTo(tImgR);

    aprilTags.drawTags(tImgL, aprilTags.tagsL);
    aprilTags.drawTags(tImgR, aprilTags.tagsR);

    resize(tImgL,tImgL,Size(),0.5,0.5);
    resize(tImgR,tImgR,Size(),0.5,0.5);
    imshow("tags left", tImgL);
    imshow("tags right", tImgR);

    char c = waitKeyProc(1);

    if (c == 'w') {
        cout<<c<<endl;
        cout<<foundTagsL<<" "<<foundTagsR<<endl;
        if (foundTagsL && foundTagsR) {
            vector<Point3f> pts3d;
            vector<Point> pts2dL, pts2dR;

            for (int i = 0; i < 4; i++) {
                pts2dL.push_back(
                        Point(aprilTags.tagsL.at(0).p[i].first,
                              aprilTags.tagsL.at(0).p[i].second));
                pts2dR.push_back(
                        Point(aprilTags.tagsR.at(0).p[i].first,
                              aprilTags.tagsR.at(0).p[i].second));
            }
            pts2dL.push_back(
                    Point(aprilTags.tagsL.at(0).cxy.first,
                          aprilTags.tagsL.at(0).cxy.second));
            pts2dR.push_back(
                    Point(aprilTags.tagsR.at(0).cxy.first,
                          aprilTags.tagsR.at(0).cxy.second));

            getPoint3d(pts2dL, pts2dR, pts3d);
            cout<<(Mat)pts3d<<endl;
        }
    }
    return false;
}

double CVClass::computeReprojectionErrors(
        const vector<vector<Point3f> > &objectPoints,
        const vector<vector<Point2f> > &imagePoints,
        const vector<Mat> &rvecs, const vector<Mat> &tvecs,
        const Mat &cameraMatrix, const Mat &distCoeffs,
        vector<float> &perViewErrors) {
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int) objectPoints.size(); i++) {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
        int n = (int) objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

bool CVClass::runCalibration(vector<vector<Point2f> > imagePoints,
                             Size imageSize, Size boardSize, Pattern patternType,
                             float squareSize, float aspectRatio,
                             int flags, Mat &cameraMatrix, Mat &distCoeffs,
                             vector<Mat> &rvecs, vector<Mat> &tvecs,
                             vector<float> &reprojErrs,
                             double &totalAvgErr) {
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (flags & CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, flags/* | CALIB_FIX_K4 | CALIB_FIX_K5*/);
    ///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

void CVClass::saveCameraParams(const string &filename,
                               Size imageSize, Size boardSize,
                               float squareSize, float aspectRatio, int flags,
                               const Mat &cameraMatrix, const Mat &distCoeffs,
                               const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                               const vector<float> &reprojErrs,
                               const vector<vector<Point2f> > &imagePoints,
                               double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nframes" << (int) std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if (flags & CALIB_FIX_ASPECT_RATIO)
        fs << "aspectRatio" << aspectRatio;

    if (flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
        //cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (!reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if (!rvecs.empty() && !tvecs.empty()) {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int) rvecs.size(), 6, rvecs[0].type());
        for (int i = 0; i < (int) rvecs.size(); i++) {
            Mat r = bigmat(Range(i, i + 1), Range(0, 3));
            Mat t = bigmat(Range(i, i + 1), Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if (!imagePoints.empty()) {
        Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int) imagePoints.size(); i++) {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

bool CVClass::runAndSave(const string &outputFilename,
                         const vector<vector<Point2f> > &imagePoints,
                         Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                         float aspectRatio, int flags, Mat &cameraMatrix,
                         Mat &distCoeffs, bool writeExtrinsics, bool writePoints) {
    cout << "running" << endl;
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                             aspectRatio, flags, cameraMatrix, distCoeffs,
                             rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if (ok)
        saveCameraParams(outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr);
    return ok;
}

bool CVClass::camCalib(void) {
#if 0
    char imgL[10] = "";
    char imgR[10] = "";
    int cnt = 0;
    Mat tFrameL, tFrameR;

    while (1) {
        getImage();
        char c = waitKeyProc(1);
        if (c == 's') {
            sprintf(imgL, "imgL%d.jpg", cnt);
            sprintf(imgR, "imgR%d.jpg", cnt);
            imwrite(imgL, frameL);
            imwrite(imgR, frameR);
            cout << cnt << endl;
            cnt++;
        } else if (c == 'q')
            break;
        vector<Point2f> pointbuf;
        bool found;
        found = findCirclesGrid(frameL, boardSize, pointbuf);
        if (found)
            drawChessboardCorners(frameL, boardSize, Mat(pointbuf), found);
        found = false;
        pointbuf.clear();
        found = findCirclesGrid(frameR, boardSize, pointbuf);
        if (found)
            drawChessboardCorners(frameR, boardSize, Mat(pointbuf), found);
        resize(frameL, tFrameL, Size(640, 480));
        resize(frameR, tFrameR, Size(640, 480));
        imshow("camL", tFrameL);
        imshow("camR", tFrameR);
    }
#endif
    Size imageSize;
    Mat cameraMatrix, distCoeffs;
    string outputFilenameL = "paramL.xml";
    string outputFilenameR = "paramR.xml";
    string inputFilename = "";

    int i, nframes;
    bool undistortImage = false;
    int flags = 0;
    VideoCapture capture;
    clock_t prevTimestamp = 0;
    int mode = DETECTION;
    int cameraId = 0;
    vector<vector<Point2f> > imagePoints;
    vector<string> imageList;

    mode = CAPTURING;

    namedWindow("Image View", 1);

    char imgName[20] = "";
    char outputFilename[20] = "";
    vector<Mat> imgs;
    for (int lr = 1; lr > -1; lr--) {
        imgs.clear();
        imagePoints.clear();
        mode = CAPTURING;
        nframes = 0;
        for (int i = 0;; i++) {
            if (lr == 0)
                sprintf(imgName, "imgL%d.jpg", i);
            else if (lr == 1)
                sprintf(imgName, "imgR%d.jpg", i);

            Mat tImg;
            tImg = imread(imgName, 1);
            if (!tImg.empty()) {
                imgs.push_back(tImg);
                cout << imgName << endl;
            } else {
                cout << "empty" << endl;
                break;
            }
        }
        nframes = imgs.size();

        if (lr == 0)
            sprintf(outputFilename, "/home/jdrobot/Desktop/JDRobot/paramL.yaml");
        else if (lr == 1)
            sprintf(outputFilename, "/home/jdrobot/Desktop/JDRobot/paramR.yaml");
        //calibration
        for (i = 0;; i++) {
            cout << "imagePoints.size()=" << imagePoints.size()
                 << " mode=" << mode << " nframes=" << nframes << " lr=" << lr << " " << "i=" << i << endl;
            Mat view, viewGray;
            bool blink = false;

            if (i == nframes) {
                if (imagePoints.size() > 0) {
                    runAndSave(outputFilename, imagePoints, imageSize,
                               boardSize, pattern, squareSize, aspectRatio,
                               flags, cameraMatrix, distCoeffs,
                               writeExtrinsics, writePoints);
                }
                break;
            }


            view = imgs.at(i);
            imageSize = view.size();

            if (flipVertical)
                flip(view, view, 0);

            vector<Point2f> pointbuf;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);

            /////////////////////pattern/////////////////////
            bool found;
            switch (pattern) {
                case CHESSBOARD:
                    found = findChessboardCorners(view, boardSize, pointbuf,
                                                  CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK |
                                                  CALIB_CB_NORMALIZE_IMAGE);
                    break;
                case CIRCLES_GRID:
                    found = findCirclesGrid(view, boardSize, pointbuf);
                    break;
                case ASYMMETRIC_CIRCLES_GRID:
                    found = findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
                    break;
                default:
                    return fprintf(stderr, "Unknown pattern type\n"), -1;
            }
            ////////////////////////////////////////////////////

            // improve the found corners' coordinate accuracy
            if (pattern == CHESSBOARD && found)
                cornerSubPix(viewGray, pointbuf, Size(11, 11),
                             Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

            if (mode == CAPTURING && found &&
                (!capture.isOpened() || clock() - prevTimestamp > delay * 1e-3 * CLOCKS_PER_SEC)) {
                imagePoints.push_back(pointbuf);
                prevTimestamp = clock();
                blink = capture.isOpened();
            }
            cout << pointbuf.size() << endl;
            if (found)
                drawChessboardCorners(view, boardSize, Mat(pointbuf), found);


            string msg = mode == CAPTURING ? "100/100" :
                         mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
            int baseLine = 0;
            Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
            Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

            if (mode == CAPTURING) {
                if (undistortImage)
                    msg = format("%d/%d Undist", (int) imagePoints.size(), nframes);
                else
                    msg = format("%d/%d", (int) imagePoints.size(), nframes);
            }

            putText(view, msg, textOrigin, 1, 1,
                    mode != CALIBRATED ? Scalar(0, 0, 255) : Scalar(0, 255, 0));

            if (blink)
                bitwise_not(view, view);

            if (mode == CALIBRATED && undistortImage) {
                Mat temp = view.clone();
                undistort(temp, view, cameraMatrix, distCoeffs);
            }

            imshow("Image View", view);
            char key = (char) waitKey(capture.isOpened() ? 50 : 500);

            if (key == 27)
                break;

            if (key == 'u' && mode == CALIBRATED)
                undistortImage = !undistortImage;

            if (capture.isOpened() && key == 'g') {
                mode = CAPTURING;
                imagePoints.clear();
            }

            if (mode == CAPTURING && imagePoints.size() >= (unsigned) nframes) {
                if (runAndSave(outputFilename, imagePoints, imageSize,
                               boardSize, pattern, squareSize, aspectRatio,
                               flags, cameraMatrix, distCoeffs,
                               writeExtrinsics, writePoints))
                    mode = CALIBRATED;
                else
                    mode = DETECTION;
                if (!capture.isOpened())
                    break;
            }
        }  //calibration

        //show undistorted
        if (!capture.isOpened() && showUndistorted) {
            cout << "undistorted  ";
            Mat view, rview, map1, map2;
            initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                                    getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                    imageSize, CV_16SC2, map1, map2);

            view = imgs.at(0);
            if (view.empty())
                continue;
            //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow(imgName, rview);
            char c = (char) waitKey(1000);
            if (c == 27 || c == 'q' || c == 'Q')
                break;
        }

        cout << lr << endl;

    }
    waitKey(0);
    return false;
}

bool CVClass::stereoCalib(void) {

    bool displayCorners = true;
    bool useCalibrated = true;
    bool showRectified = true;
    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    vector<Mat> imgs[2];

    char imgName[20] = "";
    for (int i = 0;; i++) {
        sprintf(imgName, "imgL%d.jpg", i);
        Mat tImg;
        tImg = imread(imgName, 0);
        if (!tImg.empty())
            imgs[0].push_back(tImg);
        else
            break;
    }
    for (int i = 0;; i++) {
        sprintf(imgName, "imgR%d.jpg", i);
        Mat tImg;
        tImg = imread(imgName, 0);
        if (!tImg.empty())
            imgs[1].push_back(tImg);
        else
            break;
    }
    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int i, j, k, nimages = (int) imgs[0].size();

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<Mat> goodImageList;

    for (i = j = 0; i < nimages; i++) {
        for (k = 0; k < 2; k++) {
            Mat img = imgs[k][i];
            if (img.empty()) {
                cout << "test" << endl;
                getchar();
                break;
            }
            cout << "test1" << endl;

            if (imageSize == Size())
                imageSize = img.size();
            else if (img.size() != imageSize) {
                cout << "The image" << k << " " << i
                     << " has the size different from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f> &corners = imagePoints[k][j];
            for (int scale = 1; scale <= maxScale; scale++) {
                Mat timg;
                if (scale == 1)
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findCirclesGrid(timg, boardSize, corners);
                cout << "found? " << found << endl;
                if (found) {
                    if (scale > 1) {
                        Mat cornersMat(corners);
                        cornersMat *= 1. / scale;
                    }
                    break;
                }
            }
            if (displayCorners) {
                cout << k << " " << i << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640. / MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("corners", cimg1);
                char c = (char) waitKey(500);
                if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
                    exit(-1);
            } else
                putchar('.');
            if (!found)
                break;
            cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
                         TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
                                      30, 0.01));
        }
        if (k == 2) {
            goodImageList.push_back(imgs[0][i]);
            goodImageList.push_back(imgs[1][i]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if (nimages < 2) {
        cout << "Error: too little pairs to run the calibration\n";
        return false;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for (i = 0; i < nimages; i++) {
        for (j = 0; j < boardSize.height; j++)
            for (k = 0; k < boardSize.width; k++)
                objectPoints[i].push_back(Point3f(k * squareSize, j * squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    FileStorage paramL("/home/jdrobot/Desktop/JDRobot/paramL.yaml", FileStorage::READ);
    FileStorage paramR("/home/jdrobot/Desktop/JDRobot/paramR.yaml", FileStorage::READ);

    if (!paramL.isOpened()) {
        cout << "paramL.xml not available" << endl;
        getchar();
    }
    if (!paramR.isOpened()) {
        cout << "paramR.xml not available" << endl;
        getchar();
    }

    cout << (int) paramL["nframes"] << endl;

    paramL["camera_matrix"] >> cameraMatrix[0];
    paramL["distortion_coefficients"] >> distCoeffs[0];

    paramR["camera_matrix"] >> cameraMatrix[1];
    paramR["distortion_coefficients"] >> distCoeffs[1];

    paramL.release();
    paramR.release();

    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                                 cameraMatrix[0], distCoeffs[0],
                                 cameraMatrix[1], distCoeffs[1],
                                 imageSize, R, T, E, F,
                                 CALIB_FIX_ASPECT_RATIO +
                                 //CALIB_ZERO_TANGENT_DIST +
                                 //CALIB_FIX_INTRINSIC +
                                 CALIB_USE_INTRINSIC_GUESS +
                                 CALIB_SAME_FOCAL_LENGTH +
                                 CALIB_RATIONAL_MODEL +
                                 CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                                 TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
    cout << "done with RMS error=" << rms << endl;

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for (i = 0; i < nimages; i++) {
        int npt = (int) imagePoints[0][i].size();
        Mat imgpt[2];
        for (k = 0; k < 2; k++) {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
        }
        for (j = 0; j < npt; j++) {
            double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] +
                                imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x * lines[0][j][0] +
                                imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " << err / npoints << endl;

    // save intrinsic parameters
    FileStorage fs("/home/jdrobot/Desktop/JDRobot/intrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
           "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    } else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, -1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("/home/jdrobot/Desktop/JDRobot/extrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    } else
        cout << "Error: can not save the extrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

    // COMPUTE AND DISPLAY RECTIFICATION
    if (!showRectified)
        return false;

    Mat rmap[2][2];
    // IF BY CALIBRATED (BOUGUET'S METHOD)
    if (useCalibrated) {
        // we already computed everything
    }
        // OR ELSE HARTLEY'S METHOD
    else
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for (k = 0; k < 2; k++) {
            for (i = 0; i < nimages; i++)
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv() * H1 * cameraMatrix[0];
        R2 = cameraMatrix[1].inv() * H2 * cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if (!isVerticalStereo) {
        sf = 600. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h, w * 2, CV_8UC3);
    } else {
        sf = 300. / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h * 2, w, CV_8UC3);
    }

    for (i = 0; i < nimages; i++) {
        for (k = 0; k < 2; k++) {
            Mat img = goodImageList[i * 2 + k], rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if (useCalibrated) {
                Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
                          cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
                rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
            }
        }

        if (!isVerticalStereo)
            for (j = 0; j < canvas.rows; j += 16)
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for (j = 0; j < canvas.cols; j += 16)
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char) waitKey();
        if (c == 27 || c == 'q' || c == 'Q')
            break;
    }
}