//
// Created by homer on 04/10/2018.
//
#include <android/log.h>
#include <android/bitmap.h>

#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"
using cv::Mat;
using cv::Size;
using cv::Point;
using cv::Rect;
using cv::ellipse;
using cv::Scalar;
using cv::getTickCount;
using cv::getTickFrequency;
#define  LOG_TAG    "PedestrianDetection"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define  LOGE(...)  __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)
#define  DEBUG 1
/** Global variables */
char humanDetectionPath[100];
cv::CascadeClassifier humanDetection;
double t;
char evar[200];
std::string hello;
extern "C"
JNIEXPORT void JNICALL
Java_com_serenegiant_activities_MainActivity_humanDetection(JNIEnv *env, jobject instance,
                                                            jlong addrRgba) {

    Mat& frame= *(Mat*) addrRgba;
    if(frame.channels()>0)
        __android_log_write(ANDROID_LOG_DEBUG, LOG_TAG, "has channels");
    if (humanDetection.empty()){
        t = (double)getTickCount();
        sprintf( humanDetectionPath, "%s/%s",evar , "visionary.net_pedestrian_cascade_web_LBP.xml");


        if( !humanDetection.load(humanDetectionPath) ){
            abort();
        };

        t = 1000*((double)getTickCount() - t)/getTickFrequency();
    }

    //-- Detect full body
    t = (double)getTickCount();
    std::vector<Rect> fbody;
    Mat frameGray;
    cvtColor(frame,frameGray,CV_BGR2GRAY);
    equalizeHist(frameGray,frameGray);
    humanDetection.detectMultiScale( frameGray, fbody, 1.1, 2 , 0|CV_HAAR_SCALE_IMAGE , Size(36, 36));  // Size(double width, double height)
    t = 1000*((double)getTickCount() - t)/getTickFrequency();
    // Iterate through all faces and detect eyes
    t = (double)getTickCount();

    for( size_t i = 0; i < fbody.size(); i++ )
    {
        rectangle(frame,Point(fbody[i].x,fbody[i].y),Point(fbody[i].x + fbody[i].width,fbody[i].y + fbody[i].height ),Scalar(0,128,0),4,8,0);
    }

}extern "C"
JNIEXPORT jstring JNICALL
Java_com_serenegiant_services_Assetbridge_loadXml(JNIEnv *env, jobject instance, jstring path_) {
    const char *path = env->GetStringUTFChars(path_, 0);
    strcpy(evar, path);
    strcat(evar, "/");
    putenv(evar);
    hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}