#include "image_follow_line/image_follow_line.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include "filter_line/utils.h"
#include "filter_line/vincent_soille_watershed.h"
#include <ros/console.h>
#include <time.h> 

using namespace cv;
using namespace std;

double near_center(cv::Mat src_image, vector<Point> contours_ploy, std::vector<cv::Vec4f> fitlines )
{
    int img_width = src_image.cols;
    int img_height = src_image.rows;
    double error = 999999999999;
    for(int i =0 ; i < fitlines.size(); i++)
    {
        cv::Vec4f fitline = fitlines[i];
        if(fabs(fitline[0]) < 0.000000001)
            continue;
        double k_line = fitline[1]/fitline[0];
        char text_equation[1024];
        sprintf(text_equation,"y-%.2f=%.2f(x-%.2f)",fitline[3],k_line,fitline[2]);
        cv::Point pm = (contours_ploy[i]+contours_ploy[i+1])*0.5;

        int y = img_height * 0.5;
        double x ;
        x = (y - fitline[3])/k_line + fitline[2];

        double min_x = min(contours_ploy[i].x, contours_ploy[i+1].x);
        double min_y = min(contours_ploy[i].y, contours_ploy[i+1].y);
        double max_x = max(contours_ploy[i].x, contours_ploy[i+1].x);
        double max_y = max(contours_ploy[i].y, contours_ploy[i+1].y);
        if(x >= min_x && x <= max_x)
            if(y >=min_y && y <= max_y)
            {
                double tmp =  img_width * 0.5 - x;
                if(fabs(tmp) < fabs(error))
                    error = tmp;
            }
    }
    return error;
}


cv::Mat thinImage(const cv::Mat & src, const int maxIterations = -1)
{
    assert(src.type() == CV_8UC1);
    cv::Mat dst;
    int width  = src.cols;
    int height = src.rows;
    src.copyTo(dst);
    int count = 0;  //记录迭代次数
    while (true)
    {
        count++;
        if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达
            break;
        std::vector<uchar *> mFlag; //用于标记需要删除的点
        //对点标记
        for (int i = 0; i < height ;++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p+j);
                    }
                }
            }
        }

        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }

        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }

        //对点标记
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p+j);
                    }
                }
            }
        }

        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }
        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }
    }

    return dst;
}



void drawlines(cv::Mat src_image, vector<Point> contours_ploy, std::vector<cv::Vec4f> fitlines )
{

    int img_width = src_image.cols;
    int img_height = src_image.rows;
    for(int i =0 ; i < fitlines.size(); i++)
    {
        cv::Vec4f fitline = fitlines[i];
        if(fabs(fitline[0]) < 0.000000001)
            continue;
        double k_line = fitline[1]/fitline[0];
        char text_equation[1024];
        sprintf(text_equation,"y-%.2f=%.2f(x-%.2f)",fitline[3],k_line,fitline[2]);
        cv::Point pm = (contours_ploy[i]+contours_ploy[i+1])*0.5;
        putText(src_image,text_equation, pm,CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,255),1,8);
        line(src_image,contours_ploy[i],contours_ploy[i+1],Scalar(0,0,255),2);
    }
    imshow("re", src_image);
    cv::waitKey(10);
}


double warnng_line_recognition(cv::Mat im, cv::Mat grayscaleImage){
    int constValue = 10;
    int blockSize = 83; //53/63/33   //轨道在图像中越大，值越大
    int DILATE_SIZE = 3;
    int blur_size = 7;
    bool failed_flag = false;
    //                    B   G   R
    int yellow_scale[] = {0, 130, 130, 100, 255, 255};
 
    int black_scale[] = {0, 0, 0, 50, 50, 50};
 
    cv::medianBlur(grayscaleImage, grayscaleImage, blur_size);
    cv::boxFilter(grayscaleImage, grayscaleImage, -1, cv::Size(blur_size, blur_size));
 
    // 警戒线黄色图像部分分割 
    Mat mask_yellow;
    cv::inRange(im, Scalar(yellow_scale[0], yellow_scale[1], yellow_scale[2]), Scalar(yellow_scale[3], yellow_scale[4], yellow_scale[5]), mask_yellow);
    
    // 警戒线黑色部分分割
    Mat mask_black;
    cv::inRange(im, Scalar(black_scale[0], black_scale[1], black_scale[2]), Scalar(black_scale[3], black_scale[4], black_scale[5]), mask_black);
 
    // 判断是否看到了警戒线
    std::vector < std::vector<Point> > contours;
    findContours(mask_yellow, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    int i =0;
    int area = 100;
    if(contours.size() < 3) 
        failed_flag = true;
    int big_yellow_block = 0;
    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        if(contours[contourIdx].size() > area)
        {
            big_yellow_block++;
        }
    }
    if(big_yellow_block < 2)
        failed_flag = true;
    
    if(cv::countNonZero(mask_yellow) < 700 || cv::countNonZero(mask_black) < 700)
    {
        failed_flag = true; 
    } 
    if(failed_flag == true)
        putText(im,"can not find line!!!",Point(50,60),FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),4,8);//在图片上写文字
    
    // 合并黑色和黄色图像，进行闭运算
    cv::Mat seg_line = mask_yellow | mask_black;
    Mat element = getStructuringElement(MORPH_RECT,	Size(7, 7));
    
    Mat close_result;
	morphologyEx(seg_line, close_result, MORPH_CLOSE, element);
    
    // 根据轮廓和像素逻辑运算将不属于警戒线的区域去掉
    std::vector < std::vector<Point> > Icontour;
    vector<Vec4i> hierarchy;
    findContours(close_result,Icontour,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
    area = 0;
    double maxArea=0;
    Mat black_mat;
    Mat warning_line = Mat(im.size(), CV_8UC1,Scalar(0)); 
    for(int i=0;i<Icontour.size();i++)  
    {  
        black_mat = Mat(im.size(), CV_8UC1,Scalar(0)); 
        drawContours(black_mat,Icontour,i,Scalar(255),CV_FILLED,8,hierarchy);
        cv::threshold(black_mat, black_mat, 125, 255, cv::THRESH_BINARY);
        int cover_size = cv::countNonZero(black_mat|mask_yellow);
        if(cv::countNonZero(black_mat&mask_yellow) > 1500)
            drawContours(im,Icontour,i,Scalar(255,0,0),CV_FILLED,8,hierarchy);

        if(cv::countNonZero(black_mat&mask_yellow) > 1000)
        {
            drawContours(im,Icontour,i,Scalar(255,0,0),CV_FILLED,8,hierarchy);
            drawContours(warning_line,Icontour,i,Scalar(255,0,0),CV_FILLED,8,hierarchy);
        } 
    }
    cv::threshold(warning_line, warning_line, 125, 255, cv::THRESH_BINARY); 

    
    cv::imshow("warning_line", warning_line);
    waitKey(1); 

    // 将警戒线分割出后提取中心线
    if(countNonZero(warning_line) > 10)
    { 
        //cv::Mat timg = thinImage(warning_line);
        //连通域分析
        std::vector < std::vector<Point> > contours2;
        findContours(warning_line, contours2, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        int max_area_id =0;
        int max_area = 0;
        for (size_t contourIdx = 0; contourIdx < contours2.size(); contourIdx++)
        {
            if(contours2[contourIdx].size() > max_area)
            {
                area = contours2[contourIdx].size();
                max_area_id = contourIdx;
            }
        }

        //多折线拟合
        vector<Point> contours_ploy;
        approxPolyDP(contours2[max_area_id], contours_ploy, 5, true);
        std::vector<cv::Vec4f> fitlines;
        for(size_t i = 0; i < contours_ploy.size()-1; i++)
        {
            std::vector<cv::Point> pcur;
            pcur.push_back(contours_ploy[i]);
            pcur.push_back(contours_ploy[i+1]);
            Vec4f fitline;
            //拟合方法采用最小二乘法
            cv::fitLine(pcur, fitline, CV_DIST_L2,0,0.01,0.01);
            fitlines.push_back(fitline);
        }
        if(failed_flag)
            return 9999999;
        drawlines(im, contours_ploy, fitlines);
        cv::imshow("im", im);
        waitKey(1);
        return  near_center(im, contours_ploy, fitlines);  
    }

    //cv::imshow("im", im);
    //waitKey(1);
    if(failed_flag)
        return 9999999; 
    return 0.0;

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_follow_line_node");
    control control_;

     VideoCapture cam_front;
     VideoCapture cam_back;
     time_t first, second; 
     double kp = 0;
     double ki = 0;
     double kd = 0;
     double vc = 0;
     int run_time = 0;
     if(argc >= 3)
     {
        cam_front.open(argv[1]);//front 
        kp = atof(argv[2]);
        ki = atof(argv[3]); 
        vc = atof(argv[4]);
        run_time = atoi(argv[5]);
     }

    if(!cam_front.isOpened()){
        std::cout<<"cam_front open failed!";
        exit(0);
    } 

    Mat srcframe_front; 
    
    unique_ptr<thread> read_camera;

    ros::Rate loop_rate(10); 

    read_camera = make_unique<thread>([&](){
        while(true){
            cam_front >> srcframe_front;
            double error;
            if(srcframe_front.empty())
                continue;
             cv::Mat grayscaleImage;
             if(srcframe_front.channels() == 3)
                cv::cvtColor(srcframe_front, grayscaleImage, CV_BGR2GRAY);
             error = warnng_line_recognition(srcframe_front, grayscaleImage);
             if(error < 999999)
             { 
                // ROS_INFO_STREAM("error is:" << error);
                control_.CheckErrorFront(error, vc, kp, ki, kd); 
             }
             
            this_thread::sleep_for(chrono::microseconds(20));
            second=time(NULL); 
            if(difftime(second,first) > run_time)
            {
                std::cout << "time out!!!!!!!!!!!!!!!!" << std::endl;
                control_.CheckErrorFront(error, 0, 0, 0, 0);
                break;
            } 
        }
        cout << "线程结束运行" << endl; 
    });


    ros::spin(); 
}