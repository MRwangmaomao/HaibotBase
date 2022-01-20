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

int inline neighborCleanup(float* in, uchar* out, int i, int x, int y, int x_lim, int y_lim)
{
   int index;
   for (int xx = x - 1; xx < x + 2; ++xx) {
       for (int yy = y - 1; yy < y + 2; ++yy) {
           if (((xx == x) && (yy==y)) || xx < 0 || yy < 0 || xx >= x_lim || yy >= y_lim)
               continue;
           index = xx*y_lim + yy;
           if ((in[i] == in[index]) && (out[index] == 0))
               return 1;
       }
   }

   return 0;
}

void inline neighborCheck(float* in, uchar* out, int i, int x, int y, int x_lim)
{
   int indexes[8], cur_index;
   indexes[0] = x*x_lim + y;
   indexes[1] = x*x_lim + y+1;
   indexes[2] = x*x_lim + y+2;
   indexes[3] = (x+1)*x_lim + y+2;
   indexes[4] = (x + 2)*x_lim + y+2;
   indexes[5] = (x + 2)*x_lim + y + 1;
   indexes[6] = (x + 2)*x_lim + y;
   indexes[7] = (x + 1)*x_lim + y;
   cur_index = (x + 1)*x_lim + y+1;

   for (int t = 0; t < 8; t++) {
       if (in[indexes[t]] < in[cur_index]) {
           out[i] = 0;
           break;
       }
   }

   if (out[i] == 3)
       out[i] = 1;
}


//  output is a binary image
//  1: not a min region
//  0: part of a min region
//  2: not sure if min or not
//  3: uninitialized
void imregionalmin(cv::Mat& img, cv::Mat& out_img)
{
    // pad the border of img with 1 and copy to img_pad
    cv::Mat img_pad;
    cv::copyMakeBorder(img, img_pad, 1, 1, 1, 1, IPL_BORDER_CONSTANT, 1);

    //  initialize binary output to 2, unknown if min
    out_img = cv::Mat::ones(img.rows, img.cols, CV_8U)+2;

    //  initialize pointers to matrices
    float* in = (float *)(img_pad.data);
    uchar* out = (uchar *)(out_img.data);

    //  size of matrix
    int in_size = img_pad.cols*img_pad.rows;
    int out_size = img.cols*img.rows;

    int x, y;
    for (int i = 0; i < out_size; i++) {
        //  find x, y indexes
        y = i % img.cols;
        x = i / img.cols;

        neighborCheck(in, out, i, x, y, img_pad.cols);  //  all regions are either min or max
    }

    cv::Mat label;
    cv::connectedComponents(out_img, label);
    int* lab = (int *)(label.data);

   in = (float *)(img.data);
   in_size = img.cols*img.rows;

   std::vector<int> bad_labels;

   for (int i = 0; i < out_size; i++) {
       //  find x, y indexes
       y = i % img.cols;
       x = i / img.cols;

       if (lab[i] != 0) {
           if (neighborCleanup(in, out, i, x, y, img.rows, img.cols) == 1) {
               bad_labels.push_back(lab[i]);
           }
       }
   }

   std::sort(bad_labels.begin(), bad_labels.end());
   bad_labels.erase(std::unique(bad_labels.begin(), bad_labels.end()), bad_labels.end());

   for (int i = 0; i < out_size; ++i) {
       if (lab[i] != 0) {
           if (std::find(bad_labels.begin(), bad_labels.end(), lab[i]) != bad_labels.end()) {
               out[i] = 0;
           }
       }
   }
}





void non_maxima_suppression(const cv::Mat& src, const int h, cv::Mat& mask, const bool remove_plateaus)
{
    // find pixels that are equal to the local neighborhood not maximum (including 'plateaus')
    cv::dilate(src, mask, cv::Mat());
    cv::compare(src, mask, mask, cv::CMP_GE);

    // optionally filter out pixels that are equal to the local minimum ('plateaus')
    if (remove_plateaus) {
        cv::Mat non_plateau_mask;
        cv::erode(src, non_plateau_mask, cv::Mat());
        cv::compare(src, non_plateau_mask, non_plateau_mask, cv::CMP_GT);
        cv::bitwise_and(mask, non_plateau_mask, mask);
    }
}

//cv::Mat imhmin(cv::Mat in, int h)
//{
//    cv::Mat mask;
//    non_maxima_suppression(in, h, mask, true);
//    return mask;
//}

void imextendedmin(cv::Mat in, int h, cv::Mat& out)
{
    cv::Mat mhmin = imhmin(in, h);

    out = imregionalmin(mhmin);
    //imregionalmin(mhmin,out);
}
cv::Mat imcomplement(cv::Mat src)
{
    cv::Mat dst;
    if(src.type() == CV_32FC1)
        dst = 1.0 - src;
    else
         dst = 255 - src;
    return dst;
}
void Reconstruct(Mat marker, Mat mask, Mat& dst)
{
    Mat se=getStructuringElement(MORPH_RECT,Size(3,3));
    Mat tmp1(marker.size(), marker.type()), tmp2(marker.size(), marker.type());
    cv::min(marker, mask, dst);

    do {
        dst.copyTo(tmp1);
        dilate(dst, mask, se);
        cv::min(marker, mask, dst);
        tmp2=abs(tmp1-dst);
    } while (sum(tmp2).val[0] != 0);
}

cv::Mat imimposemin(cv::Mat I, cv::Mat BW )
{
    cv::Mat fm = I.clone();
    for(size_t i = 0 ; i < fm.rows; i++)
        for(size_t j =0 ; j < fm.cols; j++)
        {
            if(BW.at<uchar>(i, j) > 0)
            {
                fm.at<float>(i,j) = -FLT_MAX;
            }
            else
               fm.at<float>(i,j) = FLT_MAX;
        }

    double min, max;
    cv::minMaxIdx(I, &min, &max);
    float range = max - min;
    float h = 0;

    if(I.type() == CV_32FC1)
    {
        if(fabs(range) < 0.000001 )
        {
            h = 0.1;
        }
        else
        {
            h = range*0.001;
        }
    }
    else
        h = 1;
    cv::Mat fp1 = I + h;
    cv::Mat g(fp1.size(), CV_32FC1);

    for(size_t i = 0 ; i < fm.rows; i++)
        for(size_t j =0 ; j < fm.cols; j++)
        {
            float a = fp1.at<float>(i,j);
            float b = fm.at<float>(i,j);
            g.at<float>(i,j) = MIN(a,b);
        }

    cv::Mat dst;
    cv::Mat g1 = imcomplement(g);
    cv::Mat fm1 = imcomplement(fm);

   dst = imreconstruct(fm1, g1);
  //  Reconstruct(fm1, g1, dst);
    dst = imcomplement(dst);

    return dst;
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




cv::Mat watershedlocal(cv::Mat gray_image)
{
    int nb_row= gray_image.rows;
    int nb_col= gray_image.cols;

    gray_image.convertTo(gray_image, CV_64FC1);

    cppimage input_watershed_image(nb_row, nb_col, (double*)(gray_image.ptr()));

    vincent_soille_watershed vsw;
    vsw.process_watershed_algo(input_watershed_image, 4);

    cv::Mat output_image(gray_image.size(), CV_64FC1);
    vsw.get_labelled_array((double*)output_image.ptr());

    cv::Mat dst;
    output_image.convertTo(dst, CV_8UC1);
    return dst;
}

cv::Mat imageSegmnet(cv::Mat grayImg, cv::Mat im)
{

    Mat Ix, Iy;

 //   #pragma omp parallel sections num_threads(2)
    {
   //     #pragma omp section
        {
        Sobel(grayImg, Ix, grayImg.type(), 1, 0, 3, 1.0, 0.0, BORDER_DEFAULT);
        }
 //       #pragma omp section
        {
        Sobel(grayImg, Iy, grayImg.type(), 0, 1, 3, 1.0, 0.0, BORDER_DEFAULT);
        }
    }
    Mat angleGrad(Ix.size(), CV_32FC1);
    Mat magGrad(Ix.size(), CV_32FC1);
    cartToPolar(Ix, Iy, magGrad, angleGrad, true);



    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    morphologyEx(magGrad, magGrad, MORPH_OPEN, element);
    morphologyEx(magGrad, magGrad, MORPH_CLOSE, element);


    cv::Mat g2 = magGrad.clone();
   //cv::imwrite("g2.png", g2);

    cv::Mat out;
    imextendedmin(magGrad, 10, out);
    cv::Mat dstmat, label;
    cv::Mat outd = imcomplement(out);
    distanceTransform(outd, dstmat, cv::DIST_L2, 3);
    cv::Mat marks= watershedlocal(dstmat);

    cv::Mat em = (marks == 0);
    cv::Mat bwm = out | em;
    cv::Mat g3 = imimposemin(g2, bwm);

    for(int i = 0 ; i < g3.rows; i++)
        for(int j =0 ; j < g3.cols; j++)
        {
            if(g3.at<float>(i,j) < 0)
                g3.at<float>(i,j) = 0;
        }

    cv::Mat g4 = watershedlocal(g3);

    cv::Mat re = (g4 == 0);
    cv::Mat dilatedI;
    Mat element1 = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(re, dilatedI, MORPH_DILATE, element1);
   // dilatedI =  dilatedI;
    floodFill(dilatedI, Point(0,0), Scalar(10));
    floodFill(dilatedI, Point(re.cols -1,0), Scalar(10));
    floodFill(dilatedI, Point(re.cols -1, re.rows-1), Scalar(10));
    floodFill(dilatedI, Point(re.cols -1, re.rows-1), Scalar(10));
    cv::Mat result = (dilatedI != 10);
    std::cout<<"segment end!\n";
    return result;


}

void drawline(cv::Mat src_image, cv::Vec4f fitline )
{
       int img_width = src_image.cols;
        int img_height = src_image.rows;

        double k_line = fitline[1]/fitline[0];
        Point p1(0,k_line*(0 - fitline[2]) + fitline[3]);
        Point p2(img_width - 1,k_line*(img_width - 1 - fitline[2]) + fitline[3]);


        char text_equation[1024];
        sprintf(text_equation,"y-%.2f=%.2f(x-%.2f)",fitline[3],k_line,fitline[2]);
        putText(src_image,text_equation,Point(30,50),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,255),1,8);

        line(src_image,p1,p2,Scalar(0,0,255),2);
        imshow("re", src_image);
        // cv::waitKey(0);
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

bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
{
    //Number of key points
    int N = key_point.size();

    //构造矩阵X
    cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int j = 0; j < n + 1; j++)
        {
            for (int k = 0; k < N; k++)
            {
                X.at<double>(i, j) = X.at<double>(i, j) +
                    std::pow(key_point[k].x, i + j);
            }
        }
    }

    //构造矩阵Y
    cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    for (int i = 0; i < n + 1; i++)
    {
        for (int k = 0; k < N; k++)
        {
            Y.at<double>(i, 0) = Y.at<double>(i, 0) +
                std::pow(key_point[k].x, i) * key_point[k].y;
        }
    }

    A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
    //求解矩阵A
    cv::solve(X, Y, A, cv::DECOMP_LU);
    return true;
}


void drawcurves(cv::Mat image, cv::Mat A)
{
    std::vector<cv::Point> points_fitted;

    for (int x = 0; x < image.cols; x++)
    {
        double y = A.at<double>(0, 0) + A.at<double>(1, 0) * x +
            A.at<double>(2, 0)*std::pow(x, 2) + A.at<double>(3, 0)*std::pow(x, 3);

        points_fitted.push_back(cv::Point(x, y));
    }
    cv::polylines(image, points_fitted, false, cv::Scalar(0, 255, 255), 1, 8, 0);
    cv::imshow("image", image);
    cv::waitKey(10);
}

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

double imagelinefit_simplethresh(cv::Mat im, cv::Mat grayscaleImage)
{

      int constValue = 10;
      int blockSize = 83; //53/63/33   //轨道在图像中越大，值越大
      int DILATE_SIZE = 3;
      int blur_size = 7;


    cv::medianBlur(grayscaleImage, grayscaleImage, blur_size);
    cv::boxFilter(grayscaleImage, grayscaleImage, -1, cv::Size(blur_size, blur_size));
    cv::Mat segimg;

    //图像分割
    const int maxVal = 255;


    int adaptiveMethod = 0;
    int thresholdType = 1;
    adaptiveThreshold(grayscaleImage, segimg, maxVal, adaptiveMethod, thresholdType, blockSize, constValue);

    cv::Mat segimg1;
     cv::threshold(grayscaleImage, segimg1, 125, 255, cv::THRESH_BINARY_INV);

    //cv::imshow("segimg", segimg);
    //cv::imshow("segimg1", segimg1);


    segimg = segimg&segimg1;

    int n = cv::countNonZero(segimg);
    if(n < 5)return 99999999;

   cv::Mat re = segimg;
   cv::Mat dilatedI;
   Mat element1 = getStructuringElement(MORPH_RECT, Size(DILATE_SIZE, DILATE_SIZE));
    morphologyEx(re, dilatedI, MORPH_DILATE, element1);
  #if 0


   floodFill(dilatedI, Point(0,0), Scalar(10));
   floodFill(dilatedI, Point(re.cols -1,0), Scalar(10));
   floodFill(dilatedI, Point(re.cols -1, re.rows-1), Scalar(10));
   floodFill(dilatedI, Point(re.cols -1, re.rows-1), Scalar(10));
   cv::Mat result = (dilatedI != 10);
#else
  cv::Mat result= dilatedI;
#endif
//细化
   cv::threshold(result, result, 128, 1, cv::THRESH_BINARY);
   cv::Mat timg = thinImage(result);

    //连通域分析
   std::vector < std::vector<Point> > contours;
   findContours(timg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
   int i =0;
   int area = 0;
   for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
   {
       if(contours[contourIdx].size() > area)
       {
           area = contours[contourIdx].size();
           i = contourIdx;
       }
   }

   //曲线直线化

#if 1
   //多折线拟合
    vector<Point> contours_ploy;
    approxPolyDP(contours[i], contours_ploy, 5, true);
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
       drawlines(im, contours_ploy, fitlines);
       return  near_center(im, contours_ploy, fitlines);
#else
   //曲线拟合
   cv::Mat A;

   polynomial_curve_fit(contours[i], 3, A);
   drawcurves(im, A);

#endif
}

#if 1

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_follow_line_node");
    control control_;
    time_t first, second; 
    first=time(NULL); 
     VideoCapture cam_front;
     VideoCapture cam_back;
     double kp = 0;
     double ki = 0;
     double kd = 0;
     double vc = 0;
     int run_time = 0;
     if(argc >=3)
     {
        cam_front.open(argv[1]);//front
        cam_back.open(argv[2]);//back
        kp = atof(argv[3]);
        ki = atof(argv[4]);
        // kd = atof(argv[5]);
        vc = atof(argv[5]);
        run_time = atoi(argv[6]);
     }

    if(!cam_front.isOpened()){
        std::cout<<"cam_front open failed!";
        exit(0);
    }

//    if(!cam_back.isOpened()){
//        std::cout<<"cam_back open failed!"
//        exit(0);
//    }

    Mat srcframe_front;
    Mat srcframe_back;
    
    unique_ptr<thread> read_camera;

    ros::Rate loop_rate(10);
    // namedWindow("view_front", CV_WINDOW_NORMAL);
    // namedWindow("view_back", CV_WINDOW_NORMAL);

    read_camera = make_unique<thread>([&](){
        while(true){
            cam_front >> srcframe_front;
            double error;
            if(srcframe_front.empty())
                continue;
             cv::Mat grayscaleImage;
             if(srcframe_front.channels() == 3)
                cv::cvtColor(srcframe_front, grayscaleImage, CV_BGR2GRAY);
             error = imagelinefit_simplethresh(srcframe_front, grayscaleImage);
             if(error < 999999)
             { 
                ROS_INFO_STREAM("error is:" << error);
                control_.CheckErrorFront(error, vc, kp, ki, kd);
             }
            // cam_back >> srcframe_back;
            // imshow("view_back", srcframe_back);
            // waitKey(5);
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

#else

int main(int argc, char *argv[])
{
  std::cout<<"begin.!\n";

  std::string path = "/home/ly/workspace/dataset/imageline/1/";

 int i = 1;
 int n = 2000;
  while (i++)
  {
      if(i > n)break;
      char pix[256];
      sprintf(pix, "%d.png", i);
      std::string image_file = path+std::string(pix);
      std::cout<<image_file<<std::endl;
      cv::Mat im = cv::imread(image_file);
      if(im.empty())
          continue;
       cv::Mat grayscaleImage;
       if(im.channels() == 3)
          cv::cvtColor(im, grayscaleImage, CV_BGR2GRAY);
      imagelinefit_simplethresh(im, grayscaleImage);
  }

   std::cout<<"end"<<std::endl;

    return 0;
}


#endif






