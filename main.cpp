#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <opencv2/opencv.hpp>
#include "rich_point.h"
using namespace std;
using namespace cv;
using namespace depth_clustering;

//set the path components
string imageDirL = "image_2/";
string imageDirR = "image_3/";
string laserDir = "velodyne/";
string calibFileName = "calib.txt";
string poseFileName = "pose.txt";
string imgPatt = "%06d.png";
string laserPatt = "%06d.bin";

//载入相机投影矩阵
void loadCalib(const std::string calibFilePath,cv::Mat& calibP0,cv::Mat& calibTr)
{
    //TODO:下面应该可能需要加异常处理，防止calib.txt文件中数据不全
    ifstream calibFile(calibFilePath.c_str());
    if(calibFile.good())
    {
        string input;
        stringstream stringin;
        calibP0 = Mat(3,4,CV_32FC1);
        calibTr = Mat(3,4,CV_32FC1);
        string temp;


        getline(calibFile,input);//读入一行赋值给input,PO
        stringin = stringstream(input);
        stringin>>temp;
        for(int r = 0; r<calibP0.rows;++r)
            for(int c = 0;c<calibP0.cols;++c)
            {
                string num;
                stringin>>num;
                calibP0.at<float>(r,c) = stof(num);
            }


        getline(calibFile,input);//读入一行赋值给input,P1
        getline(calibFile,input);//读入一行赋值给input
        getline(calibFile,input);//读入一行赋值给input

        getline(calibFile,input);//读入一行赋值给input,Tr
        stringin = stringstream(input);//读入一行赋值给input,PO
        stringin>>temp;
        for(int r = 0; r<calibTr.rows;++r)
            for(int c = 0;c<calibTr.cols;++c)
            {
                string num;
                stringin>>num;
                calibTr.at<float>(r,c) = stof(num);
            }
        calibFile.close();

    } else
        return;



}

void readLaserBinFile(const std::string& laserFileName,int fileIdx,std::vector<RichPoint>& laserPt )
{
    char temp[20];
    sprintf(temp,"%06d.bin",fileIdx);
    string full_file_path = laserFileName+ static_cast<string>(temp);

    std::fstream file(full_file_path.c_str(), std::ios::in | std::ios::binary);

    if (file.good()) {
        file.seekg(0, std::ios::beg);
        float intensity = 0;
        for (int i = 0; file.good() && !file.eof(); ++i) {
            RichPoint point;
            file.read(reinterpret_cast<char*>(&point.x()), sizeof(float));
            file.read(reinterpret_cast<char*>(&point.y()), sizeof(float));
            file.read(reinterpret_cast<char*>(&point.z()), sizeof(float));
            // ignore intensity
            file.read(reinterpret_cast<char*>(&intensity), sizeof(float));
            laserPt.push_back(point);
        }
        file.close();
    }

}



#if 0
void readLaserBinFile(const std::string& laserFileName,vector<cv::Mat>& laserPt)
{
    std::fstream file(laserFileName.c_str(), std::ios::in | std::ios::binary);
    cv::Mat pt = cv::Mat::zeros(4,1,CV_32FC1);

    if (file.good()) {
        //for test
        file.seekg(0,file.end);
        std::cout<<"file size = "<<file.tellg()/sizeof(float)<<endl;
        file.seekg(0, std::ios::beg);
        float intensity = 0;
        for (int i = 0; file.good() && !file.eof(); ++i) {
            file.read(reinterpret_cast<char*>(&(pt.at<float>(0,0))), sizeof(float));
            file.read(reinterpret_cast<char*>(&(pt.at<float>(1,0))), sizeof(float));
            file.read(reinterpret_cast<char*>(&(pt.at<float>(2,0))), sizeof(float));
            // ignore intensity
            file.read(reinterpret_cast<char*>(&intensity), sizeof(float));
            laserPt.push_back(pt);
        }
        file.close();
    }


}
#endif


void readImage(const std::string& imagePath,int imgIdx,cv::Mat& imgRead)
{
    char temp[100];
    sprintf(temp,"%06d.png",imgIdx);
    string full_image_path = imagePath+ static_cast<string>(temp);
//    string full_image_path = imagePath+ "000000.png";

    imgRead = cv::imread(full_image_path);

}

void showLaserImage(std::vector<RichPoint>& laserPt,cv::Mat& rawImg,cv::Mat& calibTr,cv::Mat& calibP0)
{
    FILE* savePt;
    savePt = fopen("save_pixel.txt","w");
    //cv::Mat transformMat = calibP0*calibTr;
    float ptL1,ptL2,ptL3;
    cv::Mat camWorldPt = cv::Mat::ones(4,1,CV_32FC1);
    //遍历所有点云
    for(int ptNum = 0;ptNum<laserPt.size();++ptNum)
    {

            camWorldPt.at<float>(0,0) =
                    calibTr.at<float>(0,0)*laserPt[ptNum].x()+
                    calibTr.at<float>(0,1)*laserPt[ptNum].y()+
                    calibTr.at<float>(0,2)*laserPt[ptNum].z()+
                    calibTr.at<float>(0,3)*1;
            camWorldPt.at<float>(1,0) =
                    calibTr.at<float>(1,0)*laserPt[ptNum].x()+
                    calibTr.at<float>(1,1)*laserPt[ptNum].y()+
                    calibTr.at<float>(1,2)*laserPt[ptNum].z()+
                    calibTr.at<float>(1,3)*1;
            camWorldPt.at<float>(2,0) =
                    calibTr.at<float>(2,0)*laserPt[ptNum].x()+
                    calibTr.at<float>(2,1)*laserPt[ptNum].y()+
                    calibTr.at<float>(2,2)*laserPt[ptNum].z()+
                    calibTr.at<float>(2,3)*1;
        if(camWorldPt.at<float>(2,0)<=2)
            continue;

        cv::Mat transformMat = calibP0*camWorldPt;
        ptL1 = transformMat.at<float>(0,0)/transformMat.at<float>(2,0);
        ptL2 = transformMat.at<float>(1,0)/transformMat.at<float>(2,0);
        if(ptL1>0&&ptL2>0&&ptL1<rawImg.cols&&ptL2<rawImg.rows)
        {
//            fprintf(savePt,"%.4f %.4f\n",ptL1,ptL2);
            cv::circle(rawImg,cv::Point(ptL1,ptL2),1,cv::Scalar(255,0,0));

        }

    }


}


int main() {
    //根目录所在路径
    string seqDirPath = "/home/zhanghm/Datasets/KITTI_Dataset/odometry/01/";
    string calibFilePath = seqDirPath+calibFileName;

    Mat calibTr,calibP0;
    //载入相机投影矩阵calibP0和相机与激光雷达坐标变换矩阵
    loadCalib(calibFilePath,calibP0,calibTr);
    cout<<"calibP0 = "<<calibP0<<endl;
    cout<<"calibTr = "<<calibTr<<endl;

    int start = 0; //开始
    namedWindow("fusion_image",CV_WINDOW_NORMAL);

    FILE* savePt;
    savePt = fopen("save_pt.txt","w");
    //循环载入图片和激光雷达数据序号
    while(1)
    {
        //1)读入二进制雷达数据
        string laserFileName = seqDirPath+laserDir;
        std::vector<RichPoint> laserPt;
        readLaserBinFile(laserFileName,start,laserPt);
//        for(int ptNum = 0;ptNum<laserPt.size();++ptNum)
//        {
//            fprintf(savePt,"%.4f %.4f %.4f\n",laserPt[ptNum].x(),
//            laserPt[ptNum].y(),
//            laserPt[ptNum].z());
//        }


        //2)读入图片数据
        string imgFileName = seqDirPath+imageDirL;
        Mat imgSrc;
        readImage(imgFileName,start,imgSrc);

        //3)激光点云投影处理
        showLaserImage(laserPt,imgSrc,calibTr,calibP0);

        //4)可视化
        imshow("fusion_image",imgSrc);
        ++start;

        int key = waitKey(1);
        if(key == 32) //按空格
        {
            key = waitKey(0);
        }
        else if(key == 'q') //按q键，退出
        {
            break;
        }




    }





    return 0;
}