#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <opencv2/opencv.hpp>
#include "kitti_read_utils.h"
using namespace std;
using namespace cv;
using namespace kitti_read;

//set the path components
string imageDirL = "image_2/";
string imageDirR = "image_3/";
string laserDir = "velodyne/";
string calibFileName = "calib.txt";
string poseFileName = "pose.txt";
string imgPatt = "%06d.png";
string laserPatt = "%06d.bin";
int main(){
    //根目录所在路径
    string seqDirPath = "/home/zhanghm/Datasets/KITTI_Dataset/odometry/01/";
    string calibFilePath = seqDirPath+calibFileName;

    Mat projectionMatrix,cameraMatrix,velo2camera;
    //载入相机投影矩阵calibP0和相机与激光雷达坐标变换矩阵
    readOdometryCalib(calibFilePath,projectionMatrix,cameraMatrix,velo2camera);
//    cout<<"projectionMatrix = "<<projectionMatrix<<endl;
//    cout<<"cameraMatrix = "<<cameraMatrix<<endl;
//    cout<<"velo2camera = "<<velo2camera<<endl;

    int start = 0; //开始
    namedWindow("fusion_image",CV_WINDOW_NORMAL);

    FILE* savePt;
    savePt = fopen("save_pt.txt","w");


    //循环载入图片和激光雷达数据序号
    while(1)
    {
        //store the 3D point cloud, shared pointer, don't need delete it manually
        pcl::PointCloud<pcl::PointXYZ>::Ptr veloCloudPtr_bin (new pcl::PointCloud<pcl::PointXYZ>);

        //1)读入二进制雷达数据
        string laserFileName = seqDirPath+laserDir;
        readLidarBinData(laserFileName,start,veloCloudPtr_bin);


        //2)读入图片数据
        string imgFileName = seqDirPath+imageDirL;
        Mat imgSrc;
        readImage(imgFileName,start,imgSrc);

        //3)激光点云投影处理
        project2Image(imgSrc,veloCloudPtr_bin,projectionMatrix);

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