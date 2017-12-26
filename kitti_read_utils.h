//
// Created by zhanghm on 17-12-26.
//

#ifndef KITTI_READ_KITTI_READ_UTILS_H
#define KITTI_READ_KITTI_READ_UTILS_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//Eigen
#include <Eigen/Dense>

namespace kitti_read{
    bool readOdometryCalib(const std::string& calibFile,cv::Mat& projectionMatrix,cv::Mat& cameraMatrix2,cv::Mat& velo2camera);

    //! \brief read image sequence
    //! \param imagePath image parent diretory name, include '/'
    //! \param imgIdx image sequence number
    //! \param imgRead[out] the image has read
    //! \return
    bool readImage(const std::string& imagePath,const int imgIdx,cv::Mat& imgRead);

    //! \brief read point cloud from binary file
    // veloCloudPtr[out] store all point cloud data
    bool readLidarBinData(const std::string& lidarFile,const int fileIdx,pcl::PointCloud<pcl::PointXYZ>::Ptr veloCloudPtr);


    bool project2Image(cv::Mat& rgbImg,pcl::PointCloud<pcl::PointXYZ>::Ptr veloCloudPtr,cv::Mat& projectionMatrix);

}
#endif //KITTI_READ_KITTI_READ_UTILS_H
