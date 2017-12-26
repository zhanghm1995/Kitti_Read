//
// Created by zhanghm on 17-12-26.
//
#include  "kitti_read_utils.h"
#include <sstream>

namespace kitti_read{
    using std::string;
    using std::vector;

    bool readOdometryCalib(const std::string& calibFile,cv::Mat& projectionMatrix,cv::Mat& cameraMatrix,cv::Mat& velo2camera)
    {
        //project 3D point under camera0 coordinate to other camera image , to image of camera2 by default
        cameraMatrix = cv::Mat(3,4,CV_32F,cv::Scalar::all(0));
        velo2camera = cv::Mat(4,4,CV_32F,cv::Scalar::all(0)); //Tr,velodyne lidar point cloud to camera0 world coordinate
        velo2camera.at<float>(3,3) = 1.0;

        std::ifstream inputFile;
        inputFile.open(calibFile.c_str(),std::ios_base::in);
        if(!inputFile.is_open())
        {
            fprintf(stderr, "cannot open the calibration file: %s!\n", calibFile.c_str());
            return false;
        }

        while( !inputFile.eof() )
        {
            std::string line_tmp;
            std::getline(inputFile,line_tmp); //read one line
            std::istringstream inputString(line_tmp);//define a string stream
            std::string tag;
            inputString>>tag;

            //just read the color camera of P2 for this function
            if ( tag == "P2:")
            {
                inputString>> cameraMatrix.at<float>(0,0) >> cameraMatrix.at<float>(0,1) >> cameraMatrix.at<float>(0,2) >> cameraMatrix.at<float>(0,3)
                           >> cameraMatrix.at<float>(1,0) >> cameraMatrix.at<float>(1,1) >> cameraMatrix.at<float>(1,2) >> cameraMatrix.at<float>(1,3)
                           >> cameraMatrix.at<float>(2,0) >> cameraMatrix.at<float>(2,1) >> cameraMatrix.at<float>(2,2) >> cameraMatrix.at<float>(2,3);
            }

            if(tag == "Tr:")
            {
                inputString >> velo2camera.at<float>(0,0) >> velo2camera.at<float>(0,1) >> velo2camera.at<float>(0,2) >> velo2camera.at<float>(0,3)
                            >> velo2camera.at<float>(1,0) >> velo2camera.at<float>(1,1) >> velo2camera.at<float>(1,2) >> velo2camera.at<float>(1,3)
                            >> velo2camera.at<float>(2,0) >> velo2camera.at<float>(2,1) >> velo2camera.at<float>(2,2) >> velo2camera.at<float>(2,3);
            }

        }
        inputFile.close();

        //projectionMatrix = 3X4;
        projectionMatrix = cameraMatrix*velo2camera;//project 3D point in lidar to camera image coordinate

        return true;

    }

    bool readImage(const std::string& imagePath,const int imgIdx,cv::Mat& imgRead)
    {
        char temp[100];
        sprintf(temp,"%06d.png",imgIdx);
        string full_image_path = imagePath+ static_cast<string>(temp);
//    string full_image_path = imagePath+ "000000.png";

        imgRead = cv::imread(full_image_path);
    }


    bool readLidarBinData(const std::string& lidarFile,const int fileIdx,pcl::PointCloud<pcl::PointXYZ>::Ptr veloCloudPtr)
    {
        char temp[20];
        sprintf(temp,"%06d.bin",fileIdx);
        string full_file_path = lidarFile + static_cast<string>(temp);

        FILE* veloData;
        veloData = fopen(full_file_path.c_str(),"rb");

        if( veloData == NULL )
        {
            fprintf(stderr,"falied to load the velodyne file %s!\n", full_file_path.c_str());
            return false;
        }

        int32_t veloDataNum = 1000000;
        float* data = (float*)malloc(veloDataNum*sizeof(float));
        float* px = data + 0;
        float* py = data + 1;
        float* pz = data + 2;
        float* pr = data + 3;

        veloDataNum = fread(data,sizeof(float),veloDataNum,veloData)/4; //return the point cloud number

        for(int32_t i = 0; i < veloDataNum; i++) {
            pcl::PointXYZ pointTmp;
            pointTmp.x = *px;
            pointTmp.y = *py;
            pointTmp.z = *pz;

            veloCloudPtr->points.push_back(pointTmp);

            px += 4;
            py += 4;
            pz += 4;
            pr += 4;
        }

        free(data);
        fclose(veloData);

        veloCloudPtr->width = (int) veloCloudPtr->points.size();
        veloCloudPtr->height = 1;

        return true;


    }


    bool project2Image(cv::Mat& rgbImg,pcl::PointCloud<pcl::PointXYZ>::Ptr veloCloudPtr,cv::Mat& projectionMatrix)
    {
        if(!rgbImg.data||veloCloudPtr)
        {
            printf("No valid rgbImage or valid point cloud data!");
            return false;
        }

        for(size_t i = 0; i<veloCloudPtr->points.size();++i) //scan all point cloud
        {
            cv::Mat veloPoint3D(4,1,CV_32F,cv::Scalar(1.0));
            cv::Mat imgPoint2D(3,1,CV_32F);

            veloPoint3D.at<float>(0,0) = veloCloudPtr->points[i].x;
            veloPoint3D.at<float>(1,0) = veloCloudPtr->points[i].y;
            veloPoint3D.at<float>(2,0) = veloCloudPtr->points[i].z;

            //eliminate the point behind camera
            if(veloCloudPtr->points[i].x < 0)
                continue;

            imgPoint2D = projectionMatrix*veloPoint3D;
            if( imgPoint2D.at<float>(2,0) == 0 )
            {
                fprintf(stderr,"the calculated 2D image points are wrong!\n");
                exit(0);
            }
            //scale transform
            imgPoint2D.at<float>(0,0) /= imgPoint2D.at<float>(2,0); //row number
            imgPoint2D.at<float>(1,0) /= imgPoint2D.at<float>(2,0); //col number

            int colTmp = int(imgPoint2D.at<float>(0,0)+0.5);
            int rowTmp = int(imgPoint2D.at<float>(1,0)+0.5);

            if(colTmp<0||colTmp>rgbImg.cols||rowTmp<0||rowTmp>rgbImg.rows)
            {
                continue;
            }

            cv::circle(rgbImg,cv::Point(colTmp,rowTmp),1,cv::Scalar(0,0,255));




        }
    }


}