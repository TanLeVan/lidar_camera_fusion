#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include<Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <iostream>
#include <boost/bind.hpp>
#include <cmath>

class CameraLidarFusion : public rclcpp::Node
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> MySyncPolicy;
typedef pcl::PointCloud<pcl::PointXYZI> pclPointCloud;

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pcOnImage_pub;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> p_sync;
    float lidar_maxlen{100.};
    float lidar_minlen{0.01};

    size_t count;

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& in_image, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_pc2){
        // std::cout << std::to_string(count++) <<"both messge recieved" << std::endl;
        // std::cout << "do sthg with message" << std::endl;
    
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(in_image, in_image->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
          std::cout << "cv_bridge exception: %s" << e.what();
          return;
        }

        //Conversion of sensor_msgs::msg::PontCloud2 to pcl::PointCloud<T>
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*in_pc2,pcl_pc2);
        pclPointCloud::Ptr msg_pointCloud(new pclPointCloud);
        pcl::fromPCLPointCloud2(pcl_pc2,*msg_pointCloud);
        //Filtering NaN value (empty-value point) from point cloud
        if (msg_pointCloud == NULL) 
            {return;}
        pclPointCloud::Ptr pclCloud (new pclPointCloud);
        std::vector<int> indices; //vector to contain the indice of empty point in point cloud
        pcl::removeNaNFromPointCloud(*msg_pointCloud, *pclCloud, indices);

        //////////////////Performing coordinate transformation for each point in pointcloud
        int size_inter_lidar = (int) pclCloud->points.size();
        ////Specifying neccesary transformation matrix
        Eigen::MatrixXf Camera_instrinsic(3,4);
        Camera_instrinsic << 1.4534965770638237e+03, 0., 640., 0., 
                             0., 1.4534965770638237e+03, 480., 0.,
                             0., 0., 1., 0.;
        Eigen::MatrixXf Lidar_to_camera_3D(4,4); //matrix transforming 3d point from pointcloud coordinate
                                                //to camera coordinate
        Lidar_to_camera_3D << 0., 0., 1., 0.,
                              -1., 0., 0., 0.,
                              0., 0., -1., 0., 
                              0., 0., 0., 1.;

        Eigen::MatrixXf Lidar_cam(3,1); //Matrix to store the point in pointcloud in 
                                        //pixel coordinate
        Eigen::MatrixXf pc_matrix(4,1); //matrix to temporaryly hold the point in point cloud
                                        // in the homogenous form  [x y z 1]
        Eigen::MatrixXf pointCloud_matrix(4,size_inter_lidar);
        unsigned int cols = in_image->width; //col size of input image from camera
        unsigned int rows = in_image->height; //row size of input image from camera
        unsigned int px_data=0; unsigned int py_data=0; //variable to store x,y pixel
                                                        // coordinate of a point in point cloud
        for (int i =0; i<size_inter_lidar; i++){
            pc_matrix(0,0) = pclCloud->points[i].x;
            pc_matrix(1,0) = pclCloud->points[i].y;
            pc_matrix(2,0) = pclCloud->points[i].z;
            pc_matrix(3,0) = 1.0;
            //transformation of pointcloud
            Lidar_cam = Camera_instrinsic * Lidar_to_camera_3D*pc_matrix;
            px_data = (int)(Lidar_cam(0,0)/Lidar_cam(2,0));
            py_data = (int)(Lidar_cam(1,0)/Lidar_cam(2,0));
            std::cout <<Lidar_cam<< "\n";
            //if the point lies outside of FOV of camera, then continue
            if(px_data<0.0 || px_data>=cols || py_data<0.0 || py_data>=rows)
                {continue;}
 
            int color_dis = (int)(255*sqrt(pow(pclCloud->points[i].x,2)+pow(pclCloud->points[i].y,2)
                                    +pow(pclCloud->points[i].y,2))/lidar_maxlen);
            cv::circle(cv_ptr->image, cv::Point(px_data, py_data), 1, CV_RGB(255-color_dis,0,color_dis),cv::FILLED);
        }
        sensor_msgs::msg::Image pcl_on_img;
        cv_ptr->toImageMsg(pcl_on_img);
        pcOnImage_pub->publish(pcl_on_img);
    }

   
public: 
    CameraLidarFusion() 
    : Node("camera_lidar_fusion_node")
    {
        // declare parameter which is used to specify the topic from which image and pointcloud
        // message will be received
        this->declare_parameter("image_topic", "/front_image");
        this->declare_parameter("pointcloud_topic", "/livox/lidar");

        pcOnImage_pub = this->create_publisher<sensor_msgs::msg::Image>("/front_image_with_pc", 1);
        image_sub.subscribe(this, this->get_parameter("image_topic").as_string());
        pcl_sub.subscribe(this, this->get_parameter("pointcloud_topic").as_string());
        p_sync = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(3), image_sub, pcl_sub);
        p_sync->registerCallback(boost::bind(&CameraLidarFusion::callback,this, _1,_2));
    }

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraLidarFusion>());
    rclcpp::shutdown();

    return 0;
}