#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Segmenter {
private:
    tf::TransformListener tfListener;
    ros::Publisher cloud_pub;

public:
    Segmenter(ros::NodeHandle node) : tfListener() {
        this->cloud_pub = node.advertise<PointCloudT>("plane_segment", 1);
    }

    void cloud_cb(const PointCloudT::ConstPtr& cloudOrig) {
        std::cerr << "Point cloud data: " << cloudOrig->points.size() << " points" << std::endl;

        // Transform to map frame
        PointCloudT::Ptr cloud(new PointCloudT);
        tf::StampedTransform transform;
        try {
            // Look up transform
            this->tfListener.waitForTransform("map", cloudOrig->header.frame_id, ros::Time(0), ros::Duration(0.1));
            this->tfListener.lookupTransform("map", cloudOrig->header.frame_id, ros::Time(0), transform);
            pcl_ros::transformPointCloud(*cloudOrig, *cloud, transform);
            cloud->header.frame_id = "map";
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setMaxIterations(1000);

        Eigen::Vector3f axis(0., 0., 1.);
        seg.setAxis(axis);
        seg.setEpsAngle(3. * M_PI / 180.);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, coefficients);

        if (inliers->indices.size() == 0) {
          PCL_ERROR ("Could not estimate a planar model for the given dataset.");
          return;
        }

        pcl::ExtractIndices<PointT> extract;

        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud);

        std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
                                            << coefficients.values[1] << " "
                                            << coefficients.values[2] << " " 
                                            << coefficients.values[3] << std::endl;

        std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

        this->cloud_pub.publish(cloud);

    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "plane_segment");
    ros::NodeHandle node;

    Segmenter seg(node);
    ros::Subscriber sub = node.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &Segmenter::cloud_cb, &seg);

    ros::spin();

    return 0;
}
