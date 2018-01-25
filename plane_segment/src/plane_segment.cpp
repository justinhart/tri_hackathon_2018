#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Segmenter {
private:
    tf::TransformListener tfListener;

public:
    Segmenter() : tfListener() {
    }

    void cloud_cb(const PointCloud::ConstPtr& cloud) {
        std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl;

        // Transform to map frame
        /*tf::StampedTransform transform;
        try {
            // Look up transform
            this->tfListener->lookupTransform("/map", cloud->header.frame_id, ros::Time(0), transform);
            pcl_ros::transformPointCloud(*cloud), (*cloud_plane_baselink), transform);
                cloud->header.frame_id = TARGET_FRAME;
                cloud_plane_baselink->header.frame_id = TARGET_FRAME;
                break;
            }
            //keep trying until we get the transform
            catch (tf::TransformException &ex){
                ROS_ERROR_THROTTLE(2,"%s",ex.what());
                ROS_WARN_THROTTLE(2, "   Waiting for tf to transform desired SAC axis to point cloud frame. trying again");
        }*/

        pcl::ModelCoefficients coefficients;
        pcl::PointIndices inliers;
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
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
        seg.segment(inliers, coefficients);

        if (inliers.indices.size() == 0) {
          PCL_ERROR ("Could not estimate a planar model for the given dataset.");
          return;
        }

        std::cerr << "Model coefficients: " << coefficients.values[0] << " " 
                                            << coefficients.values[1] << " "
                                            << coefficients.values[2] << " " 
                                            << coefficients.values[3] << std::endl;
      
        std::cerr << "Model inliers: " << inliers.indices.size () << std::endl;

    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "plane_segment");
    ros::NodeHandle node;

    Segmenter seg;
    ros::Subscriber sub = node.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &Segmenter::cloud_cb, &seg);

    ros::spin();

    return 0;
}
