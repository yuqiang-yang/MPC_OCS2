#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <Eigen/Core>
#include <random>
#include <vector>
#include <sensor_msgs/PointCloud2.h>

namespace graceful_mpc{
    class ClutteredEnvGenerator{
        public:
            ClutteredEnvGenerator()=delete;
            ClutteredEnvGenerator(ros::NodeHandle nh);
            ~ClutteredEnvGenerator()=default;
            void generateStaticMap();
            void updateDynamicMap();

            void publishMap();
        private:
            ros::NodeHandle nh_;
            double t_start_;

            double x_min_;
            double x_max_;
            double y_min_;
            double y_max_;
            double z_min_;
            double z_max_;

            int seed_;
            double clearance_at_origin_;

            int static_cylinder_num_;
            double static_cylinder_min_r_;
            double static_cylinder_max_r_;
            double static_cylinder_min_h_;
            double static_cylinder_max_h_;

            int static_cuboid_num_;
            double static_cuboid_min_edge_length_;
            double static_cuboid_max_edge_length_;
            double static_cuboid_min_h_;    //   0 < z < h, from ground
            double static_cuboid_max_h_;
        
            int dynamic_sphere_num_;
            double dynamic_sphere_min_r_;
            double dynamic_sphere_max_r_;
            double dynamic_sphere_min_z_;   //   z_min<z<z_max, floating
            double dynamic_sphere_max_z_;
            double dynamic_sphere_min_motion_;
            double dynamic_sphere_max_motion_;
            double dynamic_sphere_peroid_;
 
            double resolution_;
            ros::Publisher global_map_publisher_;
            ros::Publisher local_map_publisher_;
            sensor_msgs::PointCloud2 global_pcd;

            pcl::PointCloud<pcl::PointXYZ> cloudMap_;
            pcl::PointCloud<pcl::PointXYZ> dynamicMap_;
            pcl::PointCloud<pcl::PointXYZ> fullMap_;

            
            std::vector<std::vector<double>> dynamic_center_;
            std::vector<double> dynamic_theta_;
            std::vector<double> dynamic_radius_;

            std::vector<double> dynamic_motion_length_;

            // This funcition take the center (x,y), radius(r), height(in index) of a plane circle and push back all point to the cloudMap
            // It will be used to generate the circle and the cylinder
            // The z is used to translate the plane vertically. 
            void getPlaneCirclePoint(double x, double y,double z,double r, int h,pcl::PointCloud<pcl::PointXYZ>& cloudMap);
            

    }; //class ClutteredEnvGenerator
}; //namespace graceful_mpc