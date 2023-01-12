#pragma once
#include <Eigen/Core>
#include <nodelet/nodelet.h>
#include <ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

namespace object_filter{

class CupFilterNodelet: public nodelet::Nodelet{
    public:
        virtual void onInit();
    protected:
        ros::NodeHandle nh_;
        ros::Publisher filteredPointcloudPublisher_;
        ros::Subscriber objectTransformSubscriber_;
        ros::Subscriber PointcloudSubscriber_;
        

        Eigen::Vector3d computePlane(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3);
        void computeConvexHull();       //get A_ and b_
        bool isCupDetected;
        Eigen::Matrix<double,6,3> A_;   //Ax - b < 0; mean the point cloud in the cup. We use 6 plane to wrap the cup.
        Eigen::Matrix<double,6,1> b_;

        Eigen::Matrix<double,6,3> corner_;  //corner in the cup frame
        
        Eigen::Matrix<double,4,4> camera_T_cup_;  //corner in the cup frame


        void subscribePointcloudCb(sensor_msgs::PointCloud2 msg);
        void subscribeCupPoseCb(geometry_msgs::PoseStamped msg);
};

}
