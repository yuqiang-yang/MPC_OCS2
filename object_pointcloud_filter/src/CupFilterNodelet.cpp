#include <CupFilterNodelet.hpp>
#include <pluginlib/class_list_macros.h>
#include <Eigen/Core>
#include <pcl/common/generate.h>
#include <pcl/point_types.h>
#include <pcl/filters/experimental/functor_filter.h>
using namespace object_filter;


void CupFilterNodelet::onInit()
{
    NODELET_DEBUG("Initializing CupFilterNodelet...");
    nh_ = this->getPrivateNodeHandle();
    filteredPointcloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("output",100);
    PointcloudSubscriber_ = nh_.subscribe("input", 1,&CupFilterNodelet::subscribePointcloudCb, this);
    objectTransformSubscriber_ = nh_.subscribe("cup_pose", 1,&CupFilterNodelet::subscribeCupPoseCb, this);


}

Eigen::Vector3d CupFilterNodelet::computePlane(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3)
{   

}
void CupFilterNodelet::computeConvexHull()
{
    if(!isCupDetected) return;
}
void CupFilterNodelet::subscribePointcloudCb(sensor_msgs::PointCloud2 msg)
{
    if(!isCupDetected)
    {
        filteredPointcloudPublisher_.publish(msg);
        return;
    }
    computeConvexHull();    //get A_ and b_


}
void CupFilterNodelet::subscribeCupPoseCb(geometry_msgs::PoseStamped msg)
{
    camera_T_cup_ = Eigen::Matrix4d::Identity();
    camera_T_cup_(3,0) = msg.pose.position.x;
    camera_T_cup_(3,1) = msg.pose.position.y;
    camera_T_cup_(3,2) = msg.pose.position.z;
    
    Eigen::Quaternion quat(msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z);
    camera_T_cup_.block<3, 3>(0, 0) = quat.toRotationMatrix();

    isCupDetected = true;
}


PLUGINLIB_EXPORT_CLASS(CupFilterNodelet, nodelet::Nodelet)
