#include "ClutteredEnvGenerator.hpp"
using namespace graceful_mpc;

ClutteredEnvGenerator::ClutteredEnvGenerator(ros::NodeHandle nh):nh_(nh){
    global_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("global_pointcloud", 1);
    local_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("local_pointcloud", 1);
    nh_.param("x_min", x_min_, -10.0);
    nh_.param("x_max", x_max_, 10.0);
    nh_.param("y_min", y_min_, -10.0);
    nh_.param("y_max", y_max_, 10.0);
    nh_.param("z_min", z_min_, 1.0);
    nh_.param("z_max", z_max_, 3.0);

    nh_.param("clearance_at_origin", clearance_at_origin_, 0.5);
    nh_.param("resolution", resolution_, 0.05);


    nh_.param("static_cylinder_num", static_cylinder_num_, 10);
    nh_.param("static_cylinder_min_r", static_cylinder_min_r_, 0.1);
    nh_.param("static_cylinder_max_r", static_cylinder_max_r_, 0.3);
    nh_.param("static_cylinder_min_h", static_cylinder_min_h_, 0.1);
    nh_.param("static_cylinder_max_h", static_cylinder_max_h_, 0.3);
    
  
    nh_.param("static_cuboid_num", static_cuboid_num_, 10);
    nh_.param("static_cuboid_min_edge_length", static_cuboid_min_edge_length_, 0.1);
    nh_.param("static_cuboid_max_edge_length", static_cuboid_max_edge_length_, 0.3);
    nh_.param("static_cuboid_min_h", static_cuboid_min_h_, 0.1);
    nh_.param("static_cuboid_max_h", static_cuboid_max_h_, 0.3);


    nh_.param("dynamic_sphere_num", dynamic_sphere_num_, 10);
    nh_.param("dynamic_sphere_min_r", dynamic_sphere_min_r_, 0.2);
    nh_.param("dynamic_sphere_max_r", dynamic_sphere_max_r_, 0.5);
    nh_.param("dynamic_sphere_min_z", dynamic_sphere_min_z_, 0.8);
    nh_.param("dynamic_sphere_max_z", dynamic_sphere_max_z_, 2.0);
    nh_.param("dynamic_sphere_min_motion", dynamic_sphere_min_motion_, 1.0);
    nh_.param("dynamic_sphere_max_motion", dynamic_sphere_max_motion_, 2.0);
    nh_.param("dynamic_sphere_peroid", dynamic_sphere_peroid_, 5.0);

    nh_.param("seed", seed_, 1);

    dynamic_center_.resize(dynamic_sphere_num_);
    dynamic_theta_.resize(dynamic_sphere_num_);
    dynamic_radius_.resize(dynamic_sphere_num_);
    dynamic_motion_length_.resize(dynamic_sphere_num_);

    t_start_ = ros::Time::now().toSec();
}

void ClutteredEnvGenerator::generateStaticMap(){
    ROS_INFO("generate Cluttered Map");
    pcl::PointXYZ pt_random;    //temp variable
    std::default_random_engine eng;
    eng.seed(seed_);
    std::uniform_real_distribution<double> rand_x(x_min_,x_max_);
    std::uniform_real_distribution<double> rand_y(y_min_,y_max_);
    std::uniform_real_distribution<double> rand_z(z_min_,z_max_);
    std::uniform_real_distribution<double> rand_theta(0,6.28);


    std::uniform_real_distribution<double> rand_r(static_cylinder_min_r_,static_cylinder_max_r_);
    std::uniform_real_distribution<double> rand_h(static_cylinder_min_h_,static_cylinder_max_h_);
    using std::pow;
    using std::sqrt;
    // cyboid
    for(int i = 0; i < static_cuboid_num_; i++){
        double x, y,r, h;
        x = rand_x(eng);
        y = rand_y(eng);
        r = rand_r(eng);
        h = rand_h(eng);

        if (sqrt(pow(x, 2) + pow(y, 2)) < clearance_at_origin_) {
            i--;
            continue;
        }
        x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
        y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
        // ROS_INFO_STREAM(" x:" << x << " y:" << y<<" r:" << r);
        int rNum = ceil(r / resolution_);

        for (int r = -rNum / 2.0; r < rNum / 2.0; r++)
        for (int s = -rNum / 2.0; s < rNum / 2.0; s++) {
            int hNum = ceil(h / resolution_);
            for (int t = 0; t < hNum; t++) {
            pt_random.x = x + (r + 0.5) * resolution_;
            pt_random.y = y + (s + 0.5) * resolution_;
            pt_random.z = (t + 0.5) * resolution_;
            cloudMap_.points.push_back(pt_random);
            }
        }
    }

    //cylinder
    rand_r = std::uniform_real_distribution<double>(static_cylinder_min_r_,static_cylinder_max_r_);
    rand_h = std::uniform_real_distribution<double>(static_cylinder_min_h_,static_cylinder_max_h_);
    for(int i = 0; i < static_cylinder_num_; i++){
        double x, y,r, h;
        x = rand_x(eng);
        y = rand_y(eng);
        r = rand_r(eng);
        h = rand_h(eng);
        if (sqrt(pow(x, 2) + pow(y, 2)) < clearance_at_origin_) {
            i--;
            continue;
        }
       
        x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
        y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
        r = floor(r / resolution_) * resolution_ + resolution_ / 2.0; 
        
        int hNum = floor(h / resolution_);    // height num for a cylinder
        int cNum = floor(r / resolution_);    //column num for a plane circle
        for(int hh = 0;hh < hNum;hh++){
            getPlaneCirclePoint(x,y,0,r,hh,cloudMap_);
        }
    }

    rand_r = std::uniform_real_distribution<double>(dynamic_sphere_min_r_,dynamic_sphere_max_r_);
    rand_h = std::uniform_real_distribution<double>(dynamic_sphere_min_z_,dynamic_sphere_max_z_);
    // store the dynamic sphere information
    std::uniform_real_distribution<double> rand_l(dynamic_sphere_min_motion_,dynamic_sphere_max_motion_);
    
    for(int i = 0; i < dynamic_sphere_num_; i++){
        double x, y,r, h,theta,l;
        x = rand_x(eng);
        y = rand_y(eng);
        r = rand_r(eng);
        h = rand_h(eng);   
        theta = rand_theta(eng);
        l = rand_l(eng);
        if (sqrt(pow(x, 2) + pow(y, 2)) < clearance_at_origin_) {
            i--;
            continue;
        }
     
        x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
        y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
        r = floor(r / resolution_) * resolution_ + resolution_ / 2.0; 
        h = floor(h / resolution_) * resolution_ + resolution_ / 2.0; //The h is also the z coordinate of the sphere
        std::vector<double> temp(3);
        temp[0] = x; temp[1] = y; temp[2] = h;
        dynamic_center_[i] = temp;
        dynamic_theta_[i] = theta;
        dynamic_radius_[i] = r;
        dynamic_motion_length_[i] = l;

    }    
    // dynamic sphere The following code can generate static floatting sphere

    // for(int i = 0; i < dynamic_sphere_num_; i++){
    //     double x, y,r, h;
    //     x = rand_x(eng);
    //     y = rand_y(eng);
    //     r = rand_r(eng);
    //     h = rand_h(eng);   
    //     if (sqrt(pow(x, 2) + pow(y, 2)) < clearance_at_origin_) {
    //         i--;
    //         continue;
    //     }
     
    //     x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
    //     y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
    //     r = floor(r / resolution_) * resolution_ + resolution_ / 2.0; 
    //     h = floor(h / resolution_) * resolution_ + resolution_ / 2.0; //The h is also the z coordinate of the sphere
        
    //     int zzNum = floor(r/resolution_);  //cut the slice
    //     for (int zz = -zzNum; zz <= zzNum ; zz++){
    //         double rr = sqrt(r*r-pow(zz*resolution_,2));
    //         getPlaneCirclePoint(x,y,h,rr,zz,cloudMap_);

    //     }

    // }

    cloudMap_.width = cloudMap_.points.size();
    cloudMap_.height = 1;
    cloudMap_.is_dense = true;
}
void ClutteredEnvGenerator::updateDynamicMap(){
    dynamicMap_.clear();
    assert(dynamic_center_.size() == dynamic_sphere_num_ && dynamic_motion_length_.size() == dynamic_sphere_num_ && dynamic_theta_.size() == dynamic_sphere_num_); 
    for(int i = 0; i < dynamic_sphere_num_; i++){
        double x, y,r, h;
        double t = ros::Time::now().toSec() - t_start_;
        x = dynamic_center_[i][0] + dynamic_motion_length_[i] * cos(dynamic_theta_[i]) * sin(t*6.28/dynamic_sphere_peroid_);
        y = dynamic_center_[i][1] + dynamic_motion_length_[i] * sin(dynamic_theta_[i]) * sin(t*6.28/dynamic_sphere_peroid_);
        r = dynamic_radius_[i];
        h = dynamic_center_[i][2];   
    
        x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
        y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;

        int zzNum = floor(r/resolution_);  //cut the slice
        for (int zz = -zzNum; zz <= zzNum ; zz++){
            double rr = sqrt(r*r-pow(zz*resolution_,2));
            getPlaneCirclePoint(x,y,h,rr,zz,dynamicMap_);
        }
    }
}


void ClutteredEnvGenerator::publishMap(){
    fullMap_.clear();
    fullMap_ = cloudMap_;
    fullMap_ += dynamicMap_;
    pcl::toROSMsg(fullMap_, global_pcd);
    global_pcd.header.frame_id = "odom";
    global_pcd.header.stamp = ros::Time::now();
    global_map_publisher_.publish(global_pcd);

}

void ClutteredEnvGenerator::getPlaneCirclePoint(double x,double y,double z,double r,int h,pcl::PointCloud<pcl::PointXYZ>& cloudMap){
    pcl::PointXYZ pt_random;    //temp variable
    int cNum = floor(r / resolution_);    //column num for a plane circle

    for (int cc = -cNum; cc <= cNum ; cc++){
    int rNum = floor((sqrt(r*r-pow(cc*resolution_,2)))/resolution_);
    for (int rr = -rNum; rr <= rNum ; rr++){
            pt_random.x = x + (rr+0.5) * resolution_;
            pt_random.y = y + (cc+0.5) * resolution_;
            pt_random.z = z + (h+0.5) * resolution_;
            cloudMap.points.push_back(pt_random);
        }
}
}
