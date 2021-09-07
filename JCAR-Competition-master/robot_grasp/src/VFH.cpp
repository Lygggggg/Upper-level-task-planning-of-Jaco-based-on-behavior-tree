#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <sensor_msgs/PointCloud2.h>
#include <robot_grasp/GetVfh.h>
#include <robot_grasp/GetNormals.h>
#include <pcl/point_types.h>
std::vector<float> vfhdata(308,0);
class FeatureExtractor
{
public:
  explicit FeatureExtractor(ros::NodeHandle nh)
    : nh_(nh)
  {
    cluster_in_sub_ = nh_.subscribe("cluster_in", 1, &FeatureExtractor::clusterCallback, this);
    normals_out_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("normals_out", 1);
    get_normals_srv_ = nh_.advertiseService("get_normals", &FeatureExtractor::getNormalsReq, this);
    get_vfh_srv_ = nh_.advertiseService("get_vfh", &FeatureExtractor::getVFHReq, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cluster_in_sub_;
  ros::Publisher normals_out_pub_;
  ros::ServiceServer get_normals_srv_,get_vfh_srv_;

  void clusterCallback(const sensor_msgs::PointCloud2& cloud_msg)
  {
    ROS_INFO("Cluster Received");

    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(cloud_msg, *p_cloud);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sp_pcl_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    ne.setRadiusSearch(0.03);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    ROS_INFO("Done!");

    sensor_msgs::PointCloud2 normals_out_msg;
    pcl::toROSMsg(*cloud_normals, normals_out_msg);

    normals_out_pub_.publish(normals_out_msg);
  }

  bool getVFHReq(robot_grasp::GetVfh::Request &req, robot_grasp::GetVfh::Response &rsp)
  {
    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(req.cluster, *p_cloud);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sp_pcl_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch(0.03);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    ne.compute(*cloud_normals);

    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud(sp_pcl_cloud);
    vfh.setInputNormals(cloud_normals);
    vfh.setSearchMethod (tree);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
    vfh.compute (*vfhs);

    for(size_t i = 0; i < 308; i++)
    {
      vfhdata[i] = (vfhs->points[0]).histogram[i];
    }

    (rsp.VFH_data) = vfhdata;
    return true;
  }
  bool getNormalsReq(robot_grasp::GetNormals::Request &req, robot_grasp::GetNormals::Response &rsp)
  {
    rsp.cluster = req.cluster;

    pcl::PointCloud<pcl::PointXYZ> *p_cloud = new pcl::PointCloud<pcl::PointXYZ>();
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > sp_pcl_cloud(p_cloud);
    pcl::fromROSMsg(req.cluster, *p_cloud);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (sp_pcl_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    ne.setRadiusSearch(0.03);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    ne.compute(*cloud_normals);

    pcl::toROSMsg(*cloud_normals, rsp.cluster);

    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feature_extractor");
  ros::NodeHandle nh("~");

  FeatureExtractor nfe(nh);

  while (ros::ok())
    ros::spin();

  return 0;
}
