#include <ros/ros.h>

#include <tf/transform_listener.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "geometry_msgs/Point32.h"
#include "std_srvs/Empty.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <laser_geometry/laser_geometry.h>
#include "sensor_msgs/point_cloud_conversion.h"

#include <time.h>
#include <assert.h>

#include "map/map.h"

#include <iostream>
#include <string>


class DiffElevationMappingNode
{
  public:
    DiffElevationMappingNode();
    ~DiffElevationMappingNode();

    void requestMap();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    bool pubmapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
    bool savemapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
  private:
    void processMapping(const sensor_msgs::PointCloud2ConstPtr& point_cloud);

    ros::Subscriber point_cloud_sub;

    ros::Publisher map_cloud_pub;
    ros::Publisher map_msg_pub;

    ros::ServiceServer pub_server_;
    ros::ServiceServer save_server_;

    tf::TransformListener tf_;

    std::string global_frame_id_, point_cloud_topic_name_, pcd_file_;
    laser_geometry::LaserProjection projector_;

    map_t* map_;
};

DiffElevationMappingNode::DiffElevationMappingNode()
{
  ros::NodeHandle private_nh;
  // private_nh.param("point_cloud_topic_name", point_cloud_topic_name_, std::string("/hokuyo3d/hokuyo_cloud2"));
  private_nh.param("point_cloud_topic_name", point_cloud_topic_name_, std::string("/disting_cloud2"));
  private_nh.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh.param("pcd_file", pcd_file_, std::string("elevation_difference_grid_map.pcd"));

  ros::NodeHandle nh;
  point_cloud_sub = nh.subscribe(point_cloud_topic_name_, 100, &DiffElevationMappingNode::processMapping, this);
  map_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("map_cloud", 2, true);
  map_msg_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 2, true);
  pub_server_ = nh.advertiseService("up_map", &DiffElevationMappingNode::pubmapCallback, this);
  save_server_ = nh.advertiseService("save_map", &DiffElevationMappingNode::savemapCallback, this);
  requestMap();
}

DiffElevationMappingNode::~DiffElevationMappingNode(){}

void DiffElevationMappingNode::processMapping(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
{
  sensor_msgs::PointCloud2 transformed_point_cloud;
  if(!(pcl_ros::transformPointCloud(global_frame_id_, *point_cloud, transformed_point_cloud, tf_))) return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_point_cloud, *pcl_point_cloud);

  map_t *obs_map = map_alloc();
  obs_map->size_x = map_->size_x;
  obs_map->size_y = map_->size_y;
  obs_map->scale = map_->scale;
  obs_map->origin_x = map_->origin_x;
  obs_map->origin_y = map_->origin_y;
  obs_map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*obs_map->size_x*obs_map->size_y);

  for(int i=0; i<obs_map->size_x*obs_map->size_y; i++){
    obs_map->cells[i].flag = false;
    obs_map->cells[i].min = 0.0;
    obs_map->cells[i].max = 0.0;
    obs_map->cells[i].diff = 0.0;
  }

  // ver tera start
  // tf::Stamped<tf::Pose> pose;
  // tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),tf::Vector3(0,0,0)), point_cloud->header.stamp, point_cloud->header.frame_id);
  // try
  // {
  //   this->tf_.transformPose(global_frame_id_, ident, pose);
  // }
  // catch(tf::TransformException e)
  // {
  //   ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
  //   return;
  // }
  // ver tera end

  for(int i=0; i<pcl_point_cloud->points.size(); i++){
      // double x =  pcl_point_cloud->points.at(i).x - pose.getOrigin().x(); //ver tera
      // double y =  pcl_point_cloud->points.at(i).y - pose.getOrigin().y(); //ver tera
      // double z =  pcl_point_cloud->points.at(i).z - pose.getOrigin().z(); //ver tera
      // if (pcl_point_cloud->points.at(i).z >1.5) pcl_point_cloud->points.at(i).z = 1.5;
      // if (pcl_point_cloud->points.at(i).z > 1.0) continue; //ver tera
      // if (10.0 <= sqrt(x*x + y*y + z*z)) continue; //ver tera
      // map_updata_cell(map_, pcl_point_cloud->points.at(i).x, pcl_point_cloud->points.at(i).y, pcl_point_cloud->points.at(i).z);
      map_updata_cell(obs_map, pcl_point_cloud->points.at(i).x, pcl_point_cloud->points.at(i).y, pcl_point_cloud->points.at(i).z);
  }

  map_updata_map(map_, obs_map);

  map_free(obs_map);
  obs_map = NULL;
}

bool DiffElevationMappingNode::savemapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  pcl::PointCloud<pcl::PointXYZI> pc_msg;
  pc_msg.clear();
  for(int i=0;i<map_->size_x;i++) {
    for(int j=0;j<map_->size_y;j++) {
      if (!(MAP_VALID(map_, i, j))) continue;
      if (!(map_->cells[MAP_INDEX(map_, i, j)].flag)) continue;
      // if (map_->cells[MAP_INDEX(map_, i, j)].diff < 0.05) continue;
        pcl::PointXYZI p_msg;
        p_msg.x = MAP_WXGX(map_, i);
        p_msg.y = MAP_WXGX(map_, j);
        // p_msg.z = map_->cells[MAP_INDEX(map_, i, j)].diff; // p_msg.intensity  = p_msg.z;
        // p_msg.intensity  = p_msg.z;
        if (map_->cells[MAP_INDEX(map_, i, j)].visit==0) {
          p_msg.z = 0.0;
        } else {
          p_msg.z = map_->cells[MAP_INDEX(map_, i, j)].diff / map_->cells[MAP_INDEX(map_, i, j)].visit;
        }
        for (int k = 0; k < map_->cells[MAP_INDEX(map_, i, j)].diffs.size(); k++) {
          double d = map_->cells[MAP_INDEX(map_, i, j)].diffs.at(k) - p_msg.z;
          p_msg.intensity += d * d;
        }
        if (!(map_->cells[MAP_INDEX(map_, i, j)].diffs.size()==0)) {
          p_msg.intensity = 0.0;
        } else {
          p_msg.intensity = p_msg.intensity / map_->cells[MAP_INDEX(map_, i, j)].diffs.size();
        }
        pc_msg.push_back(p_msg);
    } 
  }
  // pcl::io::savePCDFile("3f.pcd", pc_msg);
  // pcl::io::savePCDFile("gaisyuu_0901_1.pcd", pc_msg);
  // pcl::io::savePCDFile("3f_0901.pcd", pc_msg);
  // pcl::io::savePCDFile("gaisyuu_0907_01.pcd", pc_msg);
  // pcl::io::savePCDFile("gaisyuu_0901_01.pcd", pc_msg);
  // pcl::io::savePCDFile("cit_all_0915_01.pcd", pc_msg);
  // pcl::io::savePCDFile("gaisyuu_0927_01.pcd", pc_msg);
  // pcl::io::savePCDFile("delta_oshimizu_1013_01.pcd", pc_msg);
  // pcl::io::savePCDFile("/home/humio/delta_oshimizu_1026_02.pcd", pc_msg);
  pcl::io::savePCDFile(pcd_file_, pc_msg);
  ROS_INFO("save success");

  return true;
}

bool DiffElevationMappingNode::pubmapCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{

  map_t* fmap = map_alloc();
  ROS_ASSERT(fmap);
  fmap->size_x = map_->size_x;
  fmap->size_y = map_->size_y;
  fmap->scale = map_->scale;
  fmap->origin_x = map_->origin_x;
  fmap->origin_y = map_->origin_y;
  fmap->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*fmap->size_x*fmap->size_y);
  ROS_ASSERT(fmap->cells);

  // 3*3の場合
  for(int center_x=2; center_x+1<map_->size_x; center_x++){
    for(int center_y=2; center_y+1<map_->size_y; center_y++){
      std::vector<double> box;
      box.clear();
      for (int x=-1; x<=1; x++){
        for(int y=-1; y<=1; y++){
          if (!(MAP_VALID(map_, x+center_x, y+center_y))) continue;
          box.push_back(map_->cells[MAP_INDEX(map_, x+center_x, y+center_y)].diff);
        }
      }
      if (box.size() != 9) continue;
      std::sort(box.begin(), box.end());
      fmap->cells[MAP_INDEX(fmap, center_x, center_y)].diff=box.at(4);
    }
  }

  for(int i=0;i<map_->size_x;i++) {
      for(int j=0;j<map_->size_y;j++) {
        if (!(MAP_VALID(map_, i, j))) continue;
        map_->cells[MAP_INDEX(map_, i, j)].diff=fmap->cells[MAP_INDEX(fmap, i, j)].diff;
      } 
  }

  sensor_msgs::PointCloud map_cloud;
  sensor_msgs::ChannelFloat32 channel;
  channel.name = std::string("intensity");
  channel.values.shrink_to_fit();
  map_cloud.channels.push_back(channel);
  for(int i=0;i<map_->size_x;i++) {
      for(int j=0;j<map_->size_y;j++) {
          if (!(MAP_VALID(map_, i, j))) continue;
          if (fabs(map_->cells[MAP_INDEX(map_, i, j)].diff) < 0.1) continue;
          // ROS_INFO("test6");
          geometry_msgs::Point32 point;
          point.x = MAP_WXGX(map_, i);
          point.y = MAP_WXGX(map_, j);
          map_cloud.points.push_back(point);
          map_cloud.channels.at(map_cloud.channels.size()-1).values.push_back(map_->cells[MAP_INDEX(map_, i, j)].diff);
      }
  }

  sensor_msgs::PointCloud2 map_cloud2;
  convertPointCloudToPointCloud2(map_cloud, map_cloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_map_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  fromROSMsg(map_cloud2, *pcl_map_cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (pcl_map_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.1);
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  sor.filter (*map_cloud_filtered);

  nav_msgs::OccupancyGrid map_msg;
  map_msg.info.width = map_->size_x;
  map_msg.info.height = map_->size_y;
  map_msg.info.resolution = map_->scale;
  map_msg.info.origin.position.x = map_->origin_x - (map_->size_x / 2) * map_->scale;
  map_msg.info.origin.position.y = map_->origin_y - (map_->size_y / 2) * map_->scale;

  map_msg.data.resize(map_msg.info.width * map_msg.info.height);

  // for(int i=0;i<map_->size_x * map_->size_y;i++)
  // {
  //   map_->cells[i].diff = 0;
  // }

  // for (size_t k=0; k<map_cloud_filtered->points.size(); k++) {
  //   int i = MAP_GXWX(map_, map_cloud_filtered->points.at(k).x), j = MAP_GYWY(map_, map_cloud_filtered->points.at(k).y);
  //   map_->cells[MAP_INDEX(map_, i, j)].diff = 1.0;
  // }

  for(int i=0;i<map_->size_x * map_->size_y;i++)
  {
    // ROS_INFO_STREAM("test7: " << ((map_->size_x * map_->size_y) -i));
    if(!(map_->cells[i].flag))
      map_msg.data[i] = -1;
    else if(fabs(map_->cells[i].diff < 0.15))
      map_msg.data[i] = 0;
    else
      map_msg.data[i] = 100;
  }
  map_msg.header.frame_id = global_frame_id_;
  map_msg.header.stamp = ros::Time::now();
  map_msg_pub.publish(map_msg);

  // pcl::toROSMsg(*map_cloud_filtered, map_cloud2);
  // convertPointCloud2ToPointCloud(map_cloud2, map_cloud);

  // map_cloud.header.frame_id = global_frame_id_;
  // map_cloud.header.stamp = ros::Time::now();
  // map_cloud_pub.publish(map_cloud);

  return true;
}

map_t* DiffElevationMappingNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();

  // map->size_x = floor(map_msg.info.width / 2 + 0.5);
  // map->size_y = floor(map_msg.info.height / 2 + 0.5);
  // ROS_INFO("%d", map->size_x);
  // ROS_INFO("%d", map->size_y);
  // map->scale = map_msg.info.resolution * 2;
  // ROS_INFO("%f", map_msg.info.origin.position.x);
  // ROS_INFO("%f", map_msg.info.origin.position.y);
  // map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  // map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // ROS_INFO("%f", map->origin_x);
  // ROS_INFO("%f", map->origin_y);
  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);

  for(int i=0; i<map->size_x*map->size_y; i++){
    map->cells[i].flag = false;
    map->cells[i].min = 0.0;
    map->cells[i].max = 0.0;
    map->cells[i].diff = 0.0;
    map->cells[i].visit = 0;
    map->cells[i].diffs.clear();
  }

  return map;
}

void DiffElevationMappingNode::requestMap()
{
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  map_ = convertMap(resp.map);
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_elevation_mapping");
  DiffElevationMappingNode de;

  ros::spin();

  return 0;
}
