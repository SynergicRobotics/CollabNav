/*
 * slam_gmapping
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/player.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"
#include "gmapping/sensor/sensor_base/sensor.h"

#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

/*
 * ### Summary
 * 
 * The main function of this class is laserCallback(). At first scan, initMapper() 
 * is going to be called to initialize GMapping::GridSlamProcessor* gsp_ which 
 * is the main guy of our algorithm. Then, addScan is going to be called to 
 * process a scan. After that, updateMap() is called periodically to update the
 * current map and also publish "map" and "map_metadata"
 * 
 * Apart from that, this class publishes "entropy" of weight of all particles, so
 * we can observe the diveristy of weight.
 *
 *
 * ## Publisher
 *    -'entropy_publisher_' publishes "entropy"
 *    -'sst_'               publishes "map"
 *    -'sstm_'              publishes "map_metadata"
 *
 * ## Service Server
 *    -'ss_'                subscribes "dynamic_map", call mapCallback() and 
 *                          return nav_msgs::GetMap::Response map_, the map
 *
 * ## Subscriber
 *    -'scan_filter_sub_'   subscribes "sensor_msgs::LaserScan"
 *
 * ## TF related    //TODO what are they?
 *    -tf::TransformListener tf_
 *    -tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_
 *    -tf::TransformBroadcaster* tfB_
 * 
 * 
 * ## Public Function Overview
 * #  SlamGMapping() - setup
 *      -initialize OpenSLAM GMapping stuff
 *      -set parameters/threshold
 *      -initial ROS publisher/service/subscriber
 *      -set callback functions
 * #  laserCallback() - process a scan
 *      -it's a callback function for sensor_msgs::LaserScan
 *      -call initMapper() if its the first scan
 *      -call addScan() to process the scan
 *      -call updateMap() to update the current map
 * #  mapCallback() - reply the current map
 *      -respond nav_msgs::GetMap::Response map_ when "dynamic_map" is received
 * #  publishLoop() - loop for broadcasting tf forever 
 *      -boost::thread* transform_thread_ is dedicate for this function
 *      -call publishTransform() forever
 * #  publishTransform()
 *      -publish a map_to_odom transformation
 * 
 * 
 * ## Private Function Overview
 * #  initMapper() - fully init GMapping::GridSlamProcessor* gsp_
 *      -get laser pose (I understand it as the orientation of laser)
 *      -init GMapping::RangeSensor gsp_laser_
 *      -init GMapping::SensorMap smap
 *      -init GMapping::OdometrySensor gsp_odom_
 *      -get initial pose of the robot
 *      -further init GMapping::GridSlamProcessor* gsp_
 * #  addScan() - process a scan
 *      -call getOdomPose() // get robot pose at scan time
 *      -pre-processing scan point (remove outliner)
 *      -init GMapping::RangeReading reading
 *      -reading.setPose(currentRobotPose)
 *      -gsp_->processScan(reading)
 * #  updateMap() - update current map
 *      -init GMapping::ScanMatcher matcher
 *      -get best particle GMapping::GridSlamProcessor::Particle best
 *      -call computePoseEntropy()
 *      -long code of matching new scan data
 *      -expand map if needed
 *      -sst_ publishes "map"
 *      -sstm_ publishes "map_metadata"
 * #  getOdomPose() - get robot pose at time t
 *      get robot pose 'gmap_pose' at time t and return true
 * #  computePoseEntropy() - calculate diversity of weight
 *      calculate entropy of weight of all particles
 * 
 * 
 * ## OpenSLAM
 * This class is a ROS wrapper of real gmapping which is based on OpenSLAM.
 * Anything from "GMapping" packages is from OpenSLAM which are
 *  -GMapping::GridSlamProcessor* gsp_;
 *  -GMapping::RangeSensor* gsp_laser_;
 *  -GMapping::OdometrySensor* gsp_odom_;
 *  -GMapping::ScanMatcher matcher;
 *  -GMapping::ScanMatcherMap smap;
 *  -GMapping::OrientedPoint              data structure for point with heading
 *  -GMapping::SensorMap
 *  -GMapping::sampleGaussian
 *  -GMapping::RangeReading
 *  //TODO read OpenSLAM
 *    -"gmapping/gridfastslam/gridslamprocessor.h"
 *    -"gmapping/sensor/sensor_base/sensor.h"
 *    -"gmapping/sensor/sensor_range/rangesensor.h"
 *    -"gmapping/sensor/sensor_odometry/odometrysensor.h"
 * 
 * 
 * ## Future Works
 *  -callback function for camera image message to detect the the other robot
 *  -rendezvous
 *    -after detect, we need to calculate other robot's pose and 'jump'
 *    -function to exchange and process the observation; maybe define another 
 *     type of message and another callback function
 *    -after process all observation, we need to 'teleport' particles back
 *  -we have to discuss where to put these changes; here or OpenSLAM code.
 * 
 */

using namespace std;
    
class Record{
  public:
    Record(GMapping::OrientedPoint odom_pose, sensor_msgs::LaserScan measurement) {
	odom_pose_ = odom_pose;
	measurement_ = measurement;
    }
    sensor_msgs::LaserScan measurement_;
    GMapping::OrientedPoint odom_pose_;
};    


class SlamGMapping
{
  public:
    SlamGMapping();
    ~SlamGMapping();

    void publishTransform();
  
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);
    void publishLoop(double transform_publish_period);
    void virtualLaserCallback(const Record &teammate_record);
    
  private:
    // ROS stuff
    ros::NodeHandle node_;
    ros::Publisher entropy_publisher_;
    ros::Publisher sst_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;
    tf::TransformListener tf_;
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformBroadcaster* tfB_;

    // OpenSLAM GMapping stuff
    GMapping::GridSlamProcessor* gsp_;
    GMapping::RangeSensor* gsp_laser_;
    GMapping::OdometrySensor* gsp_odom_;
    double gsp_laser_angle_increment_;
    unsigned int gsp_laser_beam_count_;

    bool got_first_scan_;   // becomes true after received the first scan and
                            // mapper becomes fully initialized

    bool got_map_;          // becomes true after updateMap() is called once
    nav_msgs::GetMap::Response map_;

    ros::Duration map_update_interval_; // update map every this duration
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex map_mutex_;

    int laser_count_;           // count the received scan
    int throttle_scans_;        // can be set to skip some scans

    boost::thread* transform_thread_; // call publishLoop()

    std::string base_frame_;    // init as string of "base_link"
    std::string laser_frame_;   // never used lol
    std::string map_frame_;     // init as string of "map"
    std::string odom_frame_;    // init as string of "odom"

    void updateMap(const sensor_msgs::LaserScan& scan);
    bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const ros::Time& t);
    bool initMapper(const sensor_msgs::LaserScan& scan);
    bool addScan(const sensor_msgs::LaserScan& scan, const GMapping::OrientedPoint& gmap_pose);
    double computePoseEntropy();
    
    // Parameters used by GMapping
    double maxRange_;
    double maxUrange_;
    double maxrange_;
    double sigma_;
    int kernelSize_;
    double lstep_;
    double astep_;
    int iterations_;
    double lsigma_;
    double ogain_;
    int lskip_;
    double srr_;
    double srt_;
    double str_;
    double stt_;
    double linearUpdate_;
    double angularUpdate_;
    double temporalUpdate_;
    double resampleThreshold_;
    int particles_;
    double xmin_;
    double ymin_;
    double xmax_;
    double ymax_;
    double delta_;
    double occ_thresh_;
    double llsamplerange_;
    double llsamplestep_;
    double lasamplerange_;
    double lasamplestep_;
    
    // CollabNav    
    vector<Record> records_;
};
