/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "velodyne_pointcloud/convert.h"
#include <velodyne_driver/time_conversion.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);

    private_nh.param<bool>("pub_sync_time_topic",pub_sync_time_topic_,true);
    private_nh.param<bool>("pub_ros_time_topic",pub_ros_time_topic_,true);


    // advertise output point cloud (before subscribing to input data)
    if(pub_sync_time_topic_ == true)
    {
      output_sync = node.advertise<sensor_msgs::PointCloud2>("velodyne_points_sync", 10);
    }
    if(pub_ros_time_topic_ == true)
    {
      output_ros = node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
    }

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 10,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));

    // Diagnostics
    diagnostics_.setHardwareID("Velodyne Convert");
    // Arbitrary frequencies since we don't know which RPM is used, and are only
    // concerned about monitoring the frequency.
    diag_min_freq_ = 2.0;
    diag_max_freq_ = 20.0;
    using namespace diagnostic_updater;
    diag_topic_.reset(new TopicDiagnostic("velodyne_points", diagnostics_,
                                       FrequencyStatusParam(&diag_min_freq_,
                                                            &diag_max_freq_,
                                                            0.1, 10),
                                       TimeStampStatusParam()));
  }

  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if ((output_ros.getNumSubscribers() == 0) && (output_sync.getNumSubscribers() == 0))         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud with same time and frame ID as raw data
    PointcloudXYZIR outMsgRos, outMsgSync;
    // outMsgRos's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsgRos.pc->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;

    outMsgSync.pc->header.stamp = pcl_conversions::toPCL(rosTimeFromGpsTimestamp(&(scanMsg->packets.back().data[1200])));

    outMsgSync.pc->header.frame_id = outMsgRos.pc->header.frame_id = scanMsg->header.frame_id;
    outMsgSync.pc->height = outMsgRos.pc->height = 1;

    outMsgRos.pc->points.reserve(scanMsg->packets.size() * data_->scansPerPacket());
    outMsgSync.pc->points.reserve(scanMsg->packets.size() * data_->scansPerPacket());

    // process each packet provided by the driver
    for (size_t i = 0; i < scanMsg->packets.size(); ++i)
    {
      data_->unpack(scanMsg->packets[i], outMsgRos);
      data_->unpack(scanMsg->packets[i], outMsgSync);
    }

    // publish the accumulated cloud message
    if(pub_ros_time_topic_ == true)
    {
      ROS_DEBUG_STREAM("Publishing " << outMsgRos.pc->height * outMsgRos.pc->width
                     << " Velodyne points, time: " << outMsgRos.pc->header.stamp);
      output_ros.publish(outMsgRos.pc);
    }

    if(pub_sync_time_topic_ == true)
    {
      ROS_DEBUG_STREAM("Publishing " << outMsgSync.pc->height * outMsgSync.pc->width
                     << " Velodyne points, synctime: " << outMsgSync.pc->header.stamp);
      output_sync.publish(outMsgSync.pc);
    }

    //ROS_ERROR("time sync before ros: %f", scanMsg->header.stamp.toSec()- rosTimeFromGpsTimestamp(&(scanMsg->packets.back().data[1200])).toSec());


    diag_topic_->tick(scanMsg->header.stamp);
    diagnostics_.update();
  }

} // namespace velodyne_pointcloud
