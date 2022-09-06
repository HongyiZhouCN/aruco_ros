#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination;
int dctComponentsToRemove;
aruco::MarkerDetector mDetector;
std::vector<aruco::Marker> markers;
ros::Subscriber cam_info_sub;
bool cam_info_received;
image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;

ros::Publisher pose_pub;
// ros::Publisher pose_pub2;
// ros::Publisher pose_pub3;
// ros::Publisher pose_pub4;

std::string parent_name;
std::string child_name;

double marker_size;

cv::aruco::Board* arucoBoard = NULL;

std::vector<std::vector<cv::Point3f>> cornerPoints(4);
std::vector<int> ids(4);
cv::Ptr<cv::aruco::Dictionary> dictionary;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  double ticksBefore = cv::getTickCount();
  static tf::TransformBroadcaster br;
  if (cam_info_received)
  {
    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      if (normalizeImageIllumination)
      {
        ROS_WARN("normalizeImageIllumination is unimplemented!");
      }

      // detection results will go into "markers"
      // markers.clear();
      // ok, let's detect
      // mDetector.detect(inImage, markers, camParam, marker_size, false);
      // for each marker, draw info and its boundaries in the image

      auto arucoBoard = cv::aruco::Board::create(cornerPoints, dictionary, ids);

      std::vector<int> markerIds;
      std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
      cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
      cv::aruco::detectMarkers(inImage, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
      cv::Vec3d rvec, tvec;
      if(markerIds.size() > 0){
        int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, arucoBoard, camParam.CameraMatrix, camParam.Distorsion, rvec, tvec);
        if (valid > 0){
          cv::aruco::drawAxis(inImage, camParam.CameraMatrix, camParam.Distorsion, rvec, tvec, 0.1);

          cv::Mat rot_matrix;
          cv::Rodrigues(rvec, rot_matrix);
          tf::Matrix3x3 tf_rot(rot_matrix.at<double>(0,0),rot_matrix.at<double>(0,1),rot_matrix.at<double>(0,2),
                                rot_matrix.at<double>(1,0),rot_matrix.at<double>(1,1),rot_matrix.at<double>(1,2),
                                rot_matrix.at<double>(2,0),rot_matrix.at<double>(2,1),rot_matrix.at<double>(2,2));
          tf::Vector3 tf_trans(tvec[0], tvec[1], tvec[2]);
          tf::Transform transform = tf::Transform(tf_rot, tf_trans);

          br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name));
        }
      } 

    //   for (unsigned int i = 0; i < markers.size(); ++i)
    //   {
    //     // only publishing the selected marker
    //     if (markers[i].id == marker_id1)
    //     {
    //       tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
    //       br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name1));
    //       geometry_msgs::Pose poseMsg;
    //       tf::poseTFToMsg(transform, poseMsg);
    //       pose_pub1.publish(poseMsg);
    //     }New in Lunar markers[i]);
    //       br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name2));
    //       geometry_msgs::Pose poseMsg;
    //       tf::poseTFToMsg(transform, poseMsg);
    //       pose_pub2.publish(poseMsg);
    //     }
    //     else if (markers[i].id == marker_id3)
    //     {
    //       tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
    //       br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name3));
    //       geometry_msgs::Pose poseMsg;
    //       tf::poseTFToMsg(transform, poseMsg);
    //       pose_pub3.publish(poseMsg);
    //     }
    //     else if (markers[i].id == marker_id4)
    //     {
    //       tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
    //       br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, child_name4));
    //       geometry_msgs::Pose poseMsg;
    //       tf::poseTFToMsg(transform, poseMsg);
    //       pose_pub4.publish(poseMsg);
    //     }

    //     // but drawing all the detected markers
    //     markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
    //   }

    //   // paint a circle in the center of the image
    //   cv::circle(inImage, cv::Point(inImage.cols / 2, inImage.rows / 2), 4, cv::Scalar(0, 255, 0), 1);

      // if (markers.size() == 2)
      // {
      //   float x[2], y[2], u[2], v[2];
      //   for (unsigned int i = 0; i < 2; ++i)
      //   {
      //     ROS_DEBUG_STREAM(
      //         "Marker(" << i << ") at camera coordinates = (" << markers[i].Tvec.at<float>(0,0) << ", " << markers[i].Tvec.at<float>(1,0) << ", " << markers[i].Tvec.at<float>(2,0));
      //     // normalized coordinates of the marker
      //     x[i] = markers[i].Tvec.at<float>(0, 0) / markers[i].Tvec.at<float>(2, 0);
      //     y[i] = markers[i].Tvec.at<float>(1, 0) / markers[i].Tvec.at<float>(2, 0);
      //     // undistorted pixel
      //     u[i] = x[i] * camParam.CameraMatrix.at<float>(0, 0) + camParam.CameraMatrix.at<float>(0, 2);
      //     v[i] = y[i] * camParam.CameraMatrix.at<float>(1, 1) + camParam.CameraMatrix.at<float>(1, 2);
      //   }

      //   ROS_DEBUG_STREAM(
      //       "Mid point between the two markers in the image = (" << (x[0]+x[1])/2 << ", " << (y[0]+y[1])/2 << ")");

//        // paint a circle in the mid point of the normalized coordinates of both markers
//        cv::circle(inImage, cv::Point((u[0] + u[1]) / 2, (v[0] + v[1]) / 2), 3, cv::Scalar(0, 0, 255), cv::FILLED);

      //   // compute the midpoint in 3D:
      //   float midPoint3D[3]; // 3D point
      //   for (unsigned int i = 0; i < 3; ++i)
      //     midPoint3D[i] = (markers[0].Tvec.at<float>(i, 0) + markers[1].Tvec.at<float>(i, 0)) / 2;
      //   // now project the 3D mid point to normalized coordinates
      //   float midPointNormalized[2];
      //   midPointNormalized[0] = midPoint3D[0] / midPoint3D[2]; //x
      //   midPointNormalized[1] = midPoint3D[1] / midPoint3D[2]; //y
      //   u[0] = midPointNormalized[0] * camParam.CameraMatrix.at<float>(0, 0) + camParam.CameraMatrix.at<float>(0, 2);
      //   v[0] = midPointNormalized[1] * camParam.CameraMatrix.at<float>(1, 1) + camParam.CameraMatrix.at<float>(1, 2);

      //   ROS_DEBUG_STREAM(
      //       "3D Mid point between the two markers in undistorted pixel coordinates = (" << u[0] << ", " << v[0] << ")");

      //   // paint a circle in the mid point of the normalized coordinates of both markers
      //   cv::circle(inImage, cv::Point(u[0], v[0]), 3, cv::Scalar(0, 0, 255), cv::FILLED);

      // }

      // draw a 3D cube in each marker if there is 3D info
      // if (camParam.isValid() && marker_size != -1)
      // {
      //   for (unsigned int i = 0; i < markers.size(); ++i)
      //   {
      //     aruco::CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
      //   }
      // }

      if (image_pub.getNumSubscribers() > 0)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      if (debug_pub.getNumSubscribers() > 0)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }

      ROS_DEBUG("runtime: %f ms", 1000 * (cv::getTickCount() - ticksBefore) / cv::getTickFrequency());
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}

// wait for one camerainfo, then shut down that subscriber
void cam_info_callback(const sensor_msgs::CameraInfo &msg)
{
  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
  cam_info_received = true;
  cam_info_sub.shutdown();
}

void reconf_callback(aruco_ros::ArucoThresholdConfig &config, std::uint32_t level)
{
  // mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
  normalizeImageIllumination = config.normalizeImage;
  dctComponentsToRemove = config.dctComponentsToRemove;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "aruco_simple");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
  f_ = boost::bind(&reconf_callback, _1, _2);
  server.setCallback(f_);

  normalizeImageIllumination = false;

  nh.param<bool>("image_is_rectified", useRectifiedImages, true);
  ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

  image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
  cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);


  cam_info_received = false;
  image_pub = it.advertise("result", 1);
  debug_pub = it.advertise("debug", 1);
  pose_pub = nh.advertise<geometry_msgs::Pose>("pose", 100);

  nh.param<double>("marker_size", marker_size, 0.1);
  nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
  nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
  if (dctComponentsToRemove == 0)
    normalizeImageIllumination = false;
  nh.param<std::string>("parent_name", parent_name, "");
  nh.param<std::string>("child_name", child_name, "");

  nh.getParam("marker_ids", ids);
  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
 // Construct a proper maker corners matrix for board detection
  for (size_t i = 0; i < 4; i++)
  {
    std::vector<float> marker_corners;
    nh.getParam("marker" + std::to_string(i+1)+ "_corners", marker_corners);
    for (size_t j = 0; j < 4; j++)
    {
      cornerPoints[i].push_back(cv::Point3f(marker_corners[3 * j], marker_corners[3 * j + 1], marker_corners[3 * j + 2]));
      std::cout << cornerPoints[i][j] << std::endl;
    }
  }

  if (parent_name == "" || child_name == "")
  {
    ROS_ERROR("parent_name and/or child_name was not set!");
    return -1;
  }

  ROS_INFO("ArUco node started with marker size of %f meters and marker ids to track: %d, %d, %d, %d", marker_size, ids[0],
           ids[1], ids[2], ids[3]);
  // ROS_INFO("ArUco node will publish pose to TF with (%s, %s) and (%s, %s) as (parent,child).", parent_name.c_str(),
          //  child_name1.c_str(), parent_name.c_str(), child_name2.c_str());

  ros::spin();
}
