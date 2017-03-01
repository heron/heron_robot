/**
 *
 *  \file
 *  \brief      C++ implementation of the NMEA -> cmd_* command
                republisher for Heron.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2015, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <boost/algorithm/string.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include "heron_msgs/Course.h"
#include "heron_msgs/Drive.h"
#include "heron_msgs/Helm.h"
#include "nmea_msgs/Sentence.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"

// Boost provides functions for these, but they're a pain in the butt.
static const double TO_RADIANS = 3.14159265359 / 180.0;
static const double TO_DEGREES = 180.0 / 3.14159265359;


class Helper
{
public:
  typedef boost::function<void(const ros::V_string&)> callback_fn_t;

  Helper(ros::NodeHandle* nh, std::string sentence_type, callback_fn_t callback_fn) :
    sentence_type_(sentence_type),
    callback_fn_(callback_fn),
    sub_(nh->subscribe("rx" /*"nmea_sentence"*/, 1, &Helper::cb, this))
  {
  }

private:
  void cb(const nmea_msgs::Sentence& sentence_msg)
  {
    ROS_DEBUG_STREAM("Sentence received: " << sentence_msg.sentence);
    boost::smatch matches;
    if (!boost::regex_match(sentence_msg.sentence, matches, sentence_regex))
    {
      ROS_WARN("Sentence recieved did not match regex.");
      return;
    }

    std::string sentence_type(matches[1].first, matches[1].second);
    if (sentence_type == sentence_type_)
    {
      std::string sentence_fields(matches[2].first, matches[2].second);
      ROS_DEBUG_STREAM("Extracting fields from sentence of type: " << sentence_type);
      ROS_DEBUG_STREAM("Fields string: " << sentence_fields);

      ros::V_string fields;
      boost::split(fields, sentence_fields, boost::is_any_of(","));

      try
      {
        // Pass the fields to the supplied callback function to process.
        callback_fn_(fields);
      }
      catch(boost::bad_lexical_cast&)
      {
        ROS_WARN("Unable to parse values from sentence.");
      }
    }
  }

  static const boost::regex sentence_regex;
  std::string sentence_type_;
  callback_fn_t callback_fn_;
  ros::Subscriber sub_;
};

/**
 * This regex verifies the basic form of an NMEA sentence, and captures three groups: the sentence
 * name, all remaining fields, and the checksum. The middle group must be further split on the
 * comma character to separate individual field strings.
 */
const boost::regex Helper::sentence_regex("^\\$([A-Za-z]+),([A-Za-z0-9,.-]+)\\*([0-9A-Za-z]{2})?");


class DrivePublisher : public Helper
{
public:
  DrivePublisher(ros::NodeHandle* nh) :
    Helper(nh, "PYDIR", boost::bind(&DrivePublisher::cb, this, _1)),
    pub_(nh->advertise<heron_msgs::Drive>("cmd_drive", 1))
  {
  }

private:
  void cb(const ros::V_string& fields)
  {
    heron_msgs::Drive drive_msg;
    drive_msg.left = boost::lexical_cast<double>(fields[0]) * 0.01;
    drive_msg.right = boost::lexical_cast<double>(fields[1]) * 0.01;
    pub_.publish(drive_msg);
  }

  ros::Publisher pub_;
};


class HelmPublisher : public Helper
{
public:
  HelmPublisher(ros::NodeHandle* nh) :
    Helper(nh, "PYDEP", boost::bind(&HelmPublisher::cb, this, _1)),
    pub_(nh->advertise<heron_msgs::Helm>("cmd_helm", 1))
  {
  }

private:
  void cb(const ros::V_string& fields)
  {
    heron_msgs::Helm helm_msg;
    helm_msg.yaw_rate = boost::lexical_cast<double>(fields[0]) * TO_RADIANS * -1;
    helm_msg.thrust = boost::lexical_cast<double>(fields[1]) * 0.01;
    pub_.publish(helm_msg);
  }

  ros::Publisher pub_;
};


class CoursePublisher : public Helper
{
public:
  CoursePublisher(ros::NodeHandle* nh) :
    Helper(nh, "PYDEV", boost::bind(&CoursePublisher::cb, this, _1)),
    pub_(nh->advertise<heron_msgs::Course>("cmd_course", 1))
  {
  }

private:
  void cb(const ros::V_string& fields)
  {
    heron_msgs::Course course_msg;
    course_msg.yaw = (90 - boost::lexical_cast<double>(fields[0])) * TO_RADIANS;
    course_msg.speed = boost::lexical_cast<double>(fields[1]);
    pub_.publish(course_msg);
  }

  ros::Publisher pub_;
};


class LightsPublisher : public Helper
{
public:
  LightsPublisher(ros::NodeHandle* nh) :
    Helper(nh, "PYCLT", boost::bind(&LightsPublisher::cb, this, _1)),
    pub_(nh->advertise<std_msgs::Bool>("disable_lights", 1))
  {
  }

private:
  void cb(const ros::V_string& fields)
  {
    std_msgs::Bool lights_msg;
    lights_msg.data = boost::lexical_cast<bool>(fields[0]);
    pub_.publish(lights_msg);
  }

  ros::Publisher pub_;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "heron_nmea_command_publisher");

  ros::NodeHandle nh;
  DrivePublisher dp(&nh);
  HelmPublisher hp(&nh);
  CoursePublisher cp(&nh);
  LightsPublisher lp(&nh);
  ros::spin();
}
