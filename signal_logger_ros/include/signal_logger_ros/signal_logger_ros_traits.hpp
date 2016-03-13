/*
 * signal_logger_ros_traits.hpp
 *
 *  Created on: Feb 21, 2015
 *      Author: C. Dario Bellicoso
 */

#ifndef SIGNAL_LOGGER_ROS_TRAITS_HPP_
#define SIGNAL_LOGGER_ROS_TRAITS_HPP_

#include "signal_logger/LoggerBase.hpp"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/QuaternionStamped.h"

#include <std_msgs/Float32.h>
#include <std_msgs/Time.h>

#include <signal_logger_msgs/BoolMultiArrayStamped.h>
#include <signal_logger_msgs/Float32MultiArrayStamped.h>
#include <signal_logger_msgs/Float64MultiArrayStamped.h>
#include <signal_logger_msgs/Int8MultiArrayStamped.h>
#include <signal_logger_msgs/Int16MultiArrayStamped.h>
#include <signal_logger_msgs/Int32MultiArrayStamped.h>
#include <signal_logger_msgs/Int64MultiArrayStamped.h>

#include <signal_logger_msgs/Float32Stamped.h>
#include <signal_logger_msgs/Float64Stamped.h>
#include <signal_logger_msgs/Int8Stamped.h>
#include <signal_logger_msgs/Int32Stamped.h>
#include <signal_logger_msgs/Int64Stamped.h>
#include <signal_logger_msgs/BoolStamped.h>
#include <signal_logger_msgs/CharStamped.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "kindr_msgs/VectorAtPosition.h"

namespace signal_logger_ros {

namespace traits {

// generic interface
template<typename LogType_, bool VectorAtPosition_ = false> struct slr_traits;

typedef signal_logger::LoggerBase base_type;

/*******************************
 * Specializations: core types *
 *******************************/
template<>
struct slr_traits<double> {
  typedef signal_logger_msgs::Float64Stamped         msgtype;
  typedef signal_logger_msgs::Float64StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const double* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};

template<>
struct slr_traits<float> {
  typedef signal_logger_msgs::Float32Stamped         msgtype;
  typedef signal_logger_msgs::Float32StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float32StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const float* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};

template<>
struct slr_traits<int> {
  typedef signal_logger_msgs::Int32Stamped         msgtype;
  typedef signal_logger_msgs::Int32StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int32StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const int* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};

template<>
struct slr_traits<short> {
  typedef signal_logger_msgs::Int8Stamped        msgtype;
  typedef signal_logger_msgs::Int8StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const short* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};

template<>
struct slr_traits<long> {
  typedef signal_logger_msgs::Int64Stamped         msgtype;
  typedef signal_logger_msgs::Int64StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int64StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const long* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};

template<>
struct slr_traits<char> {
  typedef signal_logger_msgs::CharStamped          msgtype;
  typedef signal_logger_msgs::CharStampedPtr       msgtypePtr;
  typedef signal_logger_msgs::CharStampedConstPtr  msgtypeConstPtr;

  static void updateMsg(const char* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};

template<>
struct slr_traits<bool> {
  typedef signal_logger_msgs::BoolStamped msgtype;
  typedef signal_logger_msgs::BoolStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::BoolStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const bool* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};
/********************************/


/********************************
 * Specializations: eigen types *
 ********************************/
template<>
struct slr_traits<Eigen::Ref<Eigen::MatrixXd>> {
  typedef signal_logger_msgs::Float64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<Eigen::MatrixXd>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;

    for (int r=0; r<static_cast<Eigen::MatrixXd>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<Eigen::MatrixXd>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::Ref<Eigen::MatrixXf>> {
  typedef signal_logger_msgs::Float32MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float32MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float32MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<Eigen::MatrixXf>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;

    for (int r=0; r<static_cast<Eigen::MatrixXf>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<Eigen::MatrixXf>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::Ref<Eigen::MatrixXi>> {
  typedef signal_logger_msgs::Int32MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int32MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int32MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<Eigen::MatrixXi>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;

    for (int r=0; r<static_cast<Eigen::MatrixXi>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<Eigen::MatrixXi>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::Ref<signal_logger::LoggerBase::MatrixXs>> {
  typedef signal_logger_msgs::Int16MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int16MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int16MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<signal_logger::LoggerBase::MatrixXs>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;

    for (int r=0; r<static_cast<signal_logger::LoggerBase::MatrixXs>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::LoggerBase::MatrixXs>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::Ref<signal_logger::LoggerBase::MatrixXl>> {
  typedef signal_logger_msgs::Int64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int64MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<signal_logger::LoggerBase::MatrixXl>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;

    for (int r=0; r<static_cast<signal_logger::LoggerBase::MatrixXl>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::LoggerBase::MatrixXl>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::Ref<signal_logger::LoggerBase::MatrixXc>> {
  typedef signal_logger_msgs::Int8MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int8MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<signal_logger::LoggerBase::MatrixXc>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;

    for (int r=0; r<static_cast<signal_logger::LoggerBase::MatrixXc>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::LoggerBase::MatrixXc>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::Ref<signal_logger::LoggerBase::MatrixXUc>> {
  typedef signal_logger_msgs::Int8MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int8MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<signal_logger::LoggerBase::MatrixXUc>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;

    for (int r=0; r<static_cast<signal_logger::LoggerBase::MatrixXUc>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::LoggerBase::MatrixXUc>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::Ref<signal_logger::LoggerBase::MatrixXb>> {
  typedef signal_logger_msgs::BoolMultiArrayStamped         msgtype;
  typedef signal_logger_msgs::BoolMultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::BoolMultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<signal_logger::LoggerBase::MatrixXb>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;

    for (int r=0; r<static_cast<signal_logger::LoggerBase::MatrixXb>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::LoggerBase::MatrixXb>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};




template<>
struct slr_traits<Eigen::Ref<Eigen::Vector3d>> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Ref<Eigen::Vector3d>* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};
/********************************/


/********************************
 * Specializations: kindr types *
 ********************************/
template<>
struct slr_traits<base_type::KindrPositionD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrPositionD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrRotationQuaternionD> {
  typedef geometry_msgs::QuaternionStamped         msgtype;
  typedef geometry_msgs::QuaternionStampedPtr      msgtypePtr;
  typedef geometry_msgs::QuaternionStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrRotationQuaternionD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->quaternion.w = vectorPtr_->w();
    msg->quaternion.x = vectorPtr_->x();
    msg->quaternion.y = vectorPtr_->y();
    msg->quaternion.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrEulerAnglesZyxD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrEulerAnglesZyxD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrAngularVelocityD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrAngularVelocityD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrLinearVelocityD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrLinearVelocityD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrLinearAccelerationD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrLinearAccelerationD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrAngularAccelerationD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrAngularAccelerationD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrForceD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrForceD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrTorqueD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrTorqueD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<base_type::KindrVectorD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrVectorD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};
/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template<>
struct slr_traits<base_type::KindrForceD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrForceD* vectorPtr_,
                        const base_type::KindrPositionD* positionPtr_,
                        const std::string& vectorFrameId,
                        const std::string& positionFrameId,
                        const std::string& name,
                        msgtypePtr msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->header.frame_id = vectorFrameId;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
    msg->position.x = positionPtr_->x();
    msg->position.y = positionPtr_->y();
    msg->position.z = positionPtr_->z();
    msg->position_frame_id = positionFrameId;
    msg->name = name;
    msg->type = kindr_msgs::VectorAtPosition::TYPE_FORCE;
  }
};

template<>
struct slr_traits<base_type::KindrTorqueD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrTorqueD* vectorPtr_,
                        const base_type::KindrPositionD* positionPtr_,
                        const std::string& vectorFrameId,
                        const std::string& positionFrameId,
                        const std::string& name,
                        msgtypePtr msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->header.frame_id = vectorFrameId;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
    msg->position.x = positionPtr_->x();
    msg->position.y = positionPtr_->y();
    msg->position.z = positionPtr_->z();
    msg->position_frame_id = positionFrameId;
    msg->name = name;
    msg->type = kindr_msgs::VectorAtPosition::TYPE_TORQUE;
  }
};

template<>
struct slr_traits<base_type::KindrVectorD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrVectorD* vectorPtr_,
                        const base_type::KindrPositionD* positionPtr_,
                        const std::string& vectorFrameId,
                        const std::string& positionFrameId,
                        const std::string& name,
                        msgtypePtr msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->header.frame_id = vectorFrameId;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
    msg->position.x = positionPtr_->x();
    msg->position.y = positionPtr_->y();
    msg->position.z = positionPtr_->z();
    msg->position_frame_id = positionFrameId;
    msg->name = name;
    msg->type = kindr_msgs::VectorAtPosition::TYPE_TYPELESS;
  }
};

template<>
struct slr_traits<base_type::KindrLinearVelocityD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrLinearVelocityD* vectorPtr_,
                        const base_type::KindrPositionD* positionPtr_,
                        const std::string& vectorFrameId,
                        const std::string& positionFrameId,
                        const std::string& name,
                        msgtypePtr msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->header.frame_id = vectorFrameId;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
    msg->position.x = positionPtr_->x();
    msg->position.y = positionPtr_->y();
    msg->position.z = positionPtr_->z();
    msg->position_frame_id = positionFrameId;
    msg->name = name;
    msg->type = kindr_msgs::VectorAtPosition::TYPE_VELOCITY;
  }
};

template<>
struct slr_traits<base_type::KindrLinearAccelerationD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::KindrLinearAccelerationD* vectorPtr_,
                        const base_type::KindrPositionD* positionPtr_,
                        const std::string& vectorFrameId,
                        const std::string& positionFrameId,
                        const std::string& name,
                        msgtypePtr msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->header.frame_id = vectorFrameId;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
    msg->position.x = positionPtr_->x();
    msg->position.y = positionPtr_->y();
    msg->position.z = positionPtr_->z();
    msg->position_frame_id = positionFrameId;
    msg->name = name;
    msg->type = kindr_msgs::VectorAtPosition::TYPE_ACCELERATION;
  }
};
/***************************************************/


template<>
struct slr_traits<base_type::TimestampPair> {
  typedef std_msgs::Time         msgtype;
  typedef std_msgs::TimePtr      msgtypePtr;
  typedef std_msgs::TimeConstPtr msgtypeConstPtr;

  static void updateMsg(const base_type::TimestampPair* var, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->data.sec = var->first;
    msg->data.nsec = var->second;
  }
};


} /* namespace traits */

} /* namespace signal_logger_ros */

#endif /* SIGNAL_LOGGER_ROS_TRAITS_HPP_ */
