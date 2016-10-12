/*
 * signal_logger_ros_traits.hpp
 *
 *  Created on: Feb 21, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

#include "signal_logger/LogElementTypes.hpp"

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
#ifdef SILO_USE_KINDR
#include "kindr_msgs/VectorAtPosition.h"
#endif
namespace signal_logger_ros {

namespace traits {

// generic interface
template<typename LogType_, bool VectorAtPosition_ = false> struct slr_traits;

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
struct slr_traits<Eigen::MatrixXd> {
  typedef signal_logger_msgs::Float64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::MatrixXd* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    for (int r=0; r<static_cast<Eigen::MatrixXd>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<Eigen::MatrixXd>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::MatrixXf> {
  typedef signal_logger_msgs::Float32MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float32MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float32MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::MatrixXf* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();

    for (int r=0; r<static_cast<Eigen::MatrixXf>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<Eigen::MatrixXf>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<Eigen::MatrixXi> {
  typedef signal_logger_msgs::Int32MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int32MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int32MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::MatrixXi* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();

    for (int r=0; r<static_cast<Eigen::MatrixXi>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<Eigen::MatrixXi>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<signal_logger::MatrixXs> {
  typedef signal_logger_msgs::Int16MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int16MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int16MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::MatrixXs* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();

    for (int r=0; r<static_cast<signal_logger::MatrixXs>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::MatrixXs>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<signal_logger::MatrixXl> {
  typedef signal_logger_msgs::Int64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int64MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::MatrixXl* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();

    for (int r=0; r<static_cast<signal_logger::MatrixXl>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::MatrixXl>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<signal_logger::MatrixXc> {
  typedef signal_logger_msgs::Int8MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int8MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::MatrixXc* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();

    for (int r=0; r<static_cast<signal_logger::MatrixXc>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::MatrixXc>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<signal_logger::MatrixXUc> {
  typedef signal_logger_msgs::Int8MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int8MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::MatrixXUc * vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();

    for (int r=0; r<static_cast<signal_logger::MatrixXUc>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::MatrixXUc>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template<>
struct slr_traits<signal_logger::MatrixXb> {
  typedef signal_logger_msgs::BoolMultiArrayStamped         msgtype;
  typedef signal_logger_msgs::BoolMultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::BoolMultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::MatrixXb * vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();

    for (int r=0; r<static_cast<signal_logger::MatrixXb>(*vectorPtr_).rows(); r++)  {
      for (int c=0; c<static_cast<signal_logger::MatrixXb>(*vectorPtr_).cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};




template<>
struct slr_traits<Eigen::Vector3d> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const Eigen::Vector3d * vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};
/********************************/


#ifdef SILO_USE_KINDR
/********************************
 * Specializations: kindr types *
 ********************************/
template<>
struct slr_traits<signal_logger::KindrPositionD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrPositionD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrRotationQuaternionD> {
  typedef geometry_msgs::QuaternionStamped         msgtype;
  typedef geometry_msgs::QuaternionStampedPtr      msgtypePtr;
  typedef geometry_msgs::QuaternionStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrRotationQuaternionD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->quaternion.w = vectorPtr_->w();
    msg->quaternion.x = vectorPtr_->x();
    msg->quaternion.y = vectorPtr_->y();
    msg->quaternion.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrEulerAnglesZyxD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrEulerAnglesZyxD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrAngleAxisD> {
  typedef signal_logger_msgs::Float64MultiArrayStamped                msgtype;
  typedef signal_logger_msgs::Float64MultiArrayStampedPtr             msgtypePtr;
  typedef signal_logger_msgs::Float64MultiArrayStampedConstPtr        msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrAngleAxisD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    msg->matrix.data.push_back(vectorPtr_->angle());
    msg->matrix.data.push_back(vectorPtr_->axis()(0));
    msg->matrix.data.push_back(vectorPtr_->axis()(1));
    msg->matrix.data.push_back(vectorPtr_->axis()(2));
//    msg->matrix.layout.dim[0].label = "alphaXYZ";
//    msg->matrix.layout.dim[0].size = 4;
//    msg->matrix.layout.dim[0].stride = 4;
  }
};

template<>
struct slr_traits<signal_logger::KindrRotationMatrixD> {
  typedef signal_logger_msgs::Float64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64MultiArrayStampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrRotationMatrixD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
//    msg->matrix.layout.dim[0].label = "row";
//    msg->matrix.layout.dim[0].size = 3;
//    msg->matrix.layout.dim[0].stride = 9;
//    msg->matrix.layout.dim[1].label = "col";
//    msg->matrix.layout.dim[1].size = 3;
//    msg->matrix.layout.dim[1].stride = 9;
    msg->matrix.data.clear();
    for (int r=0; r<3; r++)  {
      for (int c=0; c<3; c++)  {
        msg->matrix.data.push_back(vectorPtr_->toImplementation()(r,c));
      }
    }
  }
};

template<>
struct slr_traits<signal_logger::KindrRotationVectorD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrRotationVectorD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrAngularVelocityD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrAngularVelocityD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrLinearVelocityD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrLinearVelocityD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrLinearAccelerationD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrLinearAccelerationD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrAngularAccelerationD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrAngularAccelerationD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrForceD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrForceD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrTorqueD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrTorqueD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template<>
struct slr_traits<signal_logger::KindrVectorD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrVectorD* vectorPtr_, msgtypePtr msg, const ros::Time& timeStamp) {
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
struct slr_traits<signal_logger::KindrForceD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrForceD* vectorPtr_,
                        const signal_logger::KindrPositionD* positionPtr_,
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
struct slr_traits<signal_logger::KindrTorqueD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrTorqueD* vectorPtr_,
                        const signal_logger::KindrPositionD* positionPtr_,
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
struct slr_traits<signal_logger::KindrVectorD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrVectorD* vectorPtr_,
                        const signal_logger::KindrPositionD* positionPtr_,
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
struct slr_traits<signal_logger::KindrLinearVelocityD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrLinearVelocityD* vectorPtr_,
                        const signal_logger::KindrPositionD* positionPtr_,
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
struct slr_traits<signal_logger::KindrLinearAccelerationD, true> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::KindrLinearAccelerationD* vectorPtr_,
                        const signal_logger::KindrPositionD* positionPtr_,
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
#endif

template<>
struct slr_traits<signal_logger::TimestampPair> {
  typedef std_msgs::Time         msgtype;
  typedef std_msgs::TimePtr      msgtypePtr;
  typedef std_msgs::TimeConstPtr msgtypeConstPtr;

  static void updateMsg(const signal_logger::TimestampPair* var, msgtypePtr msg, const ros::Time& timeStamp) {
    msg->data.sec = var->first;
    msg->data.nsec = var->second;
  }
};


} /* namespace traits */

} /* namespace signal_logger_ros */
