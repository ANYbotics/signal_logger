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

// Type traits
#ifdef SILO_USE_KINDR
template<typename>
struct is_kindr_vector : std::false_type {};

template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector<kindr::Vector<PhysicalType_,PrimType_, Dimension_>> : std::true_type {};

template<typename>
struct is_kindr_vector_at_position : std::false_type {};

template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector_at_position<signal_logger::KindrVectorAtPosition<kindr::Vector<PhysicalType_,PrimType_, Dimension_>>> : std::true_type {};
#endif

template<typename>
struct is_eigen_angle_axis : std::false_type {};

template<typename PrimType_>
struct is_eigen_angle_axis<Eigen::AngleAxis<PrimType_>> : std::true_type {};



// generic interface
template<typename ValueType_, typename Enable_ = void> struct slr_msg_traits;

/*******************************
 * Specializations: core types *
 *******************************/
template<>
struct slr_msg_traits<double> {
  typedef signal_logger_msgs::Float64Stamped         msgtype;
  typedef signal_logger_msgs::Float64StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64StampedConstPtr msgtypeConstPtr;
};

template<>
struct slr_msg_traits<float> {
  typedef signal_logger_msgs::Float32Stamped         msgtype;
  typedef signal_logger_msgs::Float32StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float32StampedConstPtr msgtypeConstPtr;
};

template<>
struct slr_msg_traits<int> {
  typedef signal_logger_msgs::Int32Stamped         msgtype;
  typedef signal_logger_msgs::Int32StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int32StampedConstPtr msgtypeConstPtr;
};

template<>
struct slr_msg_traits<short> {
  typedef signal_logger_msgs::Int8Stamped        msgtype;
  typedef signal_logger_msgs::Int8StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8StampedConstPtr msgtypeConstPtr;
};

template<>
struct slr_msg_traits<long> {
  typedef signal_logger_msgs::Int64Stamped         msgtype;
  typedef signal_logger_msgs::Int64StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int64StampedConstPtr msgtypeConstPtr;
};

template<>
struct slr_msg_traits<char> {
  typedef signal_logger_msgs::CharStamped          msgtype;
  typedef signal_logger_msgs::CharStampedPtr       msgtypePtr;
  typedef signal_logger_msgs::CharStampedConstPtr  msgtypeConstPtr;
};

template<>
struct slr_msg_traits<bool> {
  typedef signal_logger_msgs::BoolStamped msgtype;
  typedef signal_logger_msgs::BoolStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::BoolStampedConstPtr msgtypeConstPtr;
};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_same<signal_logger::TimestampPair, ValueType_>::value>::type>
{
  typedef std_msgs::Time         msgtype;
  typedef std_msgs::TimePtr      msgtypePtr;
  typedef std_msgs::TimeConstPtr msgtypeConstPtr;
};
/********************************/

/********************************
 * Specializations: eigen types *
 ********************************/
template<>
struct slr_msg_traits<Eigen::Vector3d> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type>
{
  typedef signal_logger_msgs::Float64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value
                                                      && std::is_same<typename ValueType_::Scalar, double>::value
                                                      && !std::is_same<ValueType_, Eigen::Vector3d>::value>::type> {
  typedef signal_logger_msgs::Float64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value
                                                      && std::is_same<typename ValueType_::Scalar, float>::value>::type> {
  typedef signal_logger_msgs::Float32MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float32MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float32MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value
                                                      && std::is_same<typename ValueType_::Scalar, long>::value>::type> {
  typedef signal_logger_msgs::Int64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int64MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value
                                                      && std::is_same<typename ValueType_::Scalar, int>::value>::type> {
  typedef signal_logger_msgs::Int32MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int32MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int32MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value
                                                      && std::is_same<typename ValueType_::Scalar, short>::value>::type> {
  typedef signal_logger_msgs::Int16MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int16MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int16MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value
                                                      && std::is_same<typename ValueType_::Scalar, char>::value>::type> {
  typedef signal_logger_msgs::Int8MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int8MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value
                                                      && std::is_same<typename ValueType_::Scalar, unsigned char>::value>::type> {
  typedef signal_logger_msgs::Int8MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int8MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value
                                                      && std::is_same<typename ValueType_::Scalar, bool>::value>::type> {
  typedef signal_logger_msgs::BoolMultiArrayStamped         msgtype;
  typedef signal_logger_msgs::BoolMultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::BoolMultiArrayStampedConstPtr msgtypeConstPtr;
};
/********************************/

/********************************
 * Specializations: kindr types *
 ********************************/
#ifdef SILO_USE_KINDR

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_vector<ValueType_>::value && ValueType_::Dimension == 3>::type>
{
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;
};

//! Trait for Kindr rotations
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::RotationBase<ValueType_>,ValueType_>::value>::type>
{
  typedef typename slr_msg_traits<typename ValueType_::Implementation>::msgtype      msgtype;
  typedef typename slr_msg_traits<typename ValueType_::Implementation>::msgtypePtr   msgtypePtr;
  typedef typename slr_msg_traits<typename ValueType_::Implementation>::msgtypeConstPtr  msgtypeConstPtr;
};

template<>
struct slr_msg_traits<signal_logger::KindrRotationQuaternionD> {
  typedef geometry_msgs::QuaternionStamped         msgtype;
  typedef geometry_msgs::QuaternionStampedPtr      msgtypePtr;
  typedef geometry_msgs::QuaternionStampedConstPtr msgtypeConstPtr;
};

template<>
struct slr_msg_traits<signal_logger::KindrAngularVelocityD> {
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;
};
/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  typedef kindr_msgs::VectorAtPosition         msgtype;
  typedef kindr_msgs::VectorAtPositionPtr      msgtypePtr;
  typedef kindr_msgs::VectorAtPositionConstPtr msgtypeConstPtr;
};
/***************************************************/
#endif

// generic interface
template<typename ValueType_, typename Enable_ = void> struct slr_update_traits;

/*******************************
 * Specializations: core types *
 *******************************/
template<typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_arithmetic<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_same<signal_logger::TimestampPair, ValueType_>::value>::type>
{
  static void updateMsg(const signal_logger::TimestampPair* var, typename slr_msg_traits<ValueType_>::msgtypePtr msg, const ros::Time& timeStamp) {
    msg->data.sec = var->first;
    msg->data.nsec = var->second;
  }
};
/********************************/

/********************************
 * Specializations: eigen types *
 ********************************/
template<>
struct slr_update_traits<Eigen::Vector3d> {
  static void updateMsg(const Eigen::Vector3d * vectorPtr_, slr_msg_traits<Eigen::Vector3d>::msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type>
{
  static void updateMsg(const ValueType_ * vectorPtr_, typename slr_msg_traits<ValueType_>::msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    for (int r=0; r<vectorPtr_->rows(); r++)  {
      for (int c=0; c<vectorPtr_->cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type>
{
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    msg->matrix.data.push_back(vectorPtr_->angle());
    msg->matrix.data.push_back(vectorPtr_->axis()(0));
    msg->matrix.data.push_back(vectorPtr_->axis()(1));
    msg->matrix.data.push_back(vectorPtr_->axis()(2));
  }
};

/********************************/

#ifdef SILO_USE_KINDR
/********************************
 * Specializations: kindr types *
 ********************************/

//! Trait for Kindr vectors length 3
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_kindr_vector<ValueType_>::value && ValueType_::Dimension == 3>::type>
{
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

//! Trait for Kindr rotations
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::RotationBase<ValueType_>,ValueType_>::value>::type>
{
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtypePtr msg, const ros::Time& timeStamp) {
    slr_update_traits<typename ValueType_::Implementation>::updateMsg(&vectorPtr_->toImplementation(), msg, timeStamp);
  }
};

template <>
struct slr_update_traits<signal_logger::KindrRotationQuaternionD> {
  static void updateMsg(const signal_logger::KindrRotationQuaternionD* vectorPtr_, typename slr_msg_traits<signal_logger::KindrRotationQuaternionD>::msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->quaternion.w = vectorPtr_->w();
    msg->quaternion.x = vectorPtr_->x();
    msg->quaternion.y = vectorPtr_->y();
    msg->quaternion.z = vectorPtr_->z();
  }
};

template <>
struct slr_update_traits<signal_logger::KindrAngularVelocityD> {
  static void updateMsg(const signal_logger::KindrAngularVelocityD* vectorPtr_, typename slr_msg_traits<signal_logger::KindrAngularVelocityD>::msgtypePtr msg, const ros::Time& timeStamp) {
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
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtypePtr msg, const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->header.frame_id = " ";
    msg->vector.x = vectorPtr_->vector.x();
    msg->vector.y = vectorPtr_->vector.y();
    msg->vector.z = vectorPtr_->vector.z();
    msg->position.x = vectorPtr_->position.x();
    msg->position.y = vectorPtr_->position.y();
    msg->position.z = vectorPtr_->position.z();
    msg->position_frame_id = " ";
    msg->name = " ";
    msg->type = getType();
  }

  static int getType()
  {
    switch(typeid(ValueType_))
    {
      case typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearAccelerationD>):
        return kindr_msgs::VectorAtPosition::TYPE_ACCELERATION;
      case typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearVelocityD>):
        return kindr_msgs::VectorAtPosition::TYPE_VELOCITY;
      case typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrForceD>):
        return kindr_msgs::VectorAtPosition::TYPE_FORCE;
      case typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrTorqueD>):
        return kindr_msgs::VectorAtPosition::TYPE_TORQUE;
      case typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrVectorD>):
        return kindr_msgs::VectorAtPosition::TYPE_TYPELESS;
    }

    return kindr_msgs::VectorAtPosition::TYPE_TYPELESS;
  }
};
/***************************************************/
#endif

} /* namespace traits */

} /* namespace signal_logger_ros */
