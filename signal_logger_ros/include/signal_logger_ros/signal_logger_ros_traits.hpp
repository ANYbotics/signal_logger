/*
 * signal_logger_ros_traits.hpp
 *
 *  Created on: Feb 21, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

#include "signal_logger_core/LogElementTypes.hpp"
#include "signal_logger_core/signal_logger_traits.hpp"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/QuaternionStamped.h"

#include <std_msgs/Float32.h>

#include <signal_logger_msgs/TimeStamped.h>

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
#include <signal_logger_msgs/UInt8Stamped.h>
#include <signal_logger_msgs/Int32Stamped.h>
#include <signal_logger_msgs/Int64Stamped.h>
#include <signal_logger_msgs/BoolStamped.h>
#include <signal_logger_msgs/CharStamped.h>
#include <signal_logger_msgs/UnsignedCharStamped.h>

#include "geometry_msgs/Vector3Stamped.h"
#ifdef SILO_USE_KINDR
#include "kindr_msgs/VectorAtPosition.h"
#endif
namespace signal_logger_ros {

namespace traits {

using namespace signal_logger::traits;

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
  typedef signal_logger_msgs::Int8Stamped         msgtype;
  typedef signal_logger_msgs::Int8StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8StampedConstPtr msgtypeConstPtr;
};

template<>
struct slr_msg_traits<short unsigned int> {
  typedef signal_logger_msgs::UInt8Stamped         msgtype;
  typedef signal_logger_msgs::UInt8StampedPtr      msgtypePtr;
  typedef signal_logger_msgs::UInt8StampedConstPtr msgtypeConstPtr;
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
struct slr_msg_traits<unsigned char> {
  typedef signal_logger_msgs::UnsignedCharStamped          msgtype;
  typedef signal_logger_msgs::UnsignedCharStampedPtr       msgtypePtr;
  typedef signal_logger_msgs::UnsignedCharStampedConstPtr  msgtypeConstPtr;
};

template<>
struct slr_msg_traits<bool> {
  typedef signal_logger_msgs::BoolStamped msgtype;
  typedef signal_logger_msgs::BoolStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::BoolStampedConstPtr msgtypeConstPtr;
};
/********************************/

/*******************************
 * Specializations: enum types *
 *******************************/
template<typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type>
{
  typedef typename slr_msg_traits<typename std::underlying_type<ValueType_>::type >::msgtype          msgtype;
  typedef typename slr_msg_traits<typename std::underlying_type<ValueType_>::type >::msgtypePtr       msgtypePtr;
  typedef typename slr_msg_traits<typename std::underlying_type<ValueType_>::type >::msgtypeConstPtr  msgtypeConstPtr;

};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template<>
struct slr_msg_traits<signal_logger::TimestampPair>
{
  typedef signal_logger_msgs::TimeStamped         msgtype;
  typedef signal_logger_msgs::TimeStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::TimeStampedConstPtr msgtypeConstPtr;
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
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type>
{
  typedef geometry_msgs::QuaternionStamped         msgtype;
  typedef geometry_msgs::QuaternionStampedPtr      msgtypePtr;
  typedef geometry_msgs::QuaternionStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type>
{
  typedef signal_logger_msgs::Float64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, double>::value>::type>
{
  typedef signal_logger_msgs::Float64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float64MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, float>::value>::type>
{
  typedef signal_logger_msgs::Float32MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Float32MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Float32MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, long>::value>::type>
{
  typedef signal_logger_msgs::Int64MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int64MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int64MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, int>::value>::type>
{
  typedef signal_logger_msgs::Int32MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int32MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int32MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, short>::value>::type>
{
  typedef signal_logger_msgs::Int16MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int16MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int16MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, char>::value>::type>
{
  typedef signal_logger_msgs::Int8MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int8MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, unsigned char>::value>::type>
{
  typedef signal_logger_msgs::Int8MultiArrayStamped         msgtype;
  typedef signal_logger_msgs::Int8MultiArrayStampedPtr      msgtypePtr;
  typedef signal_logger_msgs::Int8MultiArrayStampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, bool>::value>::type>
{
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
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_vector3<ValueType_>::value>::type>
{
  typedef geometry_msgs::Vector3Stamped         msgtype;
  typedef geometry_msgs::Vector3StampedPtr      msgtypePtr;
  typedef geometry_msgs::Vector3StampedConstPtr msgtypeConstPtr;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::RotationBase<ValueType_>,ValueType_>::value
||std::is_base_of<kindr::RotationDiffBase<ValueType_>,ValueType_>::value>::type>
{
  typedef typename slr_msg_traits<typename ValueType_::Implementation>::msgtype     	 msgtype;
  typedef typename slr_msg_traits<typename ValueType_::Implementation>::msgtypePtr       msgtypePtr;
  typedef typename slr_msg_traits<typename ValueType_::Implementation>::msgtypeConstPtr  msgtypeConstPtr;
};

/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type>
{
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
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_arithmetic<ValueType_>::value>::type>
{
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};
/********************************/

/*******************************
 * Specializations: enum types *
 *******************************/
template<typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type>
{
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    typename std::underlying_type<ValueType_>::type vectorPtr = static_cast< typename std::underlying_type<ValueType_>::type >(*vectorPtr_);
    slr_update_traits<typename std::underlying_type<ValueType_>::type >::updateMsg(&vectorPtr, msg, timeStamp);
  }
};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <>
struct slr_update_traits<signal_logger::TimestampPair>
{
  static void updateMsg(const signal_logger::TimestampPair* var,
                        typename slr_msg_traits<signal_logger::TimestampPair>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->value.data.sec = var->first;
    msg->value.data.nsec = var->second;
  }
};
/********************************/

/********************************
 * Specializations: eigen types *
 ********************************/
template<>
struct slr_update_traits<Eigen::Vector3d>
{
  static void updateMsg(const Eigen::Vector3d * vectorPtr_,
                        slr_msg_traits<Eigen::Vector3d>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type>
{
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->quaternion.w = vectorPtr_->w();
    msg->quaternion.x = vectorPtr_->x();
    msg->quaternion.y = vectorPtr_->y();
    msg->quaternion.z = vectorPtr_->z();
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type>
{
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    msg->matrix.data.push_back(vectorPtr_->angle());
    msg->matrix.data.push_back(vectorPtr_->axis()(0));
    msg->matrix.data.push_back(vectorPtr_->axis()(1));
    msg->matrix.data.push_back(vectorPtr_->axis()(2));
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if< is_eigen_matrix<ValueType_>::value >::type>
{
  static void updateMsg(const ValueType_ * vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    for (int r=0; r<vectorPtr_->rows(); r++)  {
      for (int c=0; c<vectorPtr_->cols(); c++)  {
        msg->matrix.data.push_back((*vectorPtr_)(r,c));
      }
    }
  }
};
/********************************/

#ifdef SILO_USE_KINDR
/********************************
 * Specializations: kindr types *
 ********************************/
//! Trait for Kindr vectors length 3
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_kindr_vector3<ValueType_>::value>::type>
{
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

//! Trait for Kindr rotations
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::RotationBase<ValueType_>,ValueType_>::value
|| std::is_base_of<kindr::RotationDiffBase<ValueType_>,ValueType_>::value>::type >
{
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    slr_update_traits<typename ValueType_::Implementation>::updateMsg(&vectorPtr_->toImplementation(), msg, timeStamp);
  }
};
/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtypePtr msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->header.frame_id = vectorPtr_->vectorFrame;
    msg->vector.x = vectorPtr_->vector.x();
    msg->vector.y = vectorPtr_->vector.y();
    msg->vector.z = vectorPtr_->vector.z();
    msg->position.x = vectorPtr_->position.x();
    msg->position.y = vectorPtr_->position.y();
    msg->position.z = vectorPtr_->position.z();
    msg->position_frame_id = vectorPtr_->positionFrame;
    msg->name = " ";
    msg->type = getType();
  }

  static int getType()
  {
    if( typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearAccelerationD>) )
      return kindr_msgs::VectorAtPosition::TYPE_ACCELERATION;
    if( typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearVelocityD>) )
      return kindr_msgs::VectorAtPosition::TYPE_VELOCITY;
    if( typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrForceD>) )
      return kindr_msgs::VectorAtPosition::TYPE_FORCE;
    if( typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrTorqueD>) )
      return kindr_msgs::VectorAtPosition::TYPE_TORQUE;

    return kindr_msgs::VectorAtPosition::TYPE_TYPELESS;
  }
};
/***************************************************/
#endif

} /* namespace traits */

} /* namespace signal_logger_ros */
