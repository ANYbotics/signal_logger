/*!
 * @file     signal_logger_ros_traits.hpp
 * @author   C. Dario Bellicoso, Gabriel Hottiger, Christian Gehring
 * @date     Feb 21, 2015
 * @brief    Message update traits for all supported types.
 */

#pragma once

#include "signal_logger_core/LogElementTypes.hpp"
#include "signal_logger_core/signal_logger_traits.hpp"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include <std_msgs/Float32.h>

#include <signal_logger_msgs/TimeStamped.h>

#include <signal_logger_msgs/BoolMultiArrayStamped.h>
#include <signal_logger_msgs/Float32MultiArrayStamped.h>
#include <signal_logger_msgs/Float64MultiArrayStamped.h>
#include <signal_logger_msgs/Int16MultiArrayStamped.h>
#include <signal_logger_msgs/Int32MultiArrayStamped.h>
#include <signal_logger_msgs/Int64MultiArrayStamped.h>
#include <signal_logger_msgs/Int8MultiArrayStamped.h>

#include <signal_logger_msgs/BoolStamped.h>
#include <signal_logger_msgs/CharStamped.h>
#include <signal_logger_msgs/Float32Stamped.h>
#include <signal_logger_msgs/Float64Stamped.h>
#include <signal_logger_msgs/Int16Stamped.h>
#include <signal_logger_msgs/Int32Stamped.h>
#include <signal_logger_msgs/Int64Stamped.h>
#include <signal_logger_msgs/Int8Stamped.h>
#include <signal_logger_msgs/UInt16Stamped.h>
#include <signal_logger_msgs/UInt32Stamped.h>
#include <signal_logger_msgs/UInt64Stamped.h>
#include <signal_logger_msgs/UInt8Stamped.h>
#include <signal_logger_msgs/UnsignedCharStamped.h>

#include "geometry_msgs/Vector3Stamped.h"
#ifdef SILO_USE_KINDR
#include "kindr_msgs/VectorAtPosition.h"
#endif
namespace signal_logger_ros {

namespace traits {

using namespace signal_logger::traits;

// generic interface
template <typename ValueType_, typename Enable_ = void>
struct slr_msg_traits;

/*******************************
 * Specializations: core types *
 *******************************/
template <>
struct slr_msg_traits<double> {
  using msgtype = signal_logger_msgs::Float64Stamped;
};

template <>
struct slr_msg_traits<float> {
  using msgtype = signal_logger_msgs::Float32Stamped;
};

template <>
struct slr_msg_traits<bool> {
  using msgtype = signal_logger_msgs::BoolStamped;
};

template <>
struct slr_msg_traits<char> {
  using msgtype = signal_logger_msgs::CharStamped;
};

template <>
struct slr_msg_traits<signed char> {
  using msgtype = signal_logger_msgs::Int8Stamped;
};

template <>
struct slr_msg_traits<unsigned char> {
  using msgtype = signal_logger_msgs::UnsignedCharStamped;
};

template <>
struct slr_msg_traits<short> {
  using msgtype = signal_logger_msgs::Int16Stamped;
};

template <>
struct slr_msg_traits<unsigned short> {
  using msgtype = signal_logger_msgs::UInt16Stamped;
};

template <>
struct slr_msg_traits<int> {
  using msgtype = signal_logger_msgs::Int32Stamped;
};

template <>
struct slr_msg_traits<unsigned int> {
  using msgtype = signal_logger_msgs::UInt32Stamped;
};

template <>
struct slr_msg_traits<long> {
  using msgtype = signal_logger_msgs::Int64Stamped;
};

template <>
struct slr_msg_traits<unsigned long> {
  using msgtype = signal_logger_msgs::UInt64Stamped;
};

/********************************/

/*******************************
 * Specializations: enum types *
 *******************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type> {
  using msgtype = typename slr_msg_traits<typename std::underlying_type<ValueType_>::type>::msgtype;
};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <>
struct slr_msg_traits<signal_logger::TimestampPair> {
  using msgtype = signal_logger_msgs::TimeStamped;
};
/********************************/


/***************************************************
 * Specializations: STL types                      *
 ***************************************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_array_type<ValueType_>::value &&
    std::is_same<double, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::Float64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_array_type<ValueType_>::value &&
    std::is_same<float, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::Float32MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_array_type<ValueType_>::value &&
    std::is_same<long, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::Int64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_array_type<ValueType_>::value &&
    std::is_same<int, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::Int32MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_array_type<ValueType_>::value &&
    std::is_same<short, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::Int16MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_array_type<ValueType_>::value &&
    std::is_same<char, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::Int8MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_array_type<ValueType_>::value &&
    std::is_same<bool, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::BoolMultiArrayStamped;
};

/********************************/

/********************************
 * Specializations: eigen types *
 ********************************/
template <>
struct slr_msg_traits<Eigen::Vector3d> {
  using msgtype = geometry_msgs::Vector3Stamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type> {
  using msgtype = geometry_msgs::QuaternionStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type> {
  using msgtype = signal_logger_msgs::Float64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar_excluding_vector3<ValueType_, double>::value>::type> {
  using msgtype = signal_logger_msgs::Float64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, float>::value>::type> {
  using msgtype = signal_logger_msgs::Float32MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, long>::value>::type> {
  using msgtype = signal_logger_msgs::Int64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, int>::value>::type> {
  using msgtype = signal_logger_msgs::Int32MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, short>::value>::type> {
  using msgtype = signal_logger_msgs::Int16MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, char>::value>::type> {
  using msgtype = signal_logger_msgs::Int8MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_,
                      typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, unsigned char>::value>::type> {
  using msgtype = signal_logger_msgs::Int8MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, bool>::value>::type> {
  using msgtype = signal_logger_msgs::BoolMultiArrayStamped;
};
/********************************/

/********************************
 * Specializations: kindr types *
 ********************************/
#ifdef SILO_USE_KINDR

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_vector<ValueType_>::value>::type> {
  using msgtype = typename slr_msg_traits<typename ValueType_::Implementation>::msgtype;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<
                                      std::is_base_of<kindr::RotationBase<ValueType_>, ValueType_>::value ||
                                      std::is_base_of<kindr::RotationDiffBase<ValueType_>, ValueType_>::value>::type> {
  using msgtype = typename slr_msg_traits<typename ValueType_::Implementation>::msgtype;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_,
                      typename std::enable_if<is_kindr_homogeneous_transformation<ValueType_>::value>::type> {
  using msgtype = geometry_msgs::PoseStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_,
    typename std::enable_if<std::is_base_of<
        kindr::Twist<typename ValueType_::Scalar, typename ValueType_::PositionDiff, typename ValueType_::RotationDiff>,
        ValueType_>::value>::type> {
  using msgtype = geometry_msgs::TwistStamped;
};
/*******

/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  using msgtype = kindr_msgs::VectorAtPosition;
};
/***************************************************/
#endif

// generic interface
template <typename ValueType_, typename Enable_ = void>
struct slr_update_traits;

/*******************************
 * Specializations: core types *
 *******************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_arithmetic<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};
/********************************/

/*******************************
 * Specializations: enum types *
 *******************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    typename std::underlying_type<ValueType_>::type vectorPtr =
        static_cast<typename std::underlying_type<ValueType_>::type>(*vectorPtr_);
    slr_update_traits<typename std::underlying_type<ValueType_>::type>::updateMsg(&vectorPtr, msg, timeStamp);
  }
};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <>
struct slr_update_traits<signal_logger::TimestampPair> {
  static void updateMsg(const signal_logger::TimestampPair* var,
                        typename slr_msg_traits<signal_logger::TimestampPair>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value.data.sec = var->first;
    msg->value.data.nsec = var->second;
  }
};
/********************************/

/***************************************************
 * Specializations: STL types                *
 ***************************************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_array_type<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    for (int i = 0; i < get_array_size<ValueType_>(); ++i) {
      msg->matrix.data.push_back((*vectorPtr_)[i]);
    }
  }
};
/********************************/

/********************************
 * Specializations: eigen types *
 ********************************/
template <>
struct slr_update_traits<Eigen::Vector3d> {
  static void updateMsg(const Eigen::Vector3d* vectorPtr_,
                        typename slr_msg_traits<Eigen::Vector3d>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->quaternion.w = vectorPtr_->w();
    msg->quaternion.x = vectorPtr_->x();
    msg->quaternion.y = vectorPtr_->y();
    msg->quaternion.z = vectorPtr_->z();
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    msg->matrix.data.push_back(vectorPtr_->angle());
    msg->matrix.data.push_back(vectorPtr_->axis()(0));
    msg->matrix.data.push_back(vectorPtr_->axis()(1));
    msg->matrix.data.push_back(vectorPtr_->axis()(2));
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_,
                         typename std::enable_if<is_eigen_matrix_excluding_vector3<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.data.clear();
    for (int r = 0; r < vectorPtr_->rows(); r++) {
      for (int c = 0; c < vectorPtr_->cols(); c++) {
        msg->matrix.data.push_back((*vectorPtr_)(r, c));
      }
    }
  }
};
/********************************/

#ifdef SILO_USE_KINDR
/********************************
 * Specializations: kindr types *
 ********************************/
//! Trait for Kindr rotations
template <typename ValueType_>
struct slr_update_traits<
    ValueType_, typename std::enable_if<std::is_base_of<kindr::RotationBase<ValueType_>, ValueType_>::value ||
                                        std::is_base_of<kindr::RotationDiffBase<ValueType_>, ValueType_>::value ||
                                        is_kindr_vector<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    slr_update_traits<typename ValueType_::Implementation>::updateMsg(&vectorPtr_->toImplementation(), msg, timeStamp);
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_,
                         typename std::enable_if<is_kindr_homogeneous_transformation<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    const kindr::RotationQuaternion<typename ValueType_::Scalar> orientation(vectorPtr_->getRotation());
    msg->pose.orientation.w = orientation.w();
    msg->pose.orientation.x = orientation.x();
    msg->pose.orientation.y = orientation.y();
    msg->pose.orientation.z = orientation.z();
    msg->pose.position.x = vectorPtr_->getPosition().x();
    msg->pose.position.y = vectorPtr_->getPosition().y();
    msg->pose.position.z = vectorPtr_->getPosition().z();
  }
};

template <typename ValueType_>
struct slr_update_traits<
    ValueType_,
    typename std::enable_if<std::is_base_of<
        kindr::Twist<typename ValueType_::Scalar, typename ValueType_::PositionDiff, typename ValueType_::RotationDiff>,
        ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->twist.linear.x = vectorPtr_->getTranslationalVelocity().x();
    msg->twist.linear.y = vectorPtr_->getTranslationalVelocity().y();
    msg->twist.linear.z = vectorPtr_->getTranslationalVelocity().z();
    msg->twist.angular.x = vectorPtr_->getRotationalVelocity().x();
    msg->twist.angular.y = vectorPtr_->getRotationalVelocity().y();
    msg->twist.angular.z = vectorPtr_->getRotationalVelocity().z();
  }
};

/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_,
                        typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const ros::Time& timeStamp) {
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

  static int getType() {
    if (typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearAccelerationD>))
      return kindr_msgs::VectorAtPosition::TYPE_ACCELERATION;
    if (typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearVelocityD>))
      return kindr_msgs::VectorAtPosition::TYPE_VELOCITY;
    if (typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrForceD>))
      return kindr_msgs::VectorAtPosition::TYPE_FORCE;
    if (typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrTorqueD>))
      return kindr_msgs::VectorAtPosition::TYPE_TORQUE;

    return kindr_msgs::VectorAtPosition::TYPE_TYPELESS;
  }
};
/***************************************************/
#endif

} /* namespace traits */

} /* namespace signal_logger_ros */