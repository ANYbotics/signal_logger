/*!
 * @file     signal_logger_ros_traits.hpp
 * @author   C. Dario Bellicoso, Gabriel Hottiger, Christian Gehring
 * @date     Feb 21, 2015
 * @brief    Message update traits for all supported types.
 */

#pragma once

#include <signal_logger_core/LogElementTypes.hpp>
#include <signal_logger_core/signal_logger_traits.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <std_msgs/msg/float32.hpp>

#include <signal_logger_msgs/msg/time_stamped.hpp>

#include <signal_logger_msgs/msg/bool_multi_array_stamped.hpp>
#include <signal_logger_msgs/msg/float32_multi_array_stamped.hpp>
#include <signal_logger_msgs/msg/float64_multi_array_stamped.hpp>
#include <signal_logger_msgs/msg/int16_multi_array_stamped.hpp>
#include <signal_logger_msgs/msg/int32_multi_array_stamped.hpp>
#include <signal_logger_msgs/msg/int64_multi_array_stamped.hpp>
#include <signal_logger_msgs/msg/int8_multi_array_stamped.hpp>

#include <signal_logger_msgs/msg/bool_stamped.hpp>
#include <signal_logger_msgs/msg/char_stamped.hpp>
#include <signal_logger_msgs/msg/float32_stamped.hpp>
#include <signal_logger_msgs/msg/float64_stamped.hpp>
#include <signal_logger_msgs/msg/int16_stamped.hpp>
#include <signal_logger_msgs/msg/int32_stamped.hpp>
#include <signal_logger_msgs/msg/int64_stamped.hpp>
#include <signal_logger_msgs/msg/int8_stamped.hpp>
#include <signal_logger_msgs/msg/map_int_double_stamped.hpp>
#include <signal_logger_msgs/msg/map_string_double_stamped.hpp>
#include <signal_logger_msgs/msg/map_string_int_stamped.hpp>
#include <signal_logger_msgs/msg/pair_int_double.hpp>
#include <signal_logger_msgs/msg/pair_int_double_stamped.hpp>
#include <signal_logger_msgs/msg/pair_string_double.hpp>
#include <signal_logger_msgs/msg/pair_string_double_stamped.hpp>
#include <signal_logger_msgs/msg/pair_string_int.hpp>
#include <signal_logger_msgs/msg/pair_string_int_stamped.hpp>
#include <signal_logger_msgs/msg/string_stamped.hpp>
#include <signal_logger_msgs/msg/u_int16_stamped.hpp>
#include <signal_logger_msgs/msg/u_int32_stamped.hpp>
#include <signal_logger_msgs/msg/u_int64_stamped.hpp>
#include <signal_logger_msgs/msg/u_int8_stamped.hpp>
#include <signal_logger_msgs/msg/unsigned_char_stamped.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <kindr_msgs/msg/vector_at_position.hpp>

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
  using msgtype = signal_logger_msgs::msg::Float64Stamped;
};

template <>
struct slr_msg_traits<float> {
  using msgtype = signal_logger_msgs::msg::Float32Stamped;
};

template <>
struct slr_msg_traits<bool> {
  using msgtype = signal_logger_msgs::msg::BoolStamped;
};

template <>
struct slr_msg_traits<char> {
  using msgtype = signal_logger_msgs::msg::CharStamped;
};

template <>
struct slr_msg_traits<signed char> {
  using msgtype = signal_logger_msgs::msg::Int8Stamped;
};

template <>
struct slr_msg_traits<unsigned char> {
  using msgtype = signal_logger_msgs::msg::UnsignedCharStamped;
};

template <>
struct slr_msg_traits<short> {
  using msgtype = signal_logger_msgs::msg::Int16Stamped;
};

template <>
struct slr_msg_traits<unsigned short> {
  using msgtype = signal_logger_msgs::msg::UInt16Stamped;
};

template <>
struct slr_msg_traits<int> {
  using msgtype = signal_logger_msgs::msg::Int32Stamped;
};

template <>
struct slr_msg_traits<unsigned int> {
  using msgtype = signal_logger_msgs::msg::UInt32Stamped;
};

template <>
struct slr_msg_traits<long> {
  using msgtype = signal_logger_msgs::msg::Int64Stamped;
};

template <>
struct slr_msg_traits<unsigned long> {
  using msgtype = signal_logger_msgs::msg::UInt64Stamped;
};

/////////////////////////////////////////

/*******************************
 * Specializations: enum types *
 *******************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type> {
  using msgtype = typename slr_msg_traits<typename std::underlying_type<ValueType_>::type>::msgtype;
};
/////////////////////////////////////////

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <>
struct slr_msg_traits<signal_logger::TimestampPair> {
  using msgtype = signal_logger_msgs::msg::TimeStamped;
};
/////////////////////////////////////////

/***************************************************
 * Specializations: STL types                      *
 ***************************************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_same<ValueType_, std::string>::value>::type> {
  using msgtype = signal_logger_msgs::msg::StringStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_pair_of<ValueType_, std::string, double>::value>::type> {
  using msgtype = signal_logger_msgs::msg::PairStringDoubleStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_pair_of<ValueType_, std::string, int>::value>::type> {
  using msgtype = signal_logger_msgs::msg::PairStringIntStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_pair_of<ValueType_, int, double>::value>::type> {
  using msgtype = signal_logger_msgs::msg::PairIntDoubleStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_, typename std::enable_if<is_container<ValueType_>::value && std::is_same<double, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Float64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_, typename std::enable_if<is_container<ValueType_>::value && std::is_same<float, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Float32MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_, typename std::enable_if<is_container<ValueType_>::value && std::is_same<long, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_, typename std::enable_if<is_container<ValueType_>::value && std::is_same<int, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int32MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_, typename std::enable_if<is_container<ValueType_>::value && std::is_same<short, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int16MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_, typename std::enable_if<is_container<ValueType_>::value && std::is_same<char, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int8MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<
    ValueType_, typename std::enable_if<is_container<ValueType_>::value && std::is_same<bool, element_type_t<ValueType_>>::value>::type> {
  using msgtype = signal_logger_msgs::msg::BoolMultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_container<ValueType_>::value &&
                                                          is_pair_of<element_type_t<ValueType_>, const std::string, double>::value>::type> {
  using msgtype = signal_logger_msgs::msg::MapStringDoubleStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_container<ValueType_>::value &&
                                                          is_pair_of<element_type_t<ValueType_>, const std::string, int>::value>::type> {
  using msgtype = signal_logger_msgs::msg::MapStringIntStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_container<ValueType_>::value &&
                                                          is_pair_of<element_type_t<ValueType_>, const int, double>::value>::type> {
  using msgtype = signal_logger_msgs::msg::MapIntDoubleStamped;
};

/////////////////////////////////////////

/********************************
 * Specializations: eigen types *
 ********************************/
template <>
struct slr_msg_traits<Eigen::Vector3d> {
  using msgtype = geometry_msgs::msg::Vector3Stamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type> {
  using msgtype = geometry_msgs::msg::QuaternionStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Float64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar_excluding_vector3<ValueType_, double>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Float64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, float>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Float32MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, long>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int64MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, int>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int32MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, short>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int16MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, char>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int8MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, unsigned char>::value>::type> {
  using msgtype = signal_logger_msgs::msg::Int8MultiArrayStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_eigen_matrix_of_scalar<ValueType_, bool>::value>::type> {
  using msgtype = signal_logger_msgs::msg::BoolMultiArrayStamped;
};
/////////////////////////////////////////

/********************************
 * Specializations: kindr types *
 ********************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_vector<ValueType_>::value>::type> {
  using msgtype = typename slr_msg_traits<typename ValueType_::Implementation>::msgtype;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::RotationBase<ValueType_>, ValueType_>::value ||
                                                          std::is_base_of<kindr::RotationDiffBase<ValueType_>, ValueType_>::value>::type> {
  using msgtype = typename slr_msg_traits<typename ValueType_::Implementation>::msgtype;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_homogeneous_transformation<ValueType_>::value>::type> {
  using msgtype = geometry_msgs::msg::PoseStamped;
};

template <typename ValueType_>
struct slr_msg_traits<ValueType_,
                      typename std::enable_if<std::is_base_of<
                          kindr::Twist<typename ValueType_::Scalar, typename ValueType_::PositionDiff, typename ValueType_::RotationDiff>,
                          ValueType_>::value>::type> {
  using msgtype = geometry_msgs::msg::TwistStamped;
};
/////////////////////////////////////////

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_>
struct slr_msg_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  using msgtype = kindr_msgs::msg::VectorAtPosition;
};
////////////////////////////////////////////////////

// generic interface
template <typename ValueType_, typename Enable_ = void>
struct slr_update_traits;

/*******************************
 * Specializations: core types *
 *******************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_arithmetic<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};
/////////////////////////////////

/*******************************
 * Specializations: enum types *
 *******************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    typename std::underlying_type<ValueType_>::type vectorPtr = static_cast<typename std::underlying_type<ValueType_>::type>(*vectorPtr_);
    slr_update_traits<typename std::underlying_type<ValueType_>::type>::updateMsg(&vectorPtr, msg, timeStamp);
  }
};
////////////////////////////////

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <>
struct slr_update_traits<signal_logger::TimestampPair> {
  static void updateMsg(const signal_logger::TimestampPair* var, typename slr_msg_traits<signal_logger::TimestampPair>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value.sec = var->first;
    msg->value.nanosec = var->second;
  }
};
/********************************/

/***************************************************
 * Specializations: STL types                *
 ***************************************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_same<ValueType_, std::string>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->value = *vectorPtr_;
  }
};

template <typename ValueType_>
struct slr_update_traits<
    ValueType_, typename std::enable_if<is_container<ValueType_>::value && std::is_arithmetic<element_type_t<ValueType_>>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.layout.dim.clear();
    msg->matrix.layout.dim.resize(1);
    msg->matrix.layout.dim[0].label = "items";
    msg->matrix.layout.dim[0].size = vectorPtr_->size();
    msg->matrix.layout.dim[0].stride = vectorPtr_->size();
    msg->matrix.layout.data_offset = 0;
    msg->matrix.data.clear();
    for (auto&& v : *vectorPtr_) {
      msg->matrix.data.push_back(v);
    }
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_,
                         typename std::enable_if<is_container<ValueType_>::value && is_pair<element_type_t<ValueType_>>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->pairs.clear();
    using PairMsgsVector = decltype(msg->pairs);
    typename PairMsgsVector::value_type pair;
    for (auto&& v : *vectorPtr_) {
      pair.first = v.first;
      pair.second = v.second;
      msg->pairs.push_back(pair);
    }
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_pair<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->first = vectorPtr_->first;
    msg->second = vectorPtr_->second;
  }
};

/////////////////////////////////////////

/********************************
 * Specializations: eigen types *
 ********************************/
template <>
struct slr_update_traits<Eigen::Vector3d> {
  static void updateMsg(const Eigen::Vector3d* vectorPtr_, typename slr_msg_traits<Eigen::Vector3d>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->vector.x = vectorPtr_->x();
    msg->vector.y = vectorPtr_->y();
    msg->vector.z = vectorPtr_->z();
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->quaternion.w = vectorPtr_->w();
    msg->quaternion.x = vectorPtr_->x();
    msg->quaternion.y = vectorPtr_->y();
    msg->quaternion.z = vectorPtr_->z();
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.layout.dim.resize(1);
    msg->matrix.layout.dim[0].label = "angleAxis";
    msg->matrix.layout.dim[0].size = vectorPtr_->size();
    msg->matrix.layout.dim[0].stride = vectorPtr_->size();
    msg->matrix.layout.data_offset = 0;
    msg->matrix.data.clear();
    msg->matrix.data.push_back(vectorPtr_->angle());
    msg->matrix.data.push_back(vectorPtr_->axis()(0));
    msg->matrix.data.push_back(vectorPtr_->axis()(1));
    msg->matrix.data.push_back(vectorPtr_->axis()(2));
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_eigen_matrix_excluding_vector3<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->matrix.layout.dim.resize(2);
    msg->matrix.layout.dim[0].label = "row";
    msg->matrix.layout.dim[0].size = vectorPtr_->rows();
    msg->matrix.layout.dim[0].stride = vectorPtr_->rows() * vectorPtr_->cols();
    msg->matrix.layout.dim[1].label = "col";
    msg->matrix.layout.dim[1].size = vectorPtr_->cols();
    msg->matrix.layout.dim[1].stride = vectorPtr_->cols();
    msg->matrix.layout.data_offset = 0;
    msg->matrix.data.clear();
    for (int r = 0; r < vectorPtr_->rows(); r++) {
      for (int c = 0; c < vectorPtr_->cols(); c++) {
        msg->matrix.data.push_back((*vectorPtr_)(r, c));
      }
    }
  }
};
/////////////////////////////////////////

/********************************
 * Specializations: kindr types *
 ********************************/
//! Trait for Kindr rotations
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::RotationBase<ValueType_>, ValueType_>::value ||
                                                             std::is_base_of<kindr::RotationDiffBase<ValueType_>, ValueType_>::value ||
                                                             is_kindr_vector<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    slr_update_traits<typename ValueType_::Implementation>::updateMsg(&vectorPtr_->toImplementation(), msg, timeStamp);
  }
};

template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_kindr_homogeneous_transformation<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
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
    ValueType_, typename std::enable_if<std::is_base_of<
                    kindr::Twist<typename ValueType_::Scalar, typename ValueType_::PositionDiff, typename ValueType_::RotationDiff>,
                    ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
    msg->header.stamp = timeStamp;
    msg->twist.linear.x = vectorPtr_->getTranslationalVelocity().x();
    msg->twist.linear.y = vectorPtr_->getTranslationalVelocity().y();
    msg->twist.linear.z = vectorPtr_->getTranslationalVelocity().z();
    msg->twist.angular.x = vectorPtr_->getRotationalVelocity().x();
    msg->twist.angular.y = vectorPtr_->getRotationalVelocity().y();
    msg->twist.angular.z = vectorPtr_->getRotationalVelocity().z();
  }
};

/////////////////////////////////////////

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_>
struct slr_update_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  static void updateMsg(const ValueType_* vectorPtr_, typename slr_msg_traits<ValueType_>::msgtype* const msg,
                        const builtin_interfaces::msg::Time& timeStamp) {
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
      return kindr_msgs::msg::VectorAtPosition::TYPE_ACCELERATION;
    if (typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearVelocityD>))
      return kindr_msgs::msg::VectorAtPosition::TYPE_VELOCITY;
    if (typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrForceD>))
      return kindr_msgs::msg::VectorAtPosition::TYPE_FORCE;
    if (typeid(ValueType_) == typeid(signal_logger::KindrVectorAtPosition<signal_logger::KindrTorqueD>))
      return kindr_msgs::msg::VectorAtPosition::TYPE_TORQUE;

    return kindr_msgs::msg::VectorAtPosition::TYPE_TYPELESS;
  }
};
/////////////////////////////////////////

} /* namespace traits */

} /* namespace signal_logger_ros */
