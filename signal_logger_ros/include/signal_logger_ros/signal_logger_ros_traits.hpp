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
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>

#include <signal_logger_msgs/EigenMatrixDouble.h>
#include <signal_logger_msgs/EigenMatrixFloat.h>

namespace signal_logger_ros {

namespace traits {

enum VarType {
  Double = 5,
  Float = 6,
  Int = 7,
  Short = 8,
  Long = 9,
  Char = 10,
  Bool = 11,
  KindrPositionType = 12,
  KindrRotationQuaternionType = 13,
  KindrEulerAnglesZyxType = 14,
  KindrLocalAngularVelocityType = 15,
  KindrAngleAxis = 16,
  KindrRotationMatrixType = 17,
  KindrRotationVectorType = 18,
  KindrLinearVelocityType = 19,
  KindrLinearAccelerationType = 20,
  KindrAngularAccelerationType = 21,
  KindrForceType = 22,
  KindrTorqueType = 23,
  KindrVectorType = 24,
  KindrVectorAtPositionType = 25,
  KindrForceAtPositionType = 34,
  KindrTorqueAtPositionType = 35,
  MatrixDouble = 26,
  MatrixFloat =27,
  MatrixInt = 28,
  MatrixShort = 29,
  MatrixLong = 30,
  MatrixChar = 31,
  MatrixUnsignedChar = 36,
  MatrixBool = 32,
  EigenVector = 33,
  KindrTypeNone = -1
};

enum Frames {
  Base = 0,
  Map = 1
};

// generic interface
template<typename LogType_> struct slr_traits;
//template<typename LogType_> struct slr_traits_type;

/*******************
 * specializations *
 *******************/
template<>
struct slr_traits<double> {
  typedef std_msgs::Float32 msgtype;
  static const VarType varType = VarType::Double;
  static void updateMsg(double* vectorPtr_, msgtype& msg) {
      msg.data = *vectorPtr_;
    }
};

template<>
struct slr_traits<float> {
  typedef std_msgs::Float32 msgtype;
  static const VarType varType = VarType::Float;
  static void updateMsg(float* vectorPtr_, msgtype& msg) {
    msg.data = *vectorPtr_;
  }
};

template<>
struct slr_traits<signal_logger::LoggerBase::KindrForceD> {
  typedef geometry_msgs::Vector3Stamped msgtype;
  static const VarType varType = VarType::KindrForceType;
};

template<>
struct slr_traits<signal_logger::LoggerBase::KindrPositionD> {
  typedef geometry_msgs::Vector3Stamped msgtype;
  static const VarType varType = VarType::KindrPositionType;
};


//template<>
//struct slr_traits<VarType::Double> {
//  typedef std_msgs::Float64 msgtype;
//};
//
//template<>
//struct slr_traits<VarType::Float> {
//  typedef std_msgs::Float32 msgtype;
//};
//
//template<>
//struct slr_traits<VarType::Int> {
//  typedef std_msgs::Int32 msgtype;
//};
//
//template<>
//struct slr_traits<VarType::Short> {
//  typedef std_msgs::Int8 msgtype;
//};
//
//template<>
//struct slr_traits<VarType::Long> {
//  typedef std_msgs::Int64 msgtype;
//};
//
//template<>
//struct slr_traits<VarType::Char> {
//  typedef std_msgs::Char msgtype;
//};
//
//template<>
//struct slr_traits<VarType::Bool> {
//  typedef std_msgs::Bool msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrPositionType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrRotationQuaternionType> {
//  typedef geometry_msgs::QuaternionStamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrEulerAnglesZyxType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrLocalAngularVelocityType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::KindrAngleAxis> {
//
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::KindrRotationMatrixType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrRotationVectorType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrLinearVelocityType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrLinearAccelerationType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrAngularAccelerationType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrForceType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrTorqueType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrVectorType> {
//  typedef geometry_msgs::Vector3Stamped msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrVectorAtPositionType> {
//  typedef kindr_msgs::VectorAtPosition msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrForceAtPositionType> {
//  typedef kindr_msgs::VectorAtPosition msgtype;
//};
//
//template<>
//struct slr_traits<VarType::KindrTorqueAtPositionType> {
//  typedef kindr_msgs::VectorAtPosition msgtype;
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::MatrixDouble> {
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::MatrixFloat> {
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::MatrixInt> {
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::MatrixShort> {
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::MatrixLong> {
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::MatrixChar> {
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::MatrixUnsignedChar> {
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::MatrixBool> {
//};
//
//// todo: define message type
//template<>
//struct slr_traits<VarType::EigenVector> {
//};
/*******************/

} /* namespace traits */

} /* namespace signal_logger_ros */

#endif /* SIGNAL_LOGGER_ROS_TRAITS_HPP_ */
