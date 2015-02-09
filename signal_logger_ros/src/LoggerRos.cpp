/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014,  Christian Gehring, Michael Bloesch,
 * Peter Fankhauser, C. Dario Bellicoso
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * LoggerRos.cpp
 *
 *  Created on: Jan 27, 2015
 *      Author: C. Dario Bellicoso
 */

#include "signal_logger_ros/LoggerRos.hpp"
#include "roco/log/log_messages.hpp"

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


namespace signal_logger_ros {

LoggerRos::LoggerRos(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle),
    collectedVars_(0)
{

}


LoggerRos::~LoggerRos()
{
  for (auto& elem : collectedVars_) {
    elem.pub_.shutdown();
  }
}


void LoggerRos::initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName) { }
void LoggerRos::startLogger() { }
void LoggerRos::stopLogger() { }
void LoggerRos::saveLoggerData() { }


void LoggerRos::updateLogger(bool updateScript) {

}


void LoggerRos::collectLoggerData() {

  ros::Time stamp = ros::Time::now();

  for (auto elem : collectedVars_) {

    if (elem.pub_.getNumSubscribers() > 0u) {
      switch (elem.type_) {
        /**************
         * Core types *
         **************/
        case(Double): {
          std_msgs::Float64Ptr msg(new std_msgs::Float64);
          double* var = boost::any_cast<double*>(elem.vectorPtr_);
          msg->data = *var;
          elem.pub_.publish(std_msgs::Float64ConstPtr(msg));
        } break;

        case(Float): {
          std_msgs::Float32Ptr msg(new std_msgs::Float32);
          float* var = boost::any_cast<float*>(elem.vectorPtr_);
          msg->data = *var;
          elem.pub_.publish(std_msgs::Float32ConstPtr(msg));
        } break;

        case(Int): {
          std_msgs::Int32Ptr msg(new std_msgs::Int32);
          int* var = boost::any_cast<int*>(elem.vectorPtr_);
          msg->data = *var;
          elem.pub_.publish(std_msgs::Int32ConstPtr(msg));
        } break;

        case(Short): {
          std_msgs::Int8Ptr msg(new std_msgs::Int8);
          int* var = boost::any_cast<int*>(elem.vectorPtr_);
          msg->data = *var;
          elem.pub_.publish(std_msgs::Int8ConstPtr(msg));
        } break;

        case(Long): {
          std_msgs::Int64Ptr msg(new std_msgs::Int64);
          int* var = boost::any_cast<int*>(elem.vectorPtr_);
          msg->data = *var;
          elem.pub_.publish(std_msgs::Int64ConstPtr(msg));
        } break;

        case(Char): {
          std_msgs::CharPtr msg(new std_msgs::Char);
          char* var = boost::any_cast<char*>(elem.vectorPtr_);
          msg->data = *var;
          elem.pub_.publish(std_msgs::CharConstPtr(msg));
        } break;

        case(Bool): {
          std_msgs::BoolPtr msg(new std_msgs::Bool);
          bool* var = boost::any_cast<bool*>(elem.vectorPtr_);
          msg->data = *var;
          elem.pub_.publish(std_msgs::BoolConstPtr(msg));
        } break;
        /**************/

        /****************
         * Matrix types *
         ****************/
        case(MatrixDouble): {
          std_msgs::Float64MultiArrayPtr msg(new std_msgs::Float64MultiArray);
          const Eigen::Ref<Eigen::MatrixXd>& var = boost::any_cast<const Eigen::Ref<Eigen::MatrixXd>&>(elem.vectorPtr_);
          for (int k=0; k<var.cols(); k++) {
            for (int h=0; h<var.rows(); h++) {
              msg->data.push_back((double)(var)(h,k));
            }
          }
          elem.pub_.publish(std_msgs::Float64MultiArrayConstPtr(msg));
        } break;

        case(MatrixFloat): {
          std_msgs::Float32MultiArrayPtr msg(new std_msgs::Float32MultiArray);
          const Eigen::Ref<Eigen::MatrixXf>& var = boost::any_cast<const Eigen::Ref<Eigen::MatrixXf>&>(elem.vectorPtr_);
          for (int k=0; k<var.cols(); k++) {
            for (int h=0; h<var.rows(); h++) {
              msg->data.push_back((float)(var)(h,k));
            }
          }
          elem.pub_.publish(std_msgs::Float32MultiArrayConstPtr(msg));

        } break;

        case(EigenVector): {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const Eigen::Ref<Eigen::Vector3d>& var = boost::any_cast<const Eigen::Ref<Eigen::Vector3d>&>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = var.x();
          msg->vector.y = var.y();
          msg->vector.z = var.z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;
        /****************/

        /***************
         * Kindr types *
         ***************/
        case(KindrPositionType) : {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrPositionD* vec = boost::any_cast<const KindrPositionD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrRotationQuaternionType) : {
          geometry_msgs::QuaternionStampedPtr msg(new geometry_msgs::QuaternionStamped);
          const KindrRotationQuaternionD* vec = boost::any_cast<const KindrRotationQuaternionD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->quaternion.x = vec->x();
          msg->quaternion.y = vec->y();
          msg->quaternion.z = vec->z();
          msg->quaternion.w = vec->w();
          elem.pub_.publish(geometry_msgs::QuaternionStampedConstPtr(msg));
        } break;

        case(KindrEulerAnglesZyxType) : {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrEulerAnglesZyxD* vec = boost::any_cast<const KindrEulerAnglesZyxD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrLocalAngularVelocityType) : {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrAngularVelocityD* vec = boost::any_cast<const KindrAngularVelocityD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrAngleAxis) : {

        } break;

        case(KindrRotationMatrixType) : {

        } break;

        case(KindrRotationVectorType) : {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrRotationVectorD* vec = boost::any_cast<const KindrRotationVectorD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrLinearVelocityType) : {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrLinearVelocityD* vec = boost::any_cast<const KindrLinearVelocityD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrLinearAccelerationType) : {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrLinearAccelerationD* vec = boost::any_cast<const KindrLinearAccelerationD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrAngularAccelerationType) : {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrAngularAccelerationD* vec = boost::any_cast<const KindrAngularAccelerationD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrForceType): {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrForceD* vec = boost::any_cast<const KindrForceD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrTorqueType): {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrTorqueD* vec = boost::any_cast<const KindrTorqueD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrVectorType): {
          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
          const KindrVectorD* vec = boost::any_cast<const KindrVectorD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
        } break;

        case(KindrVectorAtPositionType): {
          kindr_msgs::VectorAtPositionPtr msg(new kindr_msgs::VectorAtPosition);
          const KindrVectorD* vec = boost::any_cast<const KindrVectorD*>(elem.vectorPtr_);
          msg->header.stamp = stamp;
          msg->vector.x = vec->x();
          msg->vector.y = vec->y();
          msg->vector.z = vec->z();
          msg->position.x = elem.positionPtr_->x();
          msg->position.y = elem.positionPtr_->y();
          msg->position.z = elem.positionPtr_->z();
          msg->name = "";
          msg->type = elem.kindrMsg_.type;
          elem.pub_.publish(kindr_msgs::VectorAtPositionConstPtr(msg));
        } break;
        /***************/

        case(KindrTypeNone) : {

        } break;

        default: ROCO_ERROR("[LoggerRos::collectLoggerData] Unhandled collected variable type.") break;
      } // switch

    } // if subscribers > 0

  }

}


void LoggerRos::lockUpdate() { }
void LoggerRos::stopAndSaveLoggerData() { }


void LoggerRos::addFloatToLog(float* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Float32>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::Float;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addDoubleToLog(double* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Float64>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::Double;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addIntToLog(int* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int32>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::Int;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}


void LoggerRos::addShortToLog(short* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int8>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::Int;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}


void LoggerRos::addLongToLog(long* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int64>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::Int;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}


void LoggerRos::addCharToLog(char* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Char>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::Char;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addBoolToLog(bool* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Bool>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::Bool;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}


/******************
 * Eigen wrappers *
 ******************/
void LoggerRos::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::MatrixDouble;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Float32MultiArray>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::MatrixFloat;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int32MultiArray>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::MatrixInt;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int8MultiArray>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::MatrixShort;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int64MultiArray>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::MatrixDouble;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var,          const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerRos::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerRos::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var,          const std::string& name, const std::string& group, const std::string& unit, bool update) { }

void LoggerRos::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  const std::string& topicName = group + name;
  std::vector<LoggerVarInfo>::iterator collectedIterator;
  if (checkIfVarCollected(topicName, collectedIterator)) {
    collectedIterator->vectorPtr_ = var;
  } else {
    LoggerVarInfo varInfo(topicName);
    varInfo.pub_ = nodeHandle_.advertise<geometry_msgs::Vector3Stamped>(topicName, 100);
    varInfo.type_ = LoggerRos::VarType::EigenVector;
    varInfo.vectorPtr_ = var;
    collectedVars_.push_back(varInfo);
  }
}

void LoggerRos::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addDoubleToLog((double *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addFloatToLog((float *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addIntToLog((int *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addShortToLog((short *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addLongToLog((long *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addCharToLog((char *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addCharToLog((char *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addBoolToLog((bool *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog((double *)(&var(0)), static_cast<std::string>(names(0,0)), group, unit, update);
  addDoubleToLog((double *)(&var(1)), static_cast<std::string>(names(1,0)), group, unit, update);
  addDoubleToLog((double *)(&var(2)), static_cast<std::string>(names(2,0)), group, unit, update);
}
/******************/


/******************
 * Kindr wrappers *
 ******************/
void LoggerRos::addDoubleKindrPositionToLog(const KindrPositionD& position, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrPositionType, &position);
}

void LoggerRos::addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation,  const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrRotationQuaternionType, &rotation);
}

void LoggerRos::addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrEulerAnglesZyxType, &rotation);
}

void LoggerRos::addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrLocalAngularVelocityType, &angVel);
}

void LoggerRos::addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis,                   const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerRos::addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat,            const std::string& name, const std::string& group, const std::string& unit, bool update) { }

void LoggerRos::addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrRotationVectorType, &rotVec);
}

void LoggerRos::addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrLinearVelocityType, &linVel);
}

void LoggerRos::addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrLinearAccelerationType, &linAcc);
}

void LoggerRos::addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrAngularAccelerationType, &angAcc);
}

void LoggerRos::addDoubleKindrForceToLog(const KindrForceD& force, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrForceType, &force);
}

void LoggerRos::addDoubleKindrTorqueToLog(const KindrTorqueD& torque, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrTorqueType, &torque);
}

void LoggerRos::addDoubleKindrVectorToLog(const KindrVectorD& vector, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addKindr3DToCollectedVariables(group+name, LoggerRos::VarType::KindrVectorType, &vector);
}

void LoggerRos::addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector, const KindrPositionD& position, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  kindr_msgs::VectorAtPosition msg;
  msg.type = kindr_msgs::VectorAtPosition::TYPE_TYPELESS;
  addKindr3DVectorAtPositionToCollectedVariables(group+name,
                                                 msg,
                                                 &vector,
                                                 &position);
}
/******************/


bool LoggerRos::checkIfVarCollected(const std::string& topicName, std::vector<LoggerVarInfo>::iterator& it) {
  for (it = collectedVars_.begin(); it != collectedVars_.end(); it++) {
    if (it->topicName_.compare(topicName) == 0) {
      ROCO_WARN_STREAM("[LoggerRos::checkIfVarCollected] Topic '" << topicName << "' was already published.");
      return true;
    }
  }
  return false;
}


} /* namespace signal_logger_ros */
