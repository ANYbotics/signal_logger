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


namespace signal_logger_ros {

LoggerRos::LoggerRos(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle),
    collectedVars_(0)
{

}


LoggerRos::~LoggerRos()
{
//  for (auto& elem : collectedVars_) {
//    elem.pub_.shutdown();
//  }
}


void LoggerRos::initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName) { }
void LoggerRos::startLogger() { }
void LoggerRos::stopLogger() { }
void LoggerRos::saveLoggerData() { }


void LoggerRos::updateLogger(bool updateScript) {

}


void LoggerRos::collectLoggerData() {

  ros::Time stamp = ros::Time::now();

  for (auto& elem : collectedVars_) {
    elem->publish();

//    if (elem.pub_.getNumSubscribers() > 0u) {
//      switch (elem.type_) {
//        /**************
//         * Core types *
//         **************/
//        case(Double): {
////          traits::slr_traits<Double>::msgtype msg;
//////          traits::slr_traits_type<double>::msgtype msg;
//////          std_msgs::Float64 msg;
////          double* var = boost::any_cast<double*>(elem.vectorPtr_);
////          realtime_tools::RealtimePublisher<std_msgs::Float64>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<std_msgs::Float64>*>(elem.rtPub_);
////          msg.data = *var;
////          if (rtPub->trylock()) {
////            rtPub->msg_ = msg;
////            rtPub->unlockAndPublish();
////          }
//        } break;
//
//        case(Float): {
////          traits::slr_traits<Float>::msgtype msg;
////          std_msgs::Float32 msg;
////          float* var = boost::any_cast<float*>(elem.vectorPtr_);
////          realtime_tools::RealtimePublisher<std_msgs::Float32>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<std_msgs::Float32>*>(elem.rtPub_);
////          msg.data = *var;
////          if (rtPub->trylock()) {
////            rtPub->msg_ = msg;
////            rtPub->unlockAndPublish();
////          }
//        } break;
//
//        case(Int): {
////          std_msgs::Int32 msg;
////          int* var = boost::any_cast<int*>(elem.vectorPtr_);
////          realtime_tools::RealtimePublisher<std_msgs::Int32>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<std_msgs::Int32>*>(elem.rtPub_);
////          msg.data = *var;
////          if (rtPub->trylock()) {
////            rtPub->msg_ = msg;
////            rtPub->unlockAndPublish();
////          }
//        } break;
//
//        case(Short): {
////          std_msgs::Int8 msg;
////          short* var = boost::any_cast<short*>(elem.vectorPtr_);
////          realtime_tools::RealtimePublisher<std_msgs::Int8>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<std_msgs::Int8>*>(elem.rtPub_);
////          msg.data = *var;
////          if (rtPub->trylock()) {
////            rtPub->msg_ = msg;
////            rtPub->unlockAndPublish();
////          }
//        } break;
//
//        case(Long): {
//          std_msgs::Int64Ptr msg(new std_msgs::Int64);
//          int* var = boost::any_cast<int*>(elem.vectorPtr_);
//          msg->data = *var;
//          elem.pub_.publish(std_msgs::Int64ConstPtr(msg));
//        } break;
//
//        case(Char): {
//          std_msgs::CharPtr msg(new std_msgs::Char);
//          char* var = boost::any_cast<char*>(elem.vectorPtr_);
//          msg->data = *var;
//          elem.pub_.publish(std_msgs::CharConstPtr(msg));
//        } break;
//
//        case(Bool): {
//          std_msgs::BoolPtr msg(new std_msgs::Bool);
//          bool* var = boost::any_cast<bool*>(elem.vectorPtr_);
//          msg->data = *var;
//          elem.pub_.publish(std_msgs::BoolConstPtr(msg));
//        } break;
//        /**************/
//
//        /****************
//         * Matrix types *
//         ****************/
//        case(MatrixDouble): {
////          signal_logger_msgs::EigenMatrixDoublePtr msg(new signal_logger_msgs::EigenMatrixDouble);
////          const Eigen::Ref<Eigen::MatrixXd>& var = boost::any_cast<const Eigen::Ref<Eigen::MatrixXd>&>(elem.vectorPtr_);
////          msg->header.stamp = stamp;
////          for (int k=0; k<var.cols(); k++) {
////            for (int h=0; h<var.rows(); h++) {
////              msg->matrix.push_back((double)(var)(h,k));
////            }
////          }
//
////          if (elem.rtPub_.trylock()){
////            elem.rtPub_.msg_.header.stamp = stamp;
////            for (int k=0; k<var.cols(); k++) {
////              for (int h=0; h<var.rows(); h++) {
////                elem.rtPub_.msg.matrix.push_back((double)(var)(h,k));
////              }
////            }
////            elem.rtPub_.unlockAndPublish();
////           }
//
////          elem.pub_.publish(signal_logger_msgs::EigenMatrixDoubleConstPtr(msg));
//        } break;
//
//        case(MatrixFloat): {
//          std_msgs::Float32MultiArrayPtr msg(new std_msgs::Float32MultiArray);
//          const Eigen::Ref<Eigen::MatrixXf>& var = boost::any_cast<const Eigen::Ref<Eigen::MatrixXf>&>(elem.vectorPtr_);
//          for (int k=0; k<var.cols(); k++) {
//            for (int h=0; h<var.rows(); h++) {
//              msg->data.push_back((float)(var)(h,k));
//            }
//          }
//          elem.pub_.publish(std_msgs::Float32MultiArrayConstPtr(msg));
//
//        } break;
//
//        case(MatrixInt): {
//        // todo
//        } break;
//
//        case(MatrixShort): {
//        // todo
//        } break;
//
//        case(MatrixLong): {
//        // todo
//        } break;
//
//        case(MatrixChar): {
//          // todo
//        } break;
//
//        case(MatrixUnsignedChar): {
//           // todo
//        } break;
//
//        case(MatrixBool): {
//          // todo
//        } break;
//
//        case(EigenVector): {
//          geometry_msgs::Vector3StampedPtr msg(new geometry_msgs::Vector3Stamped);
//          const Eigen::Ref<Eigen::Vector3d>& var = boost::any_cast<const Eigen::Ref<Eigen::Vector3d>&>(elem.vectorPtr_);
//          msg->header.stamp = stamp;
//          msg->vector.x = var.x();
//          msg->vector.y = var.y();
//          msg->vector.z = var.z();
//          elem.pub_.publish(geometry_msgs::Vector3StampedConstPtr(msg));
//        } break;
//        /****************/
//
//        /***************
//         * Kindr types *
//         ***************/
//        case(KindrPositionType) : {
//          geometry_msgs::Vector3Stamped msg;
//          const KindrPositionD* vec = boost::any_cast<const KindrPositionD*>(elem.vectorPtr_);
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//
//        } break;
//
//        case(KindrRotationQuaternionType) : {
//          geometry_msgs::QuaternionStamped msg;
//          realtime_tools::RealtimePublisher<geometry_msgs::QuaternionStamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::QuaternionStamped>*>(elem.rtPub_);
//          const KindrRotationQuaternionD* vec = boost::any_cast<const KindrRotationQuaternionD*>(elem.vectorPtr_);
//          msg.header.stamp = stamp;
//          msg.quaternion.x = vec->x();
//          msg.quaternion.y = vec->y();
//          msg.quaternion.z = vec->z();
//          msg.quaternion.w = vec->w();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrEulerAnglesZyxType) : {
//          geometry_msgs::Vector3Stamped msg;
//          const KindrEulerAnglesZyxD* vec = boost::any_cast<const KindrEulerAnglesZyxD*>(elem.vectorPtr_);
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrLocalAngularVelocityType) : {
//          geometry_msgs::Vector3Stamped msg;
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//          const KindrAngularVelocityD* vec = boost::any_cast<const KindrAngularVelocityD*>(elem.vectorPtr_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrAngleAxis) : {
//        // todo
//        } break;
//
//        case(KindrRotationMatrixType) : {
//        // todo
//        } break;
//
//        case(KindrRotationVectorType) : {
//          geometry_msgs::Vector3Stamped msg;
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//          const KindrRotationVectorD* vec = boost::any_cast<const KindrRotationVectorD*>(elem.vectorPtr_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrLinearVelocityType) : {
//          geometry_msgs::Vector3Stamped msg;
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//          const KindrLinearVelocityD* vec = boost::any_cast<const KindrLinearVelocityD*>(elem.vectorPtr_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrLinearAccelerationType) : {
//          geometry_msgs::Vector3Stamped msg;
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//          const KindrLinearAccelerationD* vec = boost::any_cast<const KindrLinearAccelerationD*>(elem.vectorPtr_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrAngularAccelerationType) : {
//          geometry_msgs::Vector3Stamped msg;
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//          const KindrAngularAccelerationD* vec = boost::any_cast<const KindrAngularAccelerationD*>(elem.vectorPtr_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrForceType): {
//          traits::slr_traits<KindrForceType>::msgtype msg;
//          const KindrForceD* vec = boost::any_cast<const KindrForceD*>(elem.vectorPtr_);
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrTorqueType): {
//          geometry_msgs::Vector3Stamped msg;
//          const KindrTorqueD* vec = boost::any_cast<const KindrTorqueD*>(elem.vectorPtr_);
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrVectorType): {
//          geometry_msgs::Vector3Stamped msg;
//          const KindrVectorD* vec = boost::any_cast<const KindrVectorD*>(elem.vectorPtr_);
//          realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>*>(elem.rtPub_);
//
//          msg.header.stamp = stamp;
//          msg.vector.x = vec->x();
//          msg.vector.y = vec->y();
//          msg.vector.z = vec->z();
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//
//        case(KindrVectorAtPositionType): {
//          kindr_msgs::VectorAtPosition msg;
//          realtime_tools::RealtimePublisher<kindr_msgs::VectorAtPosition>* rtPub = boost::any_cast<realtime_tools::RealtimePublisher<kindr_msgs::VectorAtPosition>*>(elem.rtPub_);
//
//          switch(elem.kindrMsg_.type) {
//            case(kindr_msgs::VectorAtPosition::TYPE_TYPELESS): {
//              const KindrVectorD* vec = boost::any_cast<const KindrVectorD*>(elem.vectorPtr_);
//              msg.vector.x = vec->x();
//              msg.vector.y = vec->y();
//              msg.vector.z = vec->z();
//            } break;
//
//            case(kindr_msgs::VectorAtPosition::TYPE_FORCE): {
//              const KindrForceD* vec = boost::any_cast<const KindrForceD*>(elem.vectorPtr_);
//              msg.vector.x = vec->x();
//              msg.vector.y = vec->y();
//              msg.vector.z = vec->z();
//            } break;
//
//            case(kindr_msgs::VectorAtPosition::TYPE_TORQUE): {
//              const KindrTorqueD* vec = boost::any_cast<const KindrTorqueD*>(elem.vectorPtr_);
//              msg.vector.x = vec->x();
//              msg.vector.y = vec->y();
//              msg.vector.z = vec->z();
//            } break;
//
//            default: {
//              ROCO_WARN_STREAM("[LoggerRos::collectLoggerData] Unhandled kindr vector at position type.");
//            } break;
//          }
//
//          msg.header.stamp = stamp;
//          msg.header.frame_id = elem.kindrMsg_.header.frame_id;
//          msg.position_frame_id = elem.kindrMsg_.position_frame_id;
//          msg.position.x = elem.positionPtr_->x();
//          msg.position.y = elem.positionPtr_->y();
//          msg.position.z = elem.positionPtr_->z();
//          msg.type = elem.kindrMsg_.type;
//
//          if (rtPub->trylock()) {
//            rtPub->msg_ = msg;
//            rtPub->unlockAndPublish();
//          }
//        } break;
//        /***************/
//
//        case(KindrTypeNone) : {
//
//        } break;
//
//        default: ROCO_ERROR("[LoggerRos::collectLoggerData] Unhandled collected variable type.") break;
//      } // switch

//    } // if subscribers > 0

  }

}


void LoggerRos::lockUpdate() { }
void LoggerRos::stopAndSaveLoggerData() { }


void LoggerRos::addFloatToLog(float* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addScalarToCollectedVariables<float, std_msgs::Float32>(group+name, LoggerRos::VarType::Float, var);
}

void LoggerRos::addDoubleToLog(double* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  collectedVars_.push_back(new LogElement<double>(nodeHandle_, group+name, var));
//  addScalarToCollectedVariables<double, std_msgs::Float64>(group+name, LoggerRos::VarType::Double, var);
}

void LoggerRos::addIntToLog(int* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addScalarToCollectedVariables<int, std_msgs::Int32>(group+name, LoggerRos::VarType::Int, var);
}


void LoggerRos::addShortToLog(short* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addScalarToCollectedVariables<short, std_msgs::Int8>(group+name, LoggerRos::VarType::Short, var);
}


void LoggerRos::addLongToLog(long* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int64>(topicName, 100);
//    varInfo.type_ = LoggerRos::VarType::Int;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}


void LoggerRos::addCharToLog(char* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Char>(topicName, 100);
//    varInfo.type_ = LoggerRos::VarType::Char;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}

void LoggerRos::addBoolToLog(bool* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Bool>(topicName, 100);
//    varInfo.type_ = LoggerRos::VarType::Bool;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}


/******************
 * Eigen wrappers *
 ******************/
void LoggerRos::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<signal_logger_msgs::EigenMatrixDouble>(topicName, 100);
////    varInfo.rtPub_ = realtime_tools::RealtimePublisher<signal_logger_msgs::EigenMatrixDouble>(n, topicName, 4);
//    varInfo.type_ = LoggerRos::VarType::MatrixDouble;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}

void LoggerRos::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Float32MultiArray>(topicName, 100);
//    varInfo.type_ = LoggerRos::VarType::MatrixFloat;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}

void LoggerRos::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int32MultiArray>(topicName, 100);
//    varInfo.type_ = LoggerRos::VarType::MatrixInt;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}

void LoggerRos::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int8MultiArray>(topicName, 100);
//    varInfo.type_ = LoggerRos::VarType::MatrixShort;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}

void LoggerRos::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<std_msgs::Int64MultiArray>(topicName, 100);
//    varInfo.type_ = LoggerRos::VarType::MatrixDouble;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}

void LoggerRos::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  // todo: define appropriate ros msg
}

void LoggerRos::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  // todo: define appropriate ros msg
}

void LoggerRos::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  // todo: define appropriate ros msg
}

void LoggerRos::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  const std::string& topicName = group + name;
//  std::vector<LoggerVarInfo>::iterator collectedIterator;
//  if (checkIfVarCollected(topicName, collectedIterator)) {
//    collectedIterator->vectorPtr_ = var;
//  } else {
//    LoggerVarInfo varInfo(topicName);
//    varInfo.pub_ = nodeHandle_.advertise<geometry_msgs::Vector3Stamped>(topicName, 100);
//    varInfo.type_ = LoggerRos::VarType::EigenVector;
//    varInfo.vectorPtr_ = var;
//    collectedVars_.push_back(varInfo);
//  }
}

void LoggerRos::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
//  for (int r=0; r<var.rows(); r++)  {
//    for (int c=0; c<var.cols(); c++)  {
//      addDoubleToLog((double *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
//    }
//  }
}

void LoggerRos::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
//  for (int r=0; r<var.rows(); r++)  {
//    for (int c=0; c<var.cols(); c++)  {
//      addFloatToLog((float *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
//    }
//  }
}

void LoggerRos::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
//  for (int r=0; r<var.rows(); r++)  {
//    for (int c=0; c<var.cols(); c++)  {
//      addIntToLog((int *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
//    }
//  }
}

void LoggerRos::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
//  for (int r=0; r<var.rows(); r++)  {
//    for (int c=0; c<var.cols(); c++)  {
//      addShortToLog((short *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
//    }
//  }
}

void LoggerRos::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
//  for (int r=0; r<var.rows(); r++)  {
//    for (int c=0; c<var.cols(); c++)  {
//      addLongToLog((long *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
//    }
//  }
}

void LoggerRos::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
//  for (int r=0; r<var.rows(); r++)  {
//    for (int c=0; c<var.cols(); c++)  {
//      addCharToLog((char *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
//    }
//  }
}

void LoggerRos::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
//  for (int r=0; r<var.rows(); r++)  {
//    for (int c=0; c<var.cols(); c++)  {
//      addCharToLog((char *)(&var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
//    }
//  }
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
//  addKindr3DToCollectedVariables<KindrPositionD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrPositionType, &position);
}

void LoggerRos::addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation,  const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrRotationQuaternionD, geometry_msgs::QuaternionStamped>(group+name, LoggerRos::VarType::KindrRotationQuaternionType, &rotation);
}

void LoggerRos::addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrEulerAnglesZyxD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrEulerAnglesZyxType, &rotation);
}

void LoggerRos::addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrAngularVelocityD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrLocalAngularVelocityType, &angVel);
}

void LoggerRos::addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis,                   const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerRos::addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat,            const std::string& name, const std::string& group, const std::string& unit, bool update) { }

void LoggerRos::addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrRotationVectorD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrRotationVectorType, &rotVec);
}

void LoggerRos::addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrLinearVelocityD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrLinearVelocityType, &linVel);
}

void LoggerRos::addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrLinearAccelerationD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrLinearAccelerationType, &linAcc);
}

void LoggerRos::addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrAngularAccelerationD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrAngularAccelerationType, &angAcc);
}

void LoggerRos::addDoubleKindrForceToLog(const KindrForceD& force, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrForceD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrForceType, &force);
}

void LoggerRos::addDoubleKindrTorqueToLog(const KindrTorqueD& torque, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrTorqueD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrTorqueType, &torque);
}

void LoggerRos::addDoubleKindrVectorToLog(const KindrVectorD& vector, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrVectorD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrVectorType, &vector);
}

void LoggerRos::addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector,
                                                 const KindrPositionD& position,
                                                 const std::string& name,
                                                 const std::string& vectorFrame,
                                                 const std::string& positionFrame,
                                                 const std::string& group,
                                                 const std::string& unit,
                                                 bool update) {
//  kindr_msgs::VectorAtPosition msg;
//  msg.type = kindr_msgs::VectorAtPosition::TYPE_TYPELESS;
//  msg.header.frame_id = vectorFrame;
//  msg.position_frame_id = positionFrame;
//  msg.name = name;
//  addKindr3DVectorAtPositionToCollectedVariables(group+name,
//                                                 msg,
//                                                 &vector,
//                                                 &position);
}

void LoggerRos::addDoubleKindrForceAtPositionToLog( const KindrForceD& force,
                                                 const KindrPositionD& position,
                                                 const std::string& name,
                                                 const std::string& forceFrame,
                                                 const std::string& positionFrame,
                                                 const std::string& group,
                                                 const std::string& unit,
                                                 bool update) {
//  kindr_msgs::VectorAtPosition msg;
//  msg.type = kindr_msgs::VectorAtPosition::TYPE_FORCE;
//  msg.header.frame_id = forceFrame;
//  msg.position_frame_id = positionFrame;
//  msg.name = name;
//  addKindr3DVectorAtPositionToCollectedVariables(group+name,
//                                                 msg,
//                                                 &force,
//                                                 &position);
}

void LoggerRos::addDoubleKindrTorqueAtPositionToLog(const KindrTorqueD& torque,
                                                 const KindrPositionD& position,
                                                 const std::string& name,
                                                 const std::string& torqueFrame,
                                                 const std::string& positionFrame,
                                                 const std::string& group,
                                                 const std::string& unit,
                                                 bool update) {
//  kindr_msgs::VectorAtPosition msg;
//  msg.type = kindr_msgs::VectorAtPosition::TYPE_TORQUE;
//  msg.header.frame_id = torqueFrame;
//  msg.position_frame_id = positionFrame;
//  msg.name = name;
//  addKindr3DVectorAtPositionToCollectedVariables(group+name,
//                                                 msg,
//                                                 &torque,
//                                                 &position);
}
/******************/

bool LoggerRos::checkIfVarCollected(const std::string& topicName, std::vector<LogElementBase*>::iterator& it) {
//  for (it = collectedVars_.begin(); it != collectedVars_.end(); it++) {
//    if (it->topicName_.compare(topicName) == 0) {
//      ROCO_WARN_STREAM("[LoggerRos::checkIfVarCollected] Topic '" << topicName << "' was already published.");
//      return true;
//    }
//  }
  return false;
}

//bool LoggerRos::checkIfVarCollected(const std::string& topicName, std::vector<LoggerVarInfo>::iterator& it) {
//  for (it = collectedVars_.begin(); it != collectedVars_.end(); it++) {
//    if (it->topicName_.compare(topicName) == 0) {
//      ROCO_WARN_STREAM("[LoggerRos::checkIfVarCollected] Topic '" << topicName << "' was already published.");
//      return true;
//    }
//  }
//  return false;
//}


} /* namespace signal_logger_ros */
