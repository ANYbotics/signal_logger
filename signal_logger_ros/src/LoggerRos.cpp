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

}


void LoggerRos::initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName) { }
void LoggerRos::startLogger() { }
void LoggerRos::stopLogger() { }
void LoggerRos::saveLoggerData() { }


void LoggerRos::updateLogger(bool updateScript) {

}


void LoggerRos::collectLoggerData() {

  for (auto& elem : collectedVars_) {
    ros::Time stamp = ros::Time::now();
    elem->publish(stamp);

//    if (elem.pub_.getNumSubscribers() > 0u) {
//      switch (elem.type_) {

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
  addVarToCollection<float>(group+name, var);
}

void LoggerRos::addDoubleToLog(double* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<double>(group+name, var);
}

void LoggerRos::addIntToLog(int* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<int>(group+name, var);
}

void LoggerRos::addShortToLog(short* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<short>(group+name, var);
}


void LoggerRos::addLongToLog(long* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<long>(group+name, var);
}


void LoggerRos::addCharToLog(char* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<char>(group+name, var);
}

void LoggerRos::addBoolToLog(bool* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<bool>(group+name, var);
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
  addVarToCollection<Eigen::Ref<Eigen::Vector3d>>(group+name, &var);
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
  addVarToCollection<KindrPositionD>(group+name, &position);
}

void LoggerRos::addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation,  const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrRotationQuaternionD>(group+name, &rotation);
}

void LoggerRos::addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrEulerAnglesZyxD>(group+name, &rotation);
}

void LoggerRos::addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrAngularVelocityD>(group+name, &angVel);
}

void LoggerRos::addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis,                   const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerRos::addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat,            const std::string& name, const std::string& group, const std::string& unit, bool update) { }

void LoggerRos::addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrRotationVectorD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrRotationVectorType, &rotVec);
}

void LoggerRos::addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrLinearVelocityD>(group+name, &linVel);
}

void LoggerRos::addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrLinearAccelerationD>(group+name, &linAcc);
}

void LoggerRos::addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrAngularAccelerationD>(group+name, &angAcc);
}

void LoggerRos::addDoubleKindrForceToLog(const KindrForceD& force, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrForceD>(group+name, &force);
}

void LoggerRos::addDoubleKindrTorqueToLog(const KindrTorqueD& torque, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrTorqueD>(group+name, &torque);
}

void LoggerRos::addDoubleKindrVectorToLog(const KindrVectorD& vector, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrVectorD>(group+name, &vector);
}

void LoggerRos::addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector,
                                                 const KindrPositionD& position,
                                                 const std::string& name,
                                                 const std::string& vectorFrame,
                                                 const std::string& positionFrame,
                                                 const std::string& group,
                                                 const std::string& unit,
                                                 bool update) {
//  std::vector<LogElementBase*>::iterator collectedIterator;
//  if (checkIfVarCollected(group+name, collectedIterator)) {
//    dynamic_cast<LogElement<KindrVectorD>*>(*collectedIterator)->setLogVarAtPositionPointer(&vector,
//                                                                                            &position,
//                                                                                            vectorFrame,
//                                                                                            positionFrame,
//                                                                                            name);
//  } else {
//    collectedVars_.push_back(new LogElement<KindrVectorD>(nodeHandle_, group+name, &vector));
//  }
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
  for (it = collectedVars_.begin(); it != collectedVars_.end(); it++) {
    if ((*it)->getTopicName().compare(topicName) == 0) {
      ROCO_WARN_STREAM("[LoggerRos::checkIfVarCollected] Topic '" << topicName << "' was already being published.");
      return true;
    }
  }
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
