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


const std::string LOGGER_PREFIX = "/log";

namespace signal_logger_ros {

LoggerRos::LoggerRos(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle),
    collectedVars_(0)
{
}


LoggerRos::~LoggerRos()
{

}


void LoggerRos::initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName) {
  updateFrequency_ = updateFrequency;
  samplingFrequency_ = samplingFrequency;
}

void LoggerRos::startLogger() { }
void LoggerRos::stopLogger() { }
void LoggerRos::saveLoggerData() { }
void LoggerRos::updateLogger(bool updateScript) { }


void LoggerRos::setPublishFrequency(int frequency) {
  updateFrequency_ = frequency;
}


void LoggerRos::clearCollectedVariables() {
    collectedVars_.clear();
}

const signal_logger::LoggerBase::LoggerType LoggerRos::getLoggerType() const {
  return signal_logger::LoggerBase::LoggerType::TypeRos;
}

void LoggerRos::collectLoggerData()
{
  const ros::Time stamp = ros::Time::now();
  for (const auto& elem : collectedVars_) {
    if (elem.get()->getNumSubscribers() > 0u) {
      elem->publish(stamp);
    }
  }
}


void LoggerRos::lockUpdate() { }
void LoggerRos::stopAndSaveLoggerData() { }

/**************
 * Core types *
 **************/
void LoggerRos::addFloatToLog(const float& var, const std::string& name,
                              const std::string& group, const std::string& unit,
                              bool update)
{
  addVarToCollection<float>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addDoubleToLog(const double& var, const std::string& name,
                               const std::string& group,
                               const std::string& unit, bool update)
{
  addVarToCollection<double>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addIntToLog(const int& var, const std::string& name,
                            const std::string& group, const std::string& unit,
                            bool update)
{
  addVarToCollection<int>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addShortToLog(const short& var, const std::string& name,
                              const std::string& group, const std::string& unit,
                              bool update)
{
  addVarToCollection<short>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addLongToLog(const long& var, const std::string& name,
                             const std::string& group, const std::string& unit,
                             bool update)
{
  addVarToCollection<long>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addCharToLog(const char& var, const std::string& name,
                             const std::string& group, const std::string& unit,
                             bool update)
{
  addVarToCollection<char>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addBoolToLog(const bool& var, const std::string& name,
                             const std::string& group, const std::string& unit,
                             bool update)
{
  addVarToCollection<bool>(LOGGER_PREFIX + group + name, &var);
}
/**************/


/******************
 * Eigen wrappers *
 ******************/
void LoggerRos::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<Eigen::MatrixXd>>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<Eigen::MatrixXf>>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<Eigen::MatrixXi>>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<LoggerBase::MatrixXs>>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<LoggerBase::MatrixXl>>(group + name, &var);
}

void LoggerRos::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<LoggerBase::MatrixXc>>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<LoggerBase::MatrixXUc>>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<LoggerBase::MatrixXb>>(LOGGER_PREFIX + group + name, &var);
}

void LoggerRos::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<Eigen::Ref<Eigen::Vector3d>>(LOGGER_PREFIX + group+name, &var);
}

void LoggerRos::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<static_cast<Eigen::MatrixXd>(var).rows(); r++)  {
    for (int c=0; c<static_cast<Eigen::MatrixXd>(var).cols(); c++)  {
      addDoubleToLog((double)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<static_cast<Eigen::MatrixXf>(var).rows(); r++)  {
    for (int c=0; c<static_cast<Eigen::MatrixXf>(var).cols(); c++)  {
      addFloatToLog((float)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<static_cast<Eigen::MatrixXi>(var).rows(); r++)  {
    for (int c=0; c<static_cast<Eigen::MatrixXi>(var).cols(); c++)  {
      addIntToLog((int)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<static_cast<LoggerBase::MatrixXs>(var).rows(); r++)  {
    for (int c=0; c<static_cast<LoggerBase::MatrixXs>(var).cols(); c++)  {
      addShortToLog((short)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<static_cast<LoggerBase::MatrixXl>(var).rows(); r++)  {
    for (int c=0; c<static_cast<LoggerBase::MatrixXl>(var).cols(); c++)  {
      addLongToLog((long)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<static_cast<LoggerBase::MatrixXc>(var).rows(); r++)  {
    for (int c=0; c<static_cast<LoggerBase::MatrixXc>(var).cols(); c++)  {
      addCharToLog((char)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<static_cast<LoggerBase::MatrixXUc>(var).rows(); r++)  {
    for (int c=0; c<static_cast<LoggerBase::MatrixXUc>(var).cols(); c++)  {
      addCharToLog((char)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<static_cast<LoggerBase::MatrixXb>(var).rows(); r++)  {
    for (int c=0; c<static_cast<LoggerBase::MatrixXb>(var).cols(); c++)  {
      addBoolToLog((bool)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update);
    }
  }
}

void LoggerRos::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog((double)(var(0)), static_cast<std::string>(names(0,0)), group, unit, update);
  addDoubleToLog((double)(var(1)), static_cast<std::string>(names(1,0)), group, unit, update);
  addDoubleToLog((double)(var(2)), static_cast<std::string>(names(2,0)), group, unit, update);
}
/******************/

void LoggerRos::addTimestampToLog(const TimestampPair& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<TimestampPair>(LOGGER_PREFIX + group+name, &var);
}

/******************
 * Kindr wrappers *
 ******************/
void LoggerRos::addDoubleKindrPositionToLog(const KindrPositionD& position, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrPositionD>(LOGGER_PREFIX + group+name, &position);
}

void LoggerRos::addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation,  const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrRotationQuaternionD>(LOGGER_PREFIX + group+name, &rotation);
}

void LoggerRos::addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrEulerAnglesZyxD>(LOGGER_PREFIX + group+name, &rotation);
}

void LoggerRos::addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrAngularVelocityD>(LOGGER_PREFIX + group+name, &angVel);
}

void LoggerRos::addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis,                   const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerRos::addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat,            const std::string& name, const std::string& group, const std::string& unit, bool update) { }

void LoggerRos::addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec, const std::string& name, const std::string& group, const std::string& unit, bool update) {
//  addKindr3DToCollectedVariables<KindrRotationVectorD, geometry_msgs::Vector3Stamped>(group+name, LoggerRos::VarType::KindrRotationVectorType, &rotVec);
}

void LoggerRos::addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrLinearVelocityD>(LOGGER_PREFIX + group+name, &linVel);
}

void LoggerRos::addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrLinearAccelerationD>(LOGGER_PREFIX + group+name, &linAcc);
}

void LoggerRos::addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrAngularAccelerationD>(LOGGER_PREFIX + group+name, &angAcc);
}

void LoggerRos::addDoubleKindrForceToLog(const KindrForceD& force, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrForceD>(LOGGER_PREFIX + group+name, &force);
}

void LoggerRos::addDoubleKindrTorqueToLog(const KindrTorqueD& torque, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrTorqueD>(LOGGER_PREFIX + group+name, &torque);
}

void LoggerRos::addDoubleKindrVectorToLog(const KindrVectorD& vector, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addVarToCollection<KindrVectorD>(LOGGER_PREFIX + group+name, &vector);
}

void LoggerRos::addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector,
                                                    const KindrPositionD& position,
                                                    const std::string& name,
                                                    const std::string& vectorFrame,
                                                    const std::string& positionFrame,
                                                    const std::string& group,
                                                    const std::string& unit,
                                                    bool update) {
  addVarToCollection<KindrVectorD, true>(LOGGER_PREFIX + group+name,
                                         &vector,
                                         &position,
                                         name,
                                         vectorFrame,
                                         positionFrame);
}

void LoggerRos::addDoubleKindrForceAtPositionToLog(const KindrForceD& force,
                                                   const KindrPositionD& position,
                                                   const std::string& name,
                                                   const std::string& forceFrame,
                                                   const std::string& positionFrame,
                                                   const std::string& group,
                                                   const std::string& unit,
                                                   bool update) {
  addVarToCollection<KindrForceD, true>(LOGGER_PREFIX + group+name,
                                        &force,
                                        &position,
                                        name,
                                        forceFrame,
                                        positionFrame);
}

void LoggerRos::addDoubleKindrTorqueAtPositionToLog(const KindrTorqueD& torque,
                                                 const KindrPositionD& position,
                                                 const std::string& name,
                                                 const std::string& torqueFrame,
                                                 const std::string& positionFrame,
                                                 const std::string& group,
                                                 const std::string& unit,
                                                 bool update) {
  addVarToCollection<KindrTorqueD, true>(LOGGER_PREFIX + group+name,
                                         &torque,
                                         &position,
                                         name,
                                         torqueFrame,
                                         positionFrame);
}
void LoggerRos::addDoubleKindrLinearVelocityAtPositionToLog(const KindrLinearVelocityD& velocity,
                                                         const KindrPositionD& position,
                                                         const std::string& name,
                                                         const std::string& torqueFrame,
                                                         const std::string& positionFrame,
                                                         const std::string& group,
                                                         const std::string& unit,
                                                         bool update) {
  addVarToCollection<KindrLinearVelocityD, true>(LOGGER_PREFIX + group + name, &velocity, &position,
                                                 name, torqueFrame, positionFrame);
}
void LoggerRos::addDoubleKindrLinearAccelerationAtPositionToLog(const KindrLinearAccelerationD& acceleration,
                                                             const KindrPositionD& position,
                                                             const std::string& name,
                                                             const std::string& torqueFrame,
                                                             const std::string& positionFrame,
                                                             const std::string& group,
                                                             const std::string& unit,
                                                             bool update) {
  addVarToCollection<KindrLinearAccelerationD, true>(LOGGER_PREFIX + group + name, &acceleration,
                                                     &position, name, torqueFrame, positionFrame);
}
/******************/

bool LoggerRos::checkIfVarCollected(
    const std::string& topicName,
    std::vector<std::shared_ptr<LogElementBase>>::iterator& it)
{
  for (it = collectedVars_.begin(); it != collectedVars_.end(); it++) {
    if ((*it).get()->getTopicName().compare(topicName) == 0) {
      ROCO_WARN_STREAM(
          "[LoggerRos::checkIfVarCollected] Topic '" << topicName << "' was already being published.");
      return true;
    }
  }
  return false;
}




} /* namespace signal_logger_ros */
