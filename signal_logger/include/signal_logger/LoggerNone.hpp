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
/*!
* @file   LoggerNone.hpp
* @author   Christian Gehring, C. Dario Bellicoso
* @date   July 7, 2013
* @version  1.0
* @ingroup  signal_logger
* @brief
*/
#pragma once

#include "signal_logger/LoggerBase.hpp"

namespace signal_logger {

class LoggerNone: public LoggerBase {
 public:
  LoggerNone();
  virtual ~LoggerNone();

  virtual void initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName = std::string{LOGGER_DEFAULT_FILENAME});
  virtual void startLogger();
  virtual void stopLogger();
  virtual void saveLoggerData();
  virtual void updateLogger(bool updateScript = true);
  virtual void collectLoggerData();
  virtual void lockUpdate();
  virtual void stopAndSaveLoggerData();
  virtual const LoggerType getLoggerType() const;

  virtual void addFloatToLog(const float& var,    const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleToLog(const double& var,  const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addIntToLog(const int& var,        const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addShortToLog(const short& var,    const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addLongToLog(const long& var,      const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addCharToLog(const char& var,      const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addBoolToLog(const bool& var,      const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);


  /******************
   * Eigen wrappers *
   ******************/
  virtual void addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var,              const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var,               const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var,                 const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var,          const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var,           const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var,           const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var,  const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var,           const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var,             const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);

  virtual void addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var,              const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var,               const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var,                 const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var,          const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var,  const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var,             const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  /******************/


  /******************
   * Kindr wrappers *
   ******************/
  virtual void addDoubleKindrPositionToLog(const KindrPositionD& position,                      const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation,  const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation,          const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel,     const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis,                   const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat,            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec,            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel,            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc,    const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc,  const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrForceToLog(const KindrForceD& force,                               const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrTorqueToLog(const KindrTorqueD& torque,                            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrVectorToLog(const KindrVectorD& vector,                            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE);

  virtual void addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector,
                                                   const KindrPositionD& position,
                                                   const std::string& name,
                                                   const std::string& vectorFrame = "world",
                                                   const std::string& positionFrame = "world",
                                                   const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
                                                   const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
                                                   bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrForceAtPositionToLog( const KindrForceD& force,
                                                   const KindrPositionD& position,
                                                   const std::string& name,
                                                   const std::string& forceFrame = "world",
                                                   const std::string& positionFrame = "world",
                                                   const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
                                                   const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
                                                   bool update = LOGGER_DEFAULT_UPDATE);
  virtual void addDoubleKindrTorqueAtPositionToLog(const KindrTorqueD& torque,
                                                   const KindrPositionD& position,
                                                   const std::string& name,
                                                   const std::string& torqueFrame = "world",
                                                   const std::string& positionFrame = "world",
                                                   const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
                                                   const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
                                                   bool update = LOGGER_DEFAULT_UPDATE);
  /******************/


};

} /* namespace signal_logger */


