/*
 * Copyright (c) 2014, Christian Gehring, Michael Bloesch, Peter Fankhauser, C. Dario Bellicoso
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL  Christian Gehring, Michael Bloesch, Peter Fankhauser
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
/*
 * LoggerNone.cpp
 *
 *  Created on: Apr 2, 2014
 *      Author: Christian Gehring, C. Dario Bellicoso
 */

#include "signal_logger/LoggerNone.hpp"

namespace signal_logger {

LoggerNone::LoggerNone() {


}

LoggerNone::~LoggerNone() {

}


void LoggerNone::initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName) { }
void LoggerNone::startLogger() { }
void LoggerNone::stopLogger() { }
void LoggerNone::saveLoggerData() { }
void LoggerNone::updateLogger(bool updateScript) { }
void LoggerNone::collectLoggerData() { }
void LoggerNone::lockUpdate() { }
void LoggerNone::stopAndSaveLoggerData() { }


void LoggerNone::addFloatToLog(float* var,    const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleToLog(double* var,  const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addIntToLog(int* var,        const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addShortToLog(short* var,    const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addLongToLog(long* var,      const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addCharToLog(char* var,      const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addBoolToLog(bool* var,      const std::string& name, const std::string& group, const std::string& unit, bool update) { }


/******************
 * Eigen wrappers *
 ******************/
void LoggerNone::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var,              const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var,               const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var,                 const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var,          const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var,           const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var,           const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var,  const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var,           const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var,             const std::string& name, const std::string& group, const std::string& unit, bool update) { }

void LoggerNone::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var,              const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var,               const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var,                 const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var,          const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var,  const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var,             const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) { }
/******************/


/******************
 * Kindr wrappers *
 ******************/
void LoggerNone::addDoubleKindrPositionToLog(const KindrPositionD& position,                      const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation,  const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation,          const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel,     const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis,                   const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat,            const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec,            const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel,            const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc,    const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc,  const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrForceToLog(const KindrForceD& force,                               const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrTorqueToLog(const KindrTorqueD& torque,                            const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrVectorToLog(const KindrVectorD& vector,                            const std::string& name, const std::string& group, const std::string& unit, bool update) { }
void LoggerNone::addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector, const KindrPositionD& position, const std::string& name, const std::string& group, const std::string& unit, bool update) { }
/******************/



} /* namespace signal_logger */
