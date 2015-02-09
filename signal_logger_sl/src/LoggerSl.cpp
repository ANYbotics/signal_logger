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

#include "signal_logger_sl/LoggerSl.hpp"

/***************
 * SL includes *
 ***************/
#include "SL.h"
#include "SL_user.h"
#include "SL_collect_data.h"
#include "SL_oscilloscope.h"
/***************/

namespace signal_logger_sl {

LoggerSl::LoggerSl():
  LoggerBase()
{

}


LoggerSl::~LoggerSl() {

}


void LoggerSl::initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName) {

}


void LoggerSl::updateLogger(bool updateScript) {
  updateDataCollectScript();
  updateOscVars();
}


void LoggerSl::collectLoggerData() {

}


void LoggerSl::startLogger() {
  scd();
}


void LoggerSl::stopLogger() {
  stopcd();
}


void LoggerSl::restartLogger() {
  LoggerSl::stopLogger();
  LoggerSl::startLogger();
}


void LoggerSl::saveLoggerData() {
  saveData();
}


void LoggerSl::stopAndSaveLoggerData() {
  LoggerSl::stopLogger();
  LoggerSl::saveLoggerData();
}


void LoggerSl::lockUpdate() {
  return;
}


void LoggerSl::addFloatToLog(float* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  std::string varName = group + name;
  addVarToCollect((char *)(var), (char*)varName.c_str(), (char*)unit.c_str(), FLOAT,(int)update);
}

void LoggerSl::addDoubleToLog(double* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  std::string varName = group + name;
  addVarToCollect((char *)(var), (char*) varName.c_str(), (char*)unit.c_str(), DOUBLE,(int)update);
}

void LoggerSl::addIntToLog(int* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  std::string varName = group + name;
  addVarToCollect((char *)(var), (char*)varName.c_str(), (char*)unit.c_str(), INT,(int)update);
}

void LoggerSl::addShortToLog(short* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  std::string varName = group + name;
  addVarToCollect((char *)(var), (char*)varName.c_str(), (char*) unit.c_str(), SHORT,(int)update);
}

void LoggerSl::addLongToLog(long* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  std::string varName = group + name;
  addVarToCollect((char *)(var), (char*) varName.c_str(), (char*) unit.c_str(), LONG,(int)update);
}

void LoggerSl::addCharToLog(char* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  std::string varName = group + name;
  addVarToCollect((char *)(var), (char*) varName.c_str(), (char*) unit.c_str(), CHAR,(int)update);
}

void LoggerSl::addBoolToLog(bool* var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  std::string varName = group + name;
  addVarToCollect((char *)(var), (char*) varName.c_str(), (char*)unit.c_str(), CHAR,(int)update);
}


/*****************************
 * Add Eigen matrices to log *
 *****************************/
void LoggerSl::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addDoubleToLog((double *)(&var(r,c)), name_of_var, group, unit, update);
    }
  }
}

void LoggerSl::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addFloatToLog((float *)(&var(r,c)), name_of_var, group, unit, update);
    }
  }
}

void LoggerSl::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addIntToLog((int *)(&var(r,c)), name_of_var, group, unit, update);
    }
  }
}

void LoggerSl::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addShortToLog((short *)(&var(r,c)), name_of_var, group, unit, update);
    }
  }
}

void LoggerSl::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addLongToLog((long *)(&var(r,c)), name_of_var, group, unit, update);
    }
  }
}

void LoggerSl::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addCharToLog((char *)(&var(r,c)), name_of_var, group, unit, update);
    }
  }
}

void LoggerSl::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addCharToLog((char *)(&var(r,c)), name_of_var, group, unit, update);
    }
  }
}

void LoggerSl::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      std::string name_of_var = name + "_" + boost::lexical_cast<std::string>(r) + "_" + boost::lexical_cast<std::string>(c);
      addBoolToLog((bool *)(&var(r,c)), name_of_var, group, unit, update);
    }
  }
}

void LoggerSl::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog((double *)(&var(0)), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog((double *)(&var(1)), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog((double *)(&var(2)), std::string{name + "_z"}, group, unit, update);
}


void LoggerSl::addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addDoubleToLog((double *)(&var(r,c)), names(r,c), group, unit, update);
    }
  }
}

void LoggerSl::addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addFloatToLog((float *)(&var(r,c)), names(r,c), group, unit, update);
    }
  }
}

void LoggerSl::addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addIntToLog((int *)(&var(r,c)), names(r,c), group, unit, update);
    }
  }
}

void LoggerSl::addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addShortToLog((short *)(&var(r,c)), names(r,c), group, unit, update);
    }
  }
}

void LoggerSl::addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addLongToLog((long *)(&var(r,c)), names(r,c), group, unit, update);
    }
  }
}

void LoggerSl::addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addCharToLog((char *)(&var(r,c)), names(r,c), group, unit, update);
    }
  }
}

void LoggerSl::addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addCharToLog((char *)(&var(r,c)), names(r,c), group, unit, update);
    }
  }
}

void LoggerSl::addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  for (int r=0; r<var.rows(); r++)  {
    for (int c=0; c<var.cols(); c++)  {
      addBoolToLog((bool *)(&var(r,c)), names(r,c), group, unit, update);
    }
  }
}

void LoggerSl::addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var, const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog((double *)(&var(0)), names(0,0), group, unit, update);
  addDoubleToLog((double *)(&var(1)), names(1,0), group, unit, update);
  addDoubleToLog((double *)(&var(2)), names(2,0), group, unit, update);
}

/*****************************/


/******************
 * Kindr wrappers *
 ******************/
void LoggerSl::addDoubleKindrPositionToLog(const KindrPositionD& position, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrPositionD&>(position).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrPositionD&>(position).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrPositionD&>(position).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrRotationQuaternionD&>(rotation).toImplementation().w(), std::string{name + "_w"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrRotationQuaternionD&>(rotation).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrRotationQuaternionD&>(rotation).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrRotationQuaternionD&>(rotation).toImplementation().x(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation,  const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrEulerAnglesZyxD&>(rotation).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrEulerAnglesZyxD&>(rotation).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrEulerAnglesZyxD&>(rotation).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrAngularVelocityD&>(angVel).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrAngularVelocityD&>(angVel).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrAngularVelocityD&>(angVel).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrAngleAxisD&>(angleAxis).toImplementation().axis().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrAngleAxisD&>(angleAxis).toImplementation().axis().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrAngleAxisD&>(angleAxis).toImplementation().axis().z(), std::string{name + "_z"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrAngleAxisD&>(angleAxis).toImplementation().angle(),  std::string{name + "_angle"}, group, unit, update);
}

void LoggerSl::addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleEigenMatrixToLog(const_cast<KindrRotationMatrixD&>(rotMat).toImplementation(), name, group, unit, update);
}

void LoggerSl::addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrRotationVectorD&>(rotVec).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrRotationVectorD&>(rotVec).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrRotationVectorD&>(rotVec).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrLinearVelocityD&>(linVel).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrLinearVelocityD&>(linVel).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrLinearVelocityD&>(linVel).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrLinearAccelerationD&>(linAcc).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrLinearAccelerationD&>(linAcc).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrLinearAccelerationD&>(linAcc).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrAngularAccelerationD&>(angAcc).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrAngularAccelerationD&>(angAcc).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrAngularAccelerationD&>(angAcc).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrForceToLog(const KindrForceD& force, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrForceD&>(force).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrForceD&>(force).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrForceD&>(force).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrTorqueToLog(const KindrTorqueD& torque, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrTorqueD&>(torque).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrTorqueD&>(torque).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrTorqueD&>(torque).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrVectorToLog(const KindrVectorD& vector, const std::string& name, const std::string& group, const std::string& unit, bool update) {
  addDoubleToLog(&const_cast<KindrVectorD&>(vector).toImplementation().x(), std::string{name + "_x"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrVectorD&>(vector).toImplementation().y(), std::string{name + "_y"}, group, unit, update);
  addDoubleToLog(&const_cast<KindrVectorD&>(vector).toImplementation().z(), std::string{name + "_z"}, group, unit, update);
}

void LoggerSl::addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector, const KindrPositionD& position, const std::string& name, const std::string& group, const std::string& unit, bool update) {

}
/******************/


} /* namespace signal_logger_sl */

//#endif
