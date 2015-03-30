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
* @file 	LoggerBase.hpp
* @author 	Christian Gehring, C. Dario Bellicoso
* @date		July 7, 2013
* @version 	1.0
* @ingroup 	signal_logger
* @brief
*/
#pragma once

#include <string>
#include <Eigen/Core>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

/*****************
 * Kindr headers *
 *****************/
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>
#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/rotations/RotationDiffEigen.hpp>
/*****************/


const std::string LOGGER_DEFAULT_GROUP_NAME = "/log/";
const std::string LOGGER_DEFAULT_UNIT       = "-";
const bool LOGGER_DEFAULT_UPDATE            = false;
const std::string LOGGER_DEFAULT_FILENAME   = "logger.script";


namespace signal_logger {

//! Interface for loggers
/*! Derive your own logger from this class.
 */
class LoggerBase {
 public:
  /************************
   * Convenience typedefs *
   ************************/
  typedef kindr::phys_quant::eigen_impl::Position3D             KindrPositionD;
  typedef kindr::rotations::eigen_impl::RotationQuaternionPD    KindrRotationQuaternionD;
  typedef kindr::rotations::eigen_impl::EulerAnglesZyxPD        KindrEulerAnglesZyxD;
  typedef kindr::rotations::eigen_impl::LocalAngularVelocityPD  KindrAngularVelocityD;
  typedef kindr::rotations::eigen_impl::AngleAxisPD             KindrAngleAxisD;
  typedef kindr::rotations::eigen_impl::RotationMatrixPD        KindrRotationMatrixD;
  typedef kindr::rotations::eigen_impl::RotationVectorPD        KindrRotationVectorD;
  typedef kindr::phys_quant::eigen_impl::Velocity3D             KindrLinearVelocityD;
  typedef kindr::phys_quant::eigen_impl::Acceleration3D         KindrLinearAccelerationD;
  typedef kindr::phys_quant::eigen_impl::AngularAcceleration3D  KindrAngularAccelerationD;
  typedef kindr::phys_quant::eigen_impl::Force3D                KindrForceD;
  typedef kindr::phys_quant::eigen_impl::Torque3D               KindrTorqueD;
  typedef kindr::phys_quant::eigen_impl::VectorTypeless3D       KindrVectorD;

  typedef Eigen::Matrix< long ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXl;
  typedef Eigen::Matrix< short ,Eigen::Dynamic, Eigen::Dynamic >        MatrixXs;
  typedef Eigen::Matrix< char ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXc;
  typedef Eigen::Matrix< unsigned char ,Eigen::Dynamic, Eigen::Dynamic > MatrixXUc;
  typedef Eigen::Matrix< bool ,Eigen::Dynamic, Eigen::Dynamic >         MatrixXb;
  typedef Eigen::Matrix< std::string ,Eigen::Dynamic, Eigen::Dynamic >  MatrixXstring;
  /************************/

  enum LoggerType {
    TypeNone = 0,
    TypeStd,
    TypeSl,
    TypeRos
  };

	LoggerBase();
	virtual ~LoggerBase();

	/****************
	 * Util methods *
	 ****************/
	virtual void initLogger(int updateFrequency, int samplingFrequency, double samplingTime, const std::string& logScriptFileName = std::string{LOGGER_DEFAULT_FILENAME}) = 0;
	virtual void startLogger() = 0;
	virtual void stopLogger() = 0;
	virtual void saveLoggerData() = 0;
	virtual void updateLogger(bool updateScript = true) = 0;
	virtual void collectLoggerData() = 0;
	virtual void lockUpdate() = 0;
	virtual void stopAndSaveLoggerData() = 0;
	virtual void restartLogger();
	virtual const LoggerType getLoggerType() const = 0;
	/****************/

	/****************
	 * Core methods *
	 ****************/
	virtual void addFloatToLog(const float& var,    const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
	virtual void addDoubleToLog(const double& var,  const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
	virtual void addIntToLog(const int& var,        const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
	virtual void addShortToLog(const short& var,    const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
	virtual void addLongToLog(const long& var,      const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
	virtual void addCharToLog(const char& var,      const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addBoolToLog(const bool& var,      const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  /****************/

  /******************
   * Eigen wrappers *
   ******************/
  virtual void addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var,              const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var,               const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var,                 const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var,          const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var,           const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var,           const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var,  const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var,           const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var,             const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;

  virtual void addDoubleEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXd>& var,              const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addFloatEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXf>& var,               const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addIntEigenMatrixToLog(const Eigen::Ref<Eigen::MatrixXi>& var,                 const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addShortEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXs>& var,          const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addLongEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXl>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXc>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addUnsignedCharEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXUc>& var,  const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addBoolEigenMatrixToLog(const Eigen::Ref<LoggerBase::MatrixXb>& var,           const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleEigenVector3ToLog(const Eigen::Ref<Eigen::Vector3d>& var,             const Eigen::Ref<LoggerBase::MatrixXstring>& names, const std::string& group = std::string {LOGGER_DEFAULT_GROUP_NAME }, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  /******************/

  /******************
   * Kindr wrappers *
   ******************/
  virtual void addDoubleKindrPositionToLog(const KindrPositionD& position,                      const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrRotationQuaternionToLog(const KindrRotationQuaternionD& rotation,  const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrEulerAnglesZyxToLog(const KindrEulerAnglesZyxD& rotation,          const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrLocalAngularVelocityToLog(const KindrAngularVelocityD& angVel,     const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrAngleAxisToLog(const KindrAngleAxisD& angleAxis,                   const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrRotationMatrixToLog(const KindrRotationMatrixD& rotMat,            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrRotationVectorToLog(const KindrRotationVectorD& rotVec,            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrLinearVelocityToLog(const KindrLinearVelocityD& linVel,            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrLinearAccelerationToLog(const KindrLinearAccelerationD& linAcc,    const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrAngularAccelerationToLog(const KindrAngularAccelerationD& angAcc,  const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrForceToLog(const KindrForceD& force,                               const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrTorqueToLog(const KindrTorqueD& torque,                            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrVectorToLog(const KindrVectorD& vector,                            const std::string& name, const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, bool update = LOGGER_DEFAULT_UPDATE) = 0;

  virtual void addDoubleKindrVectorAtPositionToLog(const KindrVectorD& vector,
                                                   const KindrPositionD& position,
                                                   const std::string& name,
                                                   const std::string& vectorFrame = "world",
                                                   const std::string& positionFrame = "world",
                                                   const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
                                                   const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
                                                   bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrForceAtPositionToLog( const KindrForceD& force,
                                                   const KindrPositionD& position,
                                                   const std::string& name,
                                                   const std::string& forceFrame = "world",
                                                   const std::string& positionFrame = "world",
                                                   const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
                                                   const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
                                                   bool update = LOGGER_DEFAULT_UPDATE) = 0;
  virtual void addDoubleKindrTorqueAtPositionToLog(const KindrTorqueD& torque,
                                                   const KindrPositionD& position,
                                                   const std::string& name,
                                                   const std::string& torqueFrame = "world",
                                                   const std::string& positionFrame = "world",
                                                   const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME},
                                                   const std::string& unit = std::string{LOGGER_DEFAULT_UNIT},
                                                   bool update = LOGGER_DEFAULT_UPDATE) = 0;

  /******************/

protected:
  //! Indicates if the logger is initialized.
	bool isInitialized_;
	//! update frequency of collect()
	int updateFrequency_;
	//! sampling frequency [Hz]
	int samplingFrequency_;
	//! sampling time [s]
	double samplingTime_;

};

} /* namespace signal_logger */

