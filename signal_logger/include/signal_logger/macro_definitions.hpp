/*
 * macro_definitions.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */
#pragma once

#define ADD_VAR_DEFINITION(TYPE, NAME, ...) \
    virtual void add##NAME(const TYPE& var, \
                           const std::string& name, \
                           const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, \
                           const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, \
                           bool update = LOGGER_DEFAULT_UPDATE) = 0; /*
                            */

#define ADD_EIGEN_VAR_AS_UNDERLYING_TYPE_IMPLEMENTATION(TYPE, NAME, UNDERLYING_TYPE, UNDERLYING_TYPE_NAME) \
    void add##NAME(const TYPE& var, \
                   const SignalLoggerBase::MatrixXstring& names, \
                   const std::string& group, \
                   const std::string& unit, \
                   bool update) { \
      for (int r=0; r<static_cast<SignalLoggerBase::MatrixXstring>(names).rows(); r++)  { \
        for (int c=0; c<static_cast<SignalLoggerBase::MatrixXstring>(names).cols(); c++)  { \
          add##UNDERLYING_TYPE_NAME((UNDERLYING_TYPE)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, update); \
        } \
      } \
    } /*
 */

#define ADD_VAR_TEMPLATE_SPECIFICATIONS(TYPE, NAME, ...) \
  template < > \
  void SignalLoggerBase::addVariableToLog( const TYPE & var, \
                    const std::string& name, \
                    const std::string& group, \
                    const std::string& unit, \
                    bool update) \
  { \
    if(log_elements_.find(name) != log_elements_.end()) { \
      printf("A signal with the same name %s is already logged. Overwrite.", name.c_str()); \
    } \
    add##NAME(var, name, group, unit, update); \
  }

#define FOR_PRIMITIVE_TYPES(MACRO) \
    MACRO(float, Float)\
    MACRO(double, Double)\
    MACRO(int, Int)\
    MACRO(short, Short)\
    MACRO(long, Long)\
    MACRO(char, Char)\
    MACRO(bool, Bool)\
    MACRO(SignalLoggerBase::TimestampPair, TimeStamp)\

#define FOR_EIGEN_TYPES(MACRO) \
    MACRO(Eigen::Vector3d, DoubleEigenVector3, double, Double) \
    MACRO(Eigen::MatrixXf, FloatEigenMatrix, float, Float)\
    MACRO(Eigen::MatrixXd, DoubleEigenMatrix, double, Double)\
    MACRO(Eigen::MatrixXi, IntegerEigenMatrix, int, Int)\
    MACRO(SignalLoggerBase::MatrixXs, ShortEigenMatrix, short, Short)\
    MACRO(SignalLoggerBase::MatrixXl, LongEigenMatrix, long, Long)\
    MACRO(SignalLoggerBase::MatrixXc, CharEigenMatrix, char,Char)\
    MACRO(SignalLoggerBase::MatrixXUc, UnsignedCharEigenMatrix, char,  Char)\
    MACRO(SignalLoggerBase::MatrixXb, BoolEigenMatrix, bool, Bool)

#define FOR_KINDR_TYPES(MACRO) \
    MACRO(kindr::Position3D, DoubleKindrPosition)\
    MACRO(kindr::RotationQuaternionPD, DoubleKindrRotationQuaternion)\
    MACRO(kindr::EulerAnglesZyxPD, DoubleKindrEulerAnglesZyx)\
    MACRO(kindr::LocalAngularVelocityPD, DoubleKindrLocalAngularVelocity)\
    MACRO(kindr::AngleAxisPD, DoubleKindrAngleAxis)\
    MACRO(kindr::RotationMatrixPD, DoubleKindrRotationMatrix)\
    MACRO(kindr::RotationVectorPD, DoubleKindrRotationVector)\
    MACRO(kindr::Velocity3D, DoubleKindrLinearVelocity)\
    MACRO(kindr::Acceleration3D, DoubleKindrLinearAcceleration)\
    MACRO(kindr::AngularAcceleration3D, DoubleKindrAngularAcceleration)\
    MACRO(kindr::Force3D, DoubleKindrForce)\
    MACRO(kindr::Torque3D, DoubleKindrTorque)\
    MACRO(kindr::VectorTypeless3D, DoubleKindrVector)

#define FOR_ALL_TYPES(MACRO) \
    FOR_PRIMITIVE_TYPES(MACRO)\
    FOR_EIGEN_TYPES(MACRO)\
    FOR_KINDR_TYPES(MACRO)

