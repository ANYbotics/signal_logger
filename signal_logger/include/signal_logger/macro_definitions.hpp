/*
 * macro_definitions.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */
#pragma once

#define ADD_VAR_DEFINITION(TYPE, NAME, ...) \
    virtual void add##NAME( const TYPE & var, \
                            const std::string& name, \
                            const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, \
                            const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, \
                            const std::size_t divider = std::size_t{LOGGER_DEFAULT_DIVIDER}, \
                            const signal_logger::LogElementInterface::LogElementAction & action = LOGGER_DEFAULT_ACTION, \
                            const std::size_t bufferSize = std::size_t{LOGGER_DEFAULT_BUFFER_SIZE}, \
                            const bool bufferLooping = LOGGER_DEFAULT_BUFFER_LOOPING ) = 0; /*
                             */

#define ADD_EIGEN_VAR_AS_UNDERLYING_TYPE_IMPLEMENTATION(TYPE, NAME, UNDERLYING_TYPE, UNDERLYING_TYPE_NAME) \
    void add##NAME( const TYPE & var, \
                    const signal_logger::MatrixXstring& names, \
                    const std::string& group, \
                    const std::string& unit, \
                    const std::size_t divider, \
                    const signal_logger::LogElementInterface::LogElementAction & action, \
                    const std::size_t bufferSize, \
                    const bool bufferLooping) \
    { \
      for (int r=0; r<static_cast<signal_logger::MatrixXstring>(names).rows(); r++)  { \
        for (int c=0; c<static_cast<signal_logger::MatrixXstring>(names).cols(); c++)  { \
          add##UNDERLYING_TYPE_NAME((UNDERLYING_TYPE)(var(r,c)), static_cast<std::string>(names(r,c)), group, unit, divider, action, bufferSize, bufferLooping); \
        } \
      } \
    } /*
     */

#define ADD_VAR_TEMPLATE_SPECIFICATIONS(TYPE, NAME, ...) \
    template < > \
    void SignalLoggerBase::add( const TYPE & var, \
                                const std::string& name, \
                                const std::string& group, \
                                const std::string& unit, \
                                const std::size_t divider, \
                                const signal_logger::LogElementInterface::LogElementAction & action, \
                                const std::size_t bufferSize, \
                                const bool bufferLooping) \
    { \
      if(logElements_.find(name) != logElements_.end()) { \
        printf("A signal with the same name %s is already logged. Overwrite.", name.c_str()); \
      } \
      add##NAME(var, name, group, unit, divider, action, bufferSize, bufferLooping); \
    }

#define FOR_PRIMITIVE_TYPES(MACRO) \
    MACRO(float, Float)\
    MACRO(double, Double)\
    MACRO(int, Int)\
    MACRO(short, Short)\
    MACRO(long, Long)\
    MACRO(char, Char)\
    MACRO(bool, Bool)\
    MACRO(signal_logger::TimestampPair, TimeStamp)\

#define FOR_EIGEN_TYPES(MACRO) \
    MACRO(signal_logger::Vector3d, DoubleEigenVector3, double, Double) \
    MACRO(signal_logger::MatrixXf, FloatEigenMatrix, float, Float)\
    MACRO(signal_logger::MatrixXd, DoubleEigenMatrix, double, Double)\
    MACRO(signal_logger::MatrixXi, IntegerEigenMatrix, int, Int)\
    MACRO(signal_logger::MatrixXs, ShortEigenMatrix, short, Short)\
    MACRO(signal_logger::MatrixXl, LongEigenMatrix, long, Long)\
    MACRO(signal_logger::MatrixXc, CharEigenMatrix, char,Char)\
    MACRO(signal_logger::MatrixXUc, UnsignedCharEigenMatrix, char,  Char)\
    MACRO(signal_logger::MatrixXb, BoolEigenMatrix, bool, Bool)

#define FOR_KINDR_TYPES(MACRO) \
    MACRO(signal_logger::KindrPositionD, DoubleKindrPosition)\
    MACRO(signal_logger::KindrRotationQuaternionD, DoubleKindrRotationQuaternion)\
    MACRO(signal_logger::KindrEulerAnglesZyxD, DoubleKindrEulerAnglesZyx)\
    MACRO(signal_logger::KindrAngularVelocityD, DoubleKindrLocalAngularVelocity)\
    MACRO(signal_logger::KindrAngleAxisD, DoubleKindrAngleAxis)\
    MACRO(signal_logger::KindrRotationMatrixD, DoubleKindrRotationMatrix)\
    MACRO(signal_logger::KindrRotationVectorD, DoubleKindrRotationVector)\
    MACRO(signal_logger::KindrLinearVelocityD, DoubleKindrLinearVelocity)\
    MACRO(signal_logger::KindrLinearAccelerationD, DoubleKindrLinearAcceleration)\
    MACRO(signal_logger::KindrAngularAccelerationD, DoubleKindrAngularAcceleration)\
    MACRO(signal_logger::KindrForceD, DoubleKindrForce)\
    MACRO(signal_logger::KindrTorqueD, DoubleKindrTorque)\
    MACRO(signal_logger::KindrVectorD, DoubleKindrVector)

#define FOR_ALL_TYPES(MACRO) \
    FOR_PRIMITIVE_TYPES(MACRO)\
    FOR_EIGEN_TYPES(MACRO)\
    FOR_KINDR_TYPES(MACRO)

