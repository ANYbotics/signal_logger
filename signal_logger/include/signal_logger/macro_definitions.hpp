/*
 * macro_definitions.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// signal logger
#include "signal_logger/LogElementTypes.hpp"

/**
 *  Macro that implements a version of logging eigen matrices as their underlying types
 *  @param TYPE data type of the log variable
 */
#define ADD_VAR_TEMPLATE_EXPLICIT_INSTANTIATION(TYPE) \
    template void signal_logger::add<TYPE>(   const TYPE & var, \
                                              const std::string & name, \
                                              const std::string & group       = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_GROUP_NAME, \
                                              const std::string & unit        = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_UNIT, \
                                              const std::size_t divider       = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_DIVIDER, \
                                              const LogElementAction action   = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_ACTION, \
                                              const std::size_t bufferSize    = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_SIZE, \
                                              const BufferType bufferType     = signal_logger::SignalLoggerBase::LOG_ELEMENT_DEFAULT_BUFFER_TYPE);

/**
 *  Calls macro for all supported non-kindr types
 *  @param MACRO macro the shall be applied to every data type
 */
#define FOR_NON_KINDR_TYPES(MACRO)\
    MACRO(float)\
    MACRO(double)\
    MACRO(int)\
    MACRO(short)\
    MACRO(long)\
    MACRO(char)\
    MACRO(unsigned char)\
    MACRO(bool)\
    MACRO(signal_logger::TimestampPair)\
    MACRO(signal_logger::Vector3d)\
    MACRO(signal_logger::MatrixXf)\
    MACRO(signal_logger::MatrixXd)\
    MACRO(signal_logger::MatrixXi)\
    MACRO(signal_logger::MatrixXs)\
    MACRO(signal_logger::MatrixXl)\
    MACRO(signal_logger::MatrixXc)\
    MACRO(signal_logger::MatrixXUc)\
    MACRO(signal_logger::MatrixXb)


#ifdef SILO_USE_KINDR
/**
 *  Calls macro for all supported kindr types
 *  @param MACRO macro the shall be applied to every data type
 */
#define FOR_KINDR_TYPES(MACRO) \
    MACRO(signal_logger::KindrPositionD)\
    MACRO(signal_logger::KindrRotationQuaternionD)\
    MACRO(signal_logger::KindrEulerAnglesZyxD)\
    MACRO(signal_logger::KindrAngularVelocityD)\
    MACRO(signal_logger::KindrAngleAxisD)\
    MACRO(signal_logger::KindrRotationMatrixD)\
    MACRO(signal_logger::KindrRotationVectorD)\
    MACRO(signal_logger::KindrAngularAccelerationD)\
    MACRO(signal_logger::KindrLinearVelocityD)\
    MACRO(signal_logger::KindrLinearAccelerationD)\
    MACRO(signal_logger::KindrForceD)\
    MACRO(signal_logger::KindrTorqueD)\
    MACRO(signal_logger::KindrVectorD)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearVelocityD>)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearAccelerationD>)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrForceD>)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrTorqueD>)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrVectorD>)

#endif

/**
 *  Calls macro for all supported types
 *  @param MACRO macro the shall be applied to every data type
 */
#ifdef SILO_USE_KINDR
#define FOR_ALL_TYPES(MACRO) \
    FOR_NON_KINDR_TYPES(MACRO)\
    FOR_KINDR_TYPES(MACRO)
#else
#define FOR_ALL_TYPES(MACRO) \
    FOR_NON_KINDR_TYPES(MACRO)
#endif
