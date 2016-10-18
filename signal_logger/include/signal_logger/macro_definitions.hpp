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
 *  Macro that defines pure virtual add function for every data type
 *  @param TYPE data type of the log variable
 *  @param NAME name of the data type added to add in function name
 */
#define ADD_VAR_DEFINITION(TYPE, NAME, ...) \
    /** Function definition to add variable of type TYPE to the logger. */ \
    /** @param var            pointer to log variable */ \
    /** @param name           name of the log variable*/ \
    /** @param group          logger group the variable belongs to*/ \
    /** @param unit           unit of the log variable*/ \
    /** @param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)*/ \
    /** @param action         log action of the log variable*/ \
    /** @param bufferSize     size of the buffer storing log elements*/ \
    /** @param bufferType     determines type of buffer*/ \
    virtual void add##NAME( const TYPE * const var,\
                            const std::string & name,\
                            const std::string & group       = LOG_ELEMENT_DEFAULT_GROUP_NAME, \
                            const std::string & unit        = LOG_ELEMENT_DEFAULT_UNIT, \
                            const std::size_t divider       = LOG_ELEMENT_DEFAULT_DIVIDER, \
                            const signal_logger::LogElementAction action = LOG_ELEMENT_DEFAULT_ACTION, \
                            const std::size_t bufferSize    = LOG_ELEMENT_DEFAULT_BUFFER_SIZE, \
                            const signal_logger::BufferType bufferType = LOG_ELEMENT_DEFAULT_BUFFER_TYPE) = 0; /*
                             */
/**
 *  Macro that implements a version of logging eigen matrices as their underlying types
 *  @param TYPE                 data type of the log variable
 *  @param NAME                 name of the data type added to add in function name
 *  @param UNDERLYING_TYPE      underlying data type
 *  @param UNDERLYING_TYPE_NAME name of the underlying data type
 */
#define ADD_EIGEN_VAR_AS_UNDERLYING_TYPE_IMPLEMENTATION(TYPE, NAME, UNDERLYING_TYPE, UNDERLYING_TYPE_NAME) \
    /** Function implementation to add eigen matrices as their underlying type to the logger. */ \
    /** @param var            pointer to log matrix */ \
    /** @param names          name of every entry of the matrix*/ \
    /** @param group          logger group the variable belongs to*/ \
    /** @param unit           unit of the log variable*/ \
    /** @param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)*/ \
    /** @param action         log action of the log variable*/ \
    /** @param bufferSize     size of the buffer storing log elements*/ \
    /** @param bufferType     determines type of buffer*/ \
    void add##NAME( const TYPE * const var, \
                    const signal_logger::MatrixXstring & names, \
                    const std::string & group, \
                    const std::string & unit, \
                    const std::size_t divider, \
                    const signal_logger::LogElementAction action, \
                    const std::size_t bufferSize, \
                    const signal_logger::BufferType bufferType) \
    { \
      for(std::size_t i = 0; i < var->size(); ++i) { \
        add##UNDERLYING_TYPE_NAME(static_cast<const UNDERLYING_TYPE * const>(var->data() + i), static_cast<std::string>(*(names.data() + i)), group, unit, divider, action, bufferSize, bufferType); \
      } \
    } /*
     */

/**
 *  Macro that implements a version of logging eigen matrices as their underlying types
 *  @param TYPE data type of the log variable
 *  @param NAME name of the data type added to add in function name
 */
#define ADD_VAR_TEMPLATE_SPECIFICATIONS(TYPE, NAME, ...) \
    /** Template specification of the add function. Maps add to the corresponding addTYPE function. */ \
    /** @param var            pointer to log var */ \
    /** @param names          name of every entry of the matrix*/ \
    /** @param group          logger group the variable belongs to*/ \
    /** @param unit           unit of the log variable*/ \
    /** @param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)*/ \
    /** @param action         log action of the log variable*/ \
    /** @param bufferSize     size of the buffer storing log elements*/ \
    /** @param bufferType     determines type of buffer*/ \
    template < > \
    inline void SignalLoggerBase::add<TYPE>(  const TYPE * const var, \
                                              const std::string & name, \
                                              const std::string & group, \
                                              const std::string & unit, \
                                              const std::size_t divider, \
                                              const signal_logger::LogElementAction action, \
                                              const std::size_t bufferSize, \
                                              const signal_logger::BufferType bufferType) \
    { \
      if(logElements_.find(name) != logElements_.end()) { \
        printf("A signal with the same name %s is already logged. Overwrite.", name.c_str()); \
      } \
      add##NAME(var, name, group, unit, divider, action, bufferSize, bufferType); \
    }/*
    */

/**
 *  Macro that empty implementation of the  add function for every data type
 *  @param TYPE data type of the log variable
 *  @param NAME name of the data type added to add in function name
 */
#define ADD_NONE_VAR_IMPLEMENTATION(TYPE, NAME, ...) \
    /** Function definition to add variable of type TYPE to the logger. */ \
    /** @param var            pointer to log variable */ \
    /** @param name           name of the log variable*/ \
    /** @param group          logger group the variable belongs to*/ \
    /** @param unit           unit of the log variable*/ \
    /** @param divider        divider is defining the update frequency of the logger element (ctrl_freq/divider)*/ \
    /** @param action         log action of the log variable*/ \
    /** @param bufferSize     size of the buffer storing log elements*/ \
    /** @param bufferType     determines type of buffer*/ \
    virtual void add##NAME( const TYPE * const var,\
                            const std::string & name,\
                            const std::string & group, \
                            const std::string & unit, \
                            const std::size_t divider, \
                            const signal_logger::LogElementAction action, \
                            const std::size_t bufferSize, \
                            const signal_logger::BufferType bufferType) { } /*
                                                                             */

/**
 *  Calls macro for all supported primitive types
 *  @param MACRO macro the shall be applied to every data type
 */
#define FOR_PRIMITIVE_TYPES(MACRO) \
    MACRO(float, Float)\
    MACRO(double, Double)\
    MACRO(int, Int)\
    MACRO(short, Short)\
    MACRO(long, Long)\
    MACRO(char, Char)\
    MACRO(unsigned char, UnsignedChar)\
    MACRO(bool, Bool)\
    MACRO(signal_logger::TimestampPair, TimeStamp)

/**
 *  Calls macro for all supported eigen types
 *  @param MACRO macro the shall be applied to every data type
 */
#define FOR_EIGEN_TYPES(MACRO) \
    MACRO(signal_logger::Vector3d, DoubleEigenVector3, double, Double) \
    MACRO(signal_logger::MatrixXf, FloatEigenMatrix, float, Float)\
    MACRO(signal_logger::MatrixXd, DoubleEigenMatrix, double, Double)\
    MACRO(signal_logger::MatrixXi, IntegerEigenMatrix, int, Int)\
    MACRO(signal_logger::MatrixXs, ShortEigenMatrix, short, Short)\
    MACRO(signal_logger::MatrixXl, LongEigenMatrix, long, Long)\
    MACRO(signal_logger::MatrixXc, CharEigenMatrix, char,Char)\
    MACRO(signal_logger::MatrixXUc, UnsignedCharEigenMatrix, unsigned char,  UnsignedChar)\
    MACRO(signal_logger::MatrixXb, BoolEigenMatrix, bool, Bool)


#ifdef SILO_USE_KINDR
/**
 *  Calls macro for all supported kindr vector types
 *  @param MACRO macro the shall be applied to every data type
 */
#define FOR_KINDR_VECTORS(MACRO) \
    MACRO(signal_logger::KindrLinearVelocityD, DoubleKindrLinearVelocity)\
    MACRO(signal_logger::KindrLinearAccelerationD, DoubleKindrLinearAcceleration)\
    MACRO(signal_logger::KindrForceD, DoubleKindrForce)\
    MACRO(signal_logger::KindrTorqueD, DoubleKindrTorque)\
    MACRO(signal_logger::KindrVectorD, DoubleKindrVector)

/**
 *  Calls macro for all supported kindr vector types at position
 *  @param MACRO macro the shall be applied to every data type
 */
#define FOR_KINDR_VECTOR_AT_POSITION(MACRO) \
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearVelocityD>, DoubleKindrLinearVelocityAtPosition)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrLinearAccelerationD>, DoubleKindrLinearAccelerationAtPosition)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrForceD>, DoubleKindrForceAtPosition)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrTorqueD>, DoubleKindrTorqueAtPosition)\
    MACRO(signal_logger::KindrVectorAtPosition<signal_logger::KindrVectorD>, DoubleKindrVectorAtPosition)

/**
 *  Calls macro for all supported kindr types
 *  @param MACRO macro the shall be applied to every data type
 */
#define FOR_KINDR_TYPES(MACRO) \
    MACRO(signal_logger::KindrPositionD, DoubleKindrPosition)\
    MACRO(signal_logger::KindrRotationQuaternionD, DoubleKindrRotationQuaternion)\
    MACRO(signal_logger::KindrEulerAnglesZyxD, DoubleKindrEulerAnglesZyx)\
    MACRO(signal_logger::KindrAngularVelocityD, DoubleKindrLocalAngularVelocity)\
    MACRO(signal_logger::KindrAngleAxisD, DoubleKindrAngleAxis)\
    MACRO(signal_logger::KindrRotationMatrixD, DoubleKindrRotationMatrix)\
    MACRO(signal_logger::KindrRotationVectorD, DoubleKindrRotationVector)\
    MACRO(signal_logger::KindrAngularAccelerationD, DoubleKindrAngularAcceleration)\
    FOR_KINDR_VECTORS(MACRO)\
    FOR_KINDR_VECTOR_AT_POSITION(MACRO)
#endif

/**
 *  Calls macro for all supported types
 *  @param MACRO macro the shall be applied to every data type
 */
#ifdef SILO_USE_KINDR
#define FOR_ALL_TYPES(MACRO) \
    FOR_PRIMITIVE_TYPES(MACRO)\
    FOR_EIGEN_TYPES(MACRO)\
    FOR_KINDR_TYPES(MACRO)
#else
#define FOR_ALL_TYPES(MACRO) \
    FOR_PRIMITIVE_TYPES(MACRO)\
    FOR_EIGEN_TYPES(MACRO)
#endif
