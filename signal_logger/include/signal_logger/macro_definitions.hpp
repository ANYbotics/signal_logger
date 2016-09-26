/*
 * macro_definitions.hpp
 *
 *  Created on: Sep 26, 2016
 *      Author: Gabriel Hottiger
 */

#define DEFINITION_ADD_VAR(TYPE, NAME) \
    virtual void add##NAME(const TYPE& var, \
                           const std::string& name, \
                           const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, \
                           const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, \
                           bool update = LOGGER_DEFAULT_UPDATE) = 0;/*
                           */

#define FORWARD_ADD_VAR_EIGEN_NAMES(TYPE, NAME) \
   virtual void add##NAME(const TYPE& var, \
                          const LoggerBase::MatrixXstring& names, \
                          const std::string& group = std::string{LOGGER_DEFAULT_GROUP_NAME}, \
                          const std::string& unit = std::string{LOGGER_DEFAULT_UNIT}, \
                          bool update = LOGGER_DEFAULT_UPDATE) { \
     for(unsigned int r = 0; r < var.rows(); ++r) { \
       for(unsigned int c = 0; i < var.cols(); ++c) { \
         add##NAME(&((*var)(r,c)), &((*names)(r,c)), group, unit, update); \
       } \
     }/*
       */

#define FOR_PRIMITIVE_TYPES(MACRO) \
    MACRO(float, Float)\
    MACRO(double, Double)\
    MACRO(int, Int)\
    MACRO(short, Short)\
    MACRO(long, Long)\
    MACRO(char, Char)\
    MACRO(bool, Bool)\
    MACRO(LoggerBase::TimestampPair, TimeStamp)\


#define FOR_EIGEN_TYPES(MACRO) \
    MACRO(Eigen::MatrixXf, FloatEigenMatrix)\
    MACRO(Eigen::MatrixXd, DoubleEigenMatrix)\
    MACRO(Eigen::MatrixXi, IntegerEigenMatrix)\
    MACRO(LoggerBase::MatrixXs, ShortEigenMatrix)\
    MACRO(LoggerBase::MatrixXl, LongEigenMatrix)\
    MACRO(LoggerBase::MatrixXc, CharEigenMatrix)\
    MACRO(LoggerBase::MatrixXUc, UnsignedCharEigenMatrix)\
    MACRO(LoggerBase::MatrixXb, BoolEigenMatrix)\
    MACRO(Eigen::Vector3d, DoubleEigenVector3)

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
