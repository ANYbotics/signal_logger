/*
 * signal_logger_std_traits.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gabrielhottiger
 */

#pragma once
#include <signal_logger_core/signal_logger_traits.hpp>
#include <signal_logger_core/LogElementTypes.hpp>
#include <signal_logger_std/LogElementStd.hpp>
#include <fstream>
#include <typeinfo>
#include <message_logger/message_logger.hpp>

namespace signal_logger_std {

namespace traits {

using namespace signal_logger::traits;

// generic interface
template<typename ValueType_, typename ContainerType_, typename Enable_ = void> struct sls_traits;

/*******************************
 * Specializations: core types *
 *******************************/
template<typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<std::is_arithmetic<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })
  {
    std::cout<<"New Element "<<name<<std::endl;
    (*headerStream) << name     << " " << sizeof(ValueType_) << " " << values.size() << " "
                    << divider  << " " << isBufferLooping         << std::endl;
    for (const auto & val : values) { std::cout<<getType(val)<<""<<std::endl; dataStream->write(reinterpret_cast<const char*>(&getType(val)), sizeof(ValueType_) ); }

  }
};
/********************************/

/*******************************
 * Specializations: enum types *
 *******************************/
template<typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })

  {
    auto getUnderlyingType = [getType](const ContainerType_ & v) { return static_cast<typename std::underlying_type<ValueType_>::type >(getType(v)); };
    sls_traits<typename std::underlying_type<ValueType_>::type, ContainerType_>::writeLogElementToStreams(
        headerStream, dataStream, values, name, divider, isBufferLooping, getUnderlyingType);
  }
};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <typename ContainerType_>
struct sls_traits<signal_logger::TimestampPair, ContainerType_>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const signal_logger::TimestampPair&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })

  {
    auto getSeconds = [getType](const ContainerType_ & v) { return getType(v).first; };
    sls_traits<typename signal_logger::TimestampPair::first_type, ContainerType_>::writeLogElementToStreams(
        headerStream, dataStream, values, name + "_s", divider, isBufferLooping, getSeconds);

    auto getNanoSeconds = [getType](const ContainerType_ & v) { return getType(v).second; };
    sls_traits<typename signal_logger::TimestampPair::second_type, ContainerType_>::writeLogElementToStreams(
        headerStream, dataStream, values, name + "_ns", divider, isBufferLooping, getNanoSeconds);
  }
};
/********************************/

/********************************
 * Specializations: eigen types *
 ********************************/
template<typename ContainerType_>
struct sls_traits<Eigen::Vector3d, ContainerType_>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const Eigen::Vector3d&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })
  {
    std::vector<std::string> suffix = {"_x", "_y", "_z"};

    for (int r = 0; r<3; r++)  {
      auto getData = [r, getType](const ContainerType_ & v) { return getType(v)(r); };
      sls_traits<double, ContainerType_>::writeLogElementToStreams(
          headerStream, dataStream, values, name + suffix.at(r), divider, isBufferLooping, getData);
    }

  }
};

template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })

  {
    auto getW = [getType](const ContainerType_ & v) { return getType(v).w(); };
    sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
        headerStream, dataStream, values, name + "_w", divider, isBufferLooping, getW);

    std::vector<std::string> suffix = {"_x", "_y", "_z"};
    for (int r = 0; r<3; r++)  {
      auto getXYZ = [r, getType](const ContainerType_ & v) { return getType(v).vec()(r); };
      sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
          headerStream, dataStream, values, name + suffix.at(r), divider, isBufferLooping, getXYZ);
    }
  }
};

template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })
  {
    auto getAngle = [getType](const ContainerType_ & v) { return getType(v).angle(); };
    sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
        headerStream, dataStream, values, name + "_angle", divider, isBufferLooping, getAngle);

    std::vector<std::string> suffix = {"_x", "_y", "_z"};
    for (int r = 0; r<3; r++)  {
      auto getXYZ = [r,getType](const ContainerType_ & v) { return getType(v).axis()(r); };
      sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
          headerStream, dataStream, values, name + suffix.at(r), divider, isBufferLooping, getXYZ);
    }

  }
};

template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if< is_eigen_matrix_excluding_vector3<ValueType_>::value >::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })
  {
    std::string nameWithSuffix;
    for (int r = 0; r < values.front().rows(); ++r)
    {
      for (int c = 0; c < values.front().cols(); ++c)
      {
        nameWithSuffix = name;
        if(values.front().rows() > 1) nameWithSuffix += ("_" + std::to_string(r));
        if(values.front().cols() > 1) nameWithSuffix += ("_" + std::to_string(c));
        auto getData = [r,c,getType](const ContainerType_ & v) { return getType(v)(r,c); };
        sls_traits<typename ValueType_::Scalar, ContainerType_>::writeLogElementToStreams(
            headerStream, dataStream, values, nameWithSuffix, divider, isBufferLooping, getData);
      }
    }
  }
};
/********************************/

#ifdef SILO_USE_KINDR
/********************************
 * Specializations: kindr types *
 ********************************/
template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if< std::is_base_of<kindr::RotationBase<ValueType_>,ValueType_>::value ||
                                                       std::is_base_of<kindr::RotationDiffBase<ValueType_>,ValueType_>::value ||
                                                       is_kindr_vector3<ValueType_>::value >::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })
  {
    auto getData = [getType](const ContainerType_ & v) { return getType(v).toImplementation(); };
    sls_traits<typename ValueType_::Implementation, ContainerType_>::writeLogElementToStreams(
        headerStream, dataStream, values, name, divider, isBufferLooping, getData);
  }
};
/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_, typename ContainerType_>
struct sls_traits<ValueType_, ContainerType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ContainerType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping,
                                       const std::function<const ValueType_&(const ContainerType_&)> & getType = [](const ContainerType_ & v) { return v; })
  {
    std::string vectorFrame = values.size() ? values.front().vectorFrame : "unknown";
    auto getVector = [getType](const ContainerType_ & v) { return getType(v).vector; };
    sls_traits<typename ValueType_::VectorType, ContainerType_>::writeLogElementToStreams(
        headerStream, dataStream, values, name + "_vector_in_" + vectorFrame + "_frame", divider, isBufferLooping, getVector);

    std::string positionFrame = values.size() ? values.front().positionFrame : "unknown";
    auto getPosition = [getType](const ContainerType_ & v) { return getType(v).position; };
    sls_traits<typename signal_logger::KindrPositionD, ContainerType_>::writeLogElementToStreams(
        headerStream, dataStream, values, name + "_at_position_in_" + positionFrame + "_frame", divider, isBufferLooping, getPosition);
  }
};
#endif

} /* namespace traits */

} /* namespace signal_logger_std */

#ifdef SILO_STD_TRAITS_PLUGIN
#include SILO_STD_TRAITS_PLUGIN
#endif
