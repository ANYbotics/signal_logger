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
template<typename ValueType_, typename Enable_ = void> struct sls_traits;

/*******************************
 * Specializations: core types *
 *******************************/
template<typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_arithmetic<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping)
  {
    (*headerStream) << name << " "
        << sizeof(ValueType_) <<  " "
        << values.size() << " "
        << divider << " "
        << isBufferLooping << std::endl;

    for (const auto & val : values)  { dataStream->write(reinterpret_cast<const char*>(&val), sizeof(ValueType_) ); }
  }
};
/********************************/

/*******************************
 * Specializations: enum types *
 *******************************/
template<typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_enum<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping)
  {
    std::vector<typename std::underlying_type<ValueType_>::type > underlyingVector(values.size());
    std::transform(values.begin(), values.end(), underlyingVector.begin(), [](ValueType_ t) { return static_cast<typename std::underlying_type<ValueType_>::type >(t); });
    sls_traits< typename std::underlying_type<ValueType_>::type >::writeLogElementToStreams(headerStream, dataStream, underlyingVector, name, divider, isBufferLooping);
  }
};
/********************************/

/***************************************************
 * Specializations: Time stamp pair                *
 ***************************************************/
template <>
struct sls_traits<signal_logger::TimestampPair>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<signal_logger::TimestampPair> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    (*headerStream) << name << "_s "
        << sizeof(typename signal_logger::TimestampPair::first_type) <<  " "
        << values.size() << " "
        << divider << " "
        << isBufferLooping  << std::endl;

    (*headerStream) << name << "_ns "
        << sizeof(typename signal_logger::TimestampPair::second_type) <<  " "
        << values.size() << " "
        << divider << " "
        << isBufferLooping  << std::endl;

    // Write data
    for (const auto & val : values)  { dataStream->write(reinterpret_cast<const char*>(&val.first), sizeof( typename signal_logger::TimestampPair::first_type) ); }
    for (const auto & val : values)  { dataStream->write(reinterpret_cast<const char*>(&val.second), sizeof( typename signal_logger::TimestampPair::second_type) ); }
  }
};
/********************************/

/********************************
 * Specializations: eigen types *
 ********************************/
template<>
struct sls_traits<Eigen::Vector3d>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<Eigen::Vector3d> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    for(auto suffix : {"_x", "_y", "_z"}) {
      (*headerStream) << name << suffix << " "
          << sizeof(double) <<  " "
          << values.size() << " "
          << divider << " "
          << isBufferLooping << std::endl;
    }

    // Write data
    for (int r = 0; r<3; r++)  {
      for (const auto & val : values) {
        dataStream->write(reinterpret_cast<const char*>( &val(r) ), sizeof( double ) );
      }
    }

  }
};

template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<is_eigen_quaternion<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    for(auto suffix : {"_w", "_x", "_y", "_z", }) {
      (*headerStream) << name << suffix << " "
          << sizeof(typename ValueType_::Scalar) <<  " "
          << values.size() << " "
          << divider << " "
          << isBufferLooping << std::endl;
    }

    // Write data
    for (const auto & val : values) {
      dataStream->write(reinterpret_cast<const char*>( &( (const typename ValueType_::Scalar &) val.w() ) ) , sizeof( typename ValueType_::Scalar ) );
    }
    for (const auto & val : values) {
      dataStream->write(reinterpret_cast<const char*>( &( (const typename ValueType_::Scalar &) val.x() ) ), sizeof( typename ValueType_::Scalar ) );
    }
    for (const auto & val : values) {
      dataStream->write(reinterpret_cast<const char*>( &( (const typename ValueType_::Scalar &) val.y() ) ), sizeof( typename ValueType_::Scalar ) );
    }
    for (const auto & val : values) {
      dataStream->write(reinterpret_cast<const char*>( &( (const typename ValueType_::Scalar &) val.z() ) ), sizeof( typename ValueType_::Scalar ) );
    }
  }
};

template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<is_eigen_angle_axis<ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    for(auto suffix : {"_angle", "_axis_x", "_axis_y", "_axis_z"}) {
      (*headerStream) << name << suffix << " "
          << sizeof(typename ValueType_::Scalar) <<  " "
          << values.size() << " "
          << divider << " "
          << isBufferLooping << std::endl;
    }

    // Write data
    for (const auto & val : values) {
      dataStream->write(reinterpret_cast<const char*>( &( (const typename ValueType_::Scalar &) val.angle() ) ), sizeof( typename ValueType_::Scalar ) );
    }

    for (int r = 0; r<3; r++)  {
      for (const auto & val : values) {
        dataStream->write(reinterpret_cast<const char*>( &( (const typename ValueType_::Scalar &) val.axis()(r) ) ), sizeof( typename ValueType_::Scalar ) );
      }
    }
  }
};

template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if< is_eigen_matrix_excluding_vector3<ValueType_>::value >::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    std::string nameWithSuffix;
    for (int r = 0; r<values.front().rows(); r++)  {
      for (int c=0; c<values.front().cols(); c++)  {
        nameWithSuffix = name;
        if(values.front().rows() > 1) nameWithSuffix += ("_" + std::to_string(r));
        if(values.front().cols() > 1) nameWithSuffix += ("_" + std::to_string(c));

        (*headerStream) << nameWithSuffix << " "
            << sizeof(typename ValueType_::Scalar) << " "
            << values.size() << " "
            << divider << " "
            << isBufferLooping << std::endl;
      }
    }

    // Write data
    for (int r = 0; r<values.front().rows(); r++)  {
      for (int c=0; c<values.front().cols(); c++)  {
        for (const auto & val : values) {
          dataStream->write(reinterpret_cast<const char*>( &( val(r,c) ) ), sizeof( typename ValueType_::Scalar ));
        }
      }
    }
  }
};
/********************************/

#ifdef SILO_USE_KINDR
/********************************
 * Specializations: kindr types *
 ********************************/
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if< std::is_base_of<kindr::RotationBase<ValueType_>,ValueType_>::value ||
                                                       std::is_base_of<kindr::RotationDiffBase<ValueType_>,ValueType_>::value ||
                                                       is_kindr_vector3<ValueType_>::value >::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping )
  {
    std::vector<typename ValueType_::Implementation> implementationVector(values.size());
    std::transform(values.begin(), values.end(), implementationVector.begin(), [](ValueType_ t) { return t.toImplementation(); });
    sls_traits<typename ValueType_::Implementation>::writeLogElementToStreams(headerStream, dataStream, implementationVector, name, divider, isBufferLooping);
  }
};
/********************************/

/***************************************************
 * Specializations: kindr vector at position types *
 ***************************************************/
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<is_kindr_vector_at_position<ValueType_>::value>::type> {
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    for(auto suffix : { "_vector_in_"  + values.front().vectorFrame + "_frame_x",
                        "_vector_in_"  + values.front().vectorFrame + "_frame_x",
                        "_vector_in_"  + values.front().vectorFrame + "_frame_x",
                        "_at_position_in_"  + values.front().positionFrame + "_frame_x",
                        "_at_position_in_"  + values.front().positionFrame + "_frame_y",
                        "_at_position_in_"  + values.front().positionFrame + "_frame_z"})
    {
      (*headerStream) << name << suffix << " "
          << sizeof(double) <<  " "
          << values.size() << " "
          << divider << " "
          << isBufferLooping << std::endl;
    }

    // Write data
    for (int r = 0; r<3; r++)  {
      for (const auto & val : values) {
        dataStream->write(reinterpret_cast<const char*>( &( (const double &) val.vector(r) ) ), sizeof( double ) );
      }
    }
    for (int r = 0; r<3; r++)  {
      for (const auto & val : values) {
        dataStream->write(reinterpret_cast<const char*>( &( (const double &) val.position(r) ) ), sizeof( double ) );
      }
    }
  }
};
#endif

}


}
