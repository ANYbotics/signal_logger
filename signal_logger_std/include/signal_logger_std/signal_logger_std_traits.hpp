/*
 * signal_logger_std_traits.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gabrielhottiger
 */

#pragma once

#include <signal_logger/LogElementTypes.hpp>
#include <signal_logger_std/LogElementStd.hpp>
#include <fstream>
#include <typeinfo>

namespace signal_logger_std {

namespace traits {

#ifdef SILO_USE_KINDR
template<typename>
struct is_kindr_vector : std::false_type {};

template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector<kindr::Vector<PhysicalType_,PrimType_, Dimension_>> : std::true_type {};
#endif

template<typename>
struct is_eigen_angle_axis : std::false_type {};

template<typename PrimType_>
struct is_eigen_angle_axis<Eigen::AngleAxis<PrimType_>> : std::true_type {};



//! Default trait for primitive data types
template <typename ValueType_, typename Enable_ = void>
struct sls_traits
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

//! Trait for TimestampPair type
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_same<signal_logger::TimestampPair, ValueType_>::value>::type>
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
    (*headerStream) << name << "_s "
                    << sizeof(typename ValueType_::first_type) <<  " "
                    << values.size() << " "
                    << divider << " "
                    << isBufferLooping  << std::endl;

    (*headerStream) << name << "_ns "
                    << sizeof(typename ValueType_::second_type) <<  " "
                    << values.size() << " "
                    << divider << " "
                    << isBufferLooping  << std::endl;

    // Write data
    for (const auto & val : values)  { dataStream->write(reinterpret_cast<const char*>(&val.first), sizeof( typename ValueType_::first_type) ); }
    for (const auto & val : values)  { dataStream->write(reinterpret_cast<const char*>(&val.second), sizeof( typename ValueType_::second_type) ); }
  }
};

//! Trait for Eigen Matrices
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type>
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
    for (int r = 0; r<values.front().rows(); r++)  {
      for (int c=0; c<values.front().cols(); c++)  {
        (*headerStream) << std::string(name + "_" + std::to_string(r) + "_" + std::to_string(c)) << " "
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

//! Trait for Eigen Matrices
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
    for(auto suffix : {"_angle", "_x", "_y", "_z"}) {
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

#ifdef SILO_USE_KINDR
//! Trait for Kindr vectors length 3
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<is_kindr_vector<ValueType_>::value && ValueType_::Dimension == 3>::type>
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
    for(auto suffix : {"_x", "_y", "_z"}) {
      (*headerStream) << name << suffix << " "
                      << sizeof(typename ValueType_::Scalar) <<  " "
                      << values.size() << " "
                      << divider << " "
                      << isBufferLooping << std::endl;
    }

    // Write data
    for (int r = 0; r<3; r++)  {
      for (const auto & val : values) {
        dataStream->write(reinterpret_cast<const char*>( &( (const typename ValueType_::Scalar &) val(r) ) ), sizeof( typename ValueType_::Scalar ) );
      }
    }

  }
};

//! Trait for Kindr rotations
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::RotationBase<ValueType_>,ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const bool isBufferLooping )
  {
    std::vector<typename ValueType_::Implementation> implementationVector(values.size());
    for (unsigned int i = 0; i<values.size(); ++i) { implementationVector.at(i) = values.at(i).toImplementation(); }
    sls_traits<typename ValueType_::Implementation>::writeLogElementToStreams(headerStream, dataStream, implementationVector, name, divider, isBufferLooping);
  }
};
#endif

}


}
