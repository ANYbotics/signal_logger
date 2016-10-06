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

namespace signal_logger_std {

namespace traits {

//! Default trait for primitive data types
template <typename ValueType_, typename Enable_ = void>
struct sls_traits
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider )
  {
    (*headerStream) << name << " " << sizeof(ValueType_) <<  " " << values.size() << " " << divider << std::endl;
    for (const auto & val : values)  { dataStream->write(reinterpret_cast<const char*>(&val),sizeof(val)); }
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
                                       const std::size_t divider )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    (*headerStream) << name << "_s " << sizeof(typename ValueType_::first_type) <<  " " << values.size() << " " << divider << std::endl;
    (*headerStream) << name << "_ns " << sizeof(typename ValueType_::second_type) <<  " " << values.size() << " " << divider << std::endl;

    // Write data
    for (const auto & val : values)  { dataStream->write(reinterpret_cast<const char*>(&val.first),sizeof(val.first)); }
    for (const auto & val : values)  { dataStream->write(reinterpret_cast<const char*>(&val.second),sizeof(val.second)); }
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
                                       const std::size_t divider )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    for (int r = 0; r<values.front().rows(); r++)  {
      for (int c=0; c<values.front().cols(); c++)  {
        (*headerStream) << std::string(name + "_" + std::to_string(r) + "_" + std::to_string(c)) << " "
            << sizeof(typename ValueType_::Scalar) << " " << values.size() << " " << divider << std::endl;
      }
    }

    // Write data
    for (int r = 0; r<values.front().rows(); r++)  {
      for (int c=0; c<values.front().cols(); c++)  {
        for (const auto & val : values) {
          dataStream->write(reinterpret_cast<const char*>(&(val(r,c))),sizeof(val(r,c)));
        }
      }
    }
  }
};

//! Trait for Kindr vectors length 3
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::VectorBase<ValueType_>, ValueType_>::value && ValueType_::Dimension == 3>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider )
  {
    // Check for nonzero size
    if(values.size() == 0)
      return;

    // Write headers
    for(auto suffix : {"_x", "_y", "_z"}) {
      (*headerStream) << name << suffix << sizeof(typename ValueType_::Scalar) <<  " " << values.size() << " " << divider << std::endl;
    }

    // Write data
    for (int r = 0; r<3; r++)  {
      for (const auto & val : values) {
        dataStream->write(reinterpret_cast<const char*>(&val.toImplementation()(r)),sizeof(val.toImplementation()(r)));
      }
    }

  }
};

}

}
