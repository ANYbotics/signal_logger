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
    for (const auto & val : values)  {
      dataStream->write(reinterpret_cast<const char*>(&val),sizeof(val));
      //      (*dataStream) << val<< " ";
    }
    (*dataStream) << std::endl;
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
    (*headerStream) << name << "_s " << sizeof(typename ValueType_::first_type) <<  " " << values.size() << " " << divider << std::endl;
    (*headerStream) << name << "_ns " << sizeof(typename ValueType_::second_type) <<  " " << values.size() << " " << divider << std::endl;
    for (const auto & val : values)  {
      dataStream->write(reinterpret_cast<const char*>(&val.first),sizeof(val.first));
      //      (*dataStream) << val.first<< " ";
    }
    (*dataStream) << std::endl;
    for (const auto & val : values)  {
      dataStream->write(reinterpret_cast<const char*>(&val.second),sizeof(val.second));
      //      (*dataStream) << val.second<< " ";
    }
    (*dataStream) << std::endl;
  }
};

//! Trait for Eigen Matrices length 3
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type>
{
  static void writeLogElementToStreams(std::stringstream* headerStream,
                                       std::stringstream* dataStream,
                                       const std::vector<ValueType_> & values,
                                       const std::string & name,
                                       const std::size_t divider )
  {

  }
  //
  //  //! Write all entries as underlying type
  //  static void writeToFile(std::ofstream * file, const std::vector<typename ValueType_::Scalar> & data, std::size_t no_rows, std::size_t no_cols) {
  //    for (int r=0; r<no_rows; r++)  {
  //      for (int c=0; c<no_cols; c++)  {
  //        for (int i = 0; i < data.size()/(no_cols*no_rows); i++)  {
  ////            (*file) << data.at(i*no_cols*no_rows + r*no_cols + c) << " ";
  //            typename ValueType_::Scalar val = data.at(i*no_cols*no_rows + r*no_cols + c);
  //            file->write(reinterpret_cast<const char*>(&val),sizeof(val));
  //        }
  //        (*file) << std::endl;
  //      }
  //    }
  //  }
  //
  //  //! Header appends _r_c to the matrix name
  //  static void writeHeader(std::ofstream * file, std::string name, const std::size_t nrPoints, const double updateFrequency, std::size_t no_rows, std::size_t no_cols) {
  //    for (int r=0; r<no_rows; r++)  {
  //      for (int c=0; c<no_cols; c++)  {
  //        (*file) << std::string(name + "_" + std::to_string(r) + "_" + std::to_string(c))
  //                << " " << sizeof(typename ValueType_::Scalar) <<  " " << nrPoints/(no_rows * no_cols) << " " << updateFrequency << std::endl;
  //      }
  //    }
  //  }
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

  }
  //  static void writeToFile(std::ofstream * file, const std::vector<ValueType_> & data) {
  //    for (const auto & entry : data)  {
  ////      (*file) << entry.x()<< " ";
  //      file->write(reinterpret_cast<const char*>(&entry.toImplementation()(0)),sizeof(entry.x()));
  //    }
  //    (*file)<<std::endl;
  //    for (const auto & entry : data)  {
  ////      (*file) << entry.y()<< " ";
  //      file->write(reinterpret_cast<const char*>(&entry.toImplementation()(1)),sizeof(entry.y()));
  //    }
  //    (*file)<<std::endl;
  //    for (const auto & entry : data)  {
  ////      (*file) << entry.z()<< " ";
  //      file->write(reinterpret_cast<const char*>(&entry.toImplementation()(2)),sizeof(entry.z()));
  //    }
  //    (*file)<<std::endl;
  //  }
  //
  //  //! Header append x, y and z to data name
  //  static void writeHeader(std::ofstream * file, std::string name, const std::size_t nrPoints, const double updateFrequency) {
  //    (*file) << name << "_x " << sizeof(typename ValueType_::Scalar) <<  " " << nrPoints << " " << updateFrequency << std::endl;
  //    (*file) << name << "_y " << sizeof(typename ValueType_::Scalar) <<  " " << nrPoints << " " << updateFrequency << std::endl;
  //    (*file) << name << "_z " << sizeof(typename ValueType_::Scalar) <<  " " << nrPoints << " " << updateFrequency << std::endl;
  //  }
};

}

}
