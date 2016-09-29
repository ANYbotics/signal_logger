/*
 * signal_logger_std_traits.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gabrielhottiger
 */

#pragma once

#include <signal_logger/LogElementTypes.hpp>
#include <fstream>

namespace signal_logger_std {

namespace traits {

//! Default trait for primitive data types
template <typename ValueType_, typename Enable_ = void>
struct sls_traits
{
  //! Writes the "simple" data in binary form
  static void writeToFile(std::ofstream * file, ValueType_ * ptr) {
//    file->write(reinterpret_cast<char*>(ptr),sizeof(*ptr));
    (*file) << (*ptr) << " ";
  }

  //! Header consists of name and size of the data
  static void writeHeader(std::ofstream * file, std::string name) {
    (*file) << name << " " << sizeof(ValueType_) << std::endl;
  }

};

//! Trait for TimestampPair type
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_same<signal_logger::TimestampPair, ValueType_>::value>::type>
{
  //! Writes both entries s and ns in binary form
  static void writeToFile(std::ofstream * file, ValueType_ * ptr) {
    //    file->write(reinterpret_cast<char*>(&ptr->first),sizeof(ptr->first));
    //    file->write(reinterpret_cast<char*>(&ptr->second),sizeof(ptr->second));
    (*file) << ptr->first << ptr->second << " ";
  }

  //! Header appends _s and _ns to the name of the timestamp
  static void writeHeader(std::ofstream * file, std::string name) {
    (*file) << name << "_s " << sizeof(typename ValueType_::first_type) << std::endl;
    (*file) << name << "_ns " << sizeof(typename ValueType_::second_type) << std::endl;
  }
};

//! Trait for Eigen Matrices length 3
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type>
{
  //! Write all entries as underlying type
  static void writeToFile(std::ofstream * file, ValueType_ * ptr) {
    for (int r=0; r<ptr->rows(); r++)  {
      for (int c=0; c<ptr->cols(); c++)  {
//        file->write(reinterpret_cast<char*>(&((*ptr)(r,c))),sizeof(((*ptr)(r,c))));
        (*file) << (*ptr)(r,c) << " ";
      }
    }
  }

  //! Header appends _r_c to the matrix name
  static void writeHeader(std::ofstream * file, std::string name, std::size_t no_rows, std::size_t no_cols) {
    for (int r=0; r<no_rows; r++)  {
      for (int c=0; c<no_cols; c++)  {
        (*file) << std::string(name + "_" + std::to_string(r) + "_" + std::to_string(c))
                << " " << sizeof(typename ValueType_::Scalar) << std::endl;
      }
    }
  }
};

//! Trait for Kindr vectors length 3
template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_base_of<kindr::VectorBase<ValueType_>, ValueType_>::value && ValueType_::Dimension == 3>::type>
{
  static void writeToFile(std::ofstream * file, ValueType_ * ptr) {
    //    file->write(reinterpret_cast<char*>(&ptr->x()),sizeof(ptr->x()));
    //    file->write(reinterpret_cast<char*>(&ptr->y()),sizeof(ptr->y()));
    //    file->write(reinterpret_cast<char*>(&ptr->z()),sizeof(ptr->z()));
    (*file) << ptr->x()<< " " << ptr->y() << " " << ptr->z() << " ";
  }

  //! Header append x, y and z to data name
  static void writeHeader(std::ofstream * file, std::string name) {
    (*file) << name << "_x " << sizeof(typename ValueType_::Scalar) << std::endl;
    (*file) << name << "_y " << sizeof(typename ValueType_::Scalar) << std::endl;
    (*file) << name << "_z " << sizeof(typename ValueType_::Scalar) << std::endl;
  }
};

}

}
