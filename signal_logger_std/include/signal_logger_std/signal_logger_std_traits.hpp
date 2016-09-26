/*
 * signal_logger_std_traits.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gabrielhottiger
 */

#pragma once

#include <fstream>

namespace signal_logger_std {

namespace traits {

template <typename ValueType_, typename Enable_ = void>
struct sls_traits
{
  static void writeToFile(std::ofstream * file, ValueType_ * ptr) {
    file->write(reinterpret_cast<char*>(ptr),sizeof(*ptr));
  }

  static void writeHeader(std::ofstream * file, std::string name) {
    (*file) << name << " " << sizeof(ValueType_) << std::endl;

  }

};

template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type>
{
  static void writeToFile(std::ofstream * file, ValueType_ * ptr) {
    for (int r=0; r<ptr->rows(); r++)  {
      for (int c=0; c<ptr->cols(); c++)  {
        file->write(reinterpret_cast<char*>(&((*ptr)(r,c))),sizeof(((*ptr)(r,c))));
      }
    }
  }

  static void writeHeader(std::ofstream * file, std::string name, std::size_t no_rows, std::size_t no_cols) {
    for (int r=0; r<no_rows; r++)  {
      for (int c=0; c<no_cols; c++)  {
        (*file) << std::string(name + "_" + std::to_string(r) + "_" + std::to_string(c))
                << " " << sizeof(typename ValueType_::Scalar) << std::endl;
      }
    }
  }
};

}
}
