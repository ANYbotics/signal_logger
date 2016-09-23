/*
 * signal_logger_std_traits.hpp
 *
 *  Created on: Sep 23, 2016
 *      Author: gabrielhottiger
 */

#pragma once

namespace signal_logger_std {

namespace traits {

template <typename ValueType_, typename Enable_ = void>
struct sls_traits
{
  static void writeToFile(ValueType_ * ptr, std::ofstream * file) {
    (*file) << *ptr;
  }

};

template <typename ValueType_>
struct sls_traits<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type>
{
  static void writeToFile(ValueType_ * ptr, std::ofstream * file) {
    for (int r=0; r<ptr->rows(); r++)  {
      for (int c=0; c<ptr->cols(); c++)  {
        (*file) << (*ptr)(r,c);
      }
    }
  }
};

}
}
