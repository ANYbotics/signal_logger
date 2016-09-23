/*
 * BufferInterface.hpp
 *
 *  Created on: Sep 22, 2016
 *      Author: gabrielhottiger
 */

#pragma once

// Signal logger
#include "signal_logger/LogElementInterface.hpp"

// Eigen
#include "Eigen/Core"

namespace signal_logger {

template <typename ValueType_, typename Enable_ = void>
class LogElementBase: public LogElementInterface
{
 public:
  LogElementBase(ValueType_ * ptr, std::string name, std::size_t buffer_size) :
    LogElementInterface(typeid(ValueType_), name),
    ptr_(ptr)
  {
    pBuffer_.reset(new Buffer<ValueType_>(buffer_size));
  }

  virtual ~LogElementBase() {

  }

  void collect() {
    push_front<ValueType_>(*ptr_);
  }

  void readBuffer(ValueType_ * ptr) {
    pop_back<ValueType_>(ptr);
  }

  void publish() {
    ValueType_ * ptr = new ValueType_();
    pop_back<ValueType_>(ptr);
    std::cout << "Getting data: " << *ptr << " from buffer. Could be processed now." << std ::endl;
  }

 protected:
  ValueType_ * ptr_;

};

template <typename ValueType_>
class LogElementBase<ValueType_, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<ValueType_>, ValueType_>::value>::type> : public LogElementInterface
{
 public:
  LogElementBase(ValueType_ * ptr, std::string name, std::size_t buffer_size) :
    LogElementInterface(typeid(typename ValueType_::Scalar), name),
    ptr_(ptr),
    no_rows_(ptr->rows()),
    no_cols_(ptr->cols())
  {
    pBuffer_.reset(new Buffer<typename ValueType_::Scalar>(buffer_size*no_rows_*no_cols_));

    std::cout<<"Log Element for Eigen Matrix with cols: "<<ptr->cols()<<" rows: "<<ptr->rows()<<std::endl;

  }

  virtual ~LogElementBase() {

  }

  void collect() {
    for (int r=0; r<no_rows_; r++)  {
      for (int c=0; c<no_cols_; c++)  {
        push_front<typename ValueType_::Scalar>( (*ptr_)(r,c) );
      }
    }
  }

  void readBuffer(ValueType_ * ptr)
  {
    ptr->resize(no_rows_, no_cols_);

    for (int r=0; r<no_rows_; r++)  {
      for (int c=0; c<no_cols_; c++)  {
        pop_back<typename ValueType_::Scalar>(& ((*ptr)(r,c)) );
      }
    }
  }

  void publish() {
    ValueType_ * ptr = new ValueType_(no_rows_, no_cols_);

    for (int r=0; r<no_rows_; r++)  {
      for (int c=0; c<no_cols_; c++)  {
        pop_back<typename ValueType_::Scalar>(& ((*ptr)(r,c)) );
      }
    }
    std::cout << "Getting matrix: " << std::endl;
    for (int r=0; r<no_rows_; r++)  {
      for (int c=0; c<no_cols_; c++)  {
        std::cout<< (*ptr)(r,c) << "\t";
      }
      std::cout<<std::endl;
    }
    std::cout << " from buffer. Could be processed now." << std ::endl;
  }

 protected:
  ValueType_ * ptr_;
  std::size_t no_rows_;
  std::size_t no_cols_;

};

} /* namespace signal_logger */

