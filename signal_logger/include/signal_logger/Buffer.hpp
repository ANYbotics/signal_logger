/*
 * CircularBuffer.hpp
 *
 *  Created on: Sep 21, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// Signal logger
#include "signal_logger/LogElementTypes.hpp"

// Message logger
#include "message_logger/message_logger.hpp"

// Eigen
#include <Eigen/Core>

// Boost
#include <boost/circular_buffer.hpp>
#include <boost/thread/mutex.hpp>

// STL
#include <type_traits>

namespace signal_logger {

/** Buffer that stores all elements in a circular manner. Allows looping, thus overwriting
 *  old entries with newer ones, keeps count of the unread items, push and pop influence this count.
 *  If buffer is empty no elements can be popped, if buffer is full and not looping
 *  no elements can be pushed. Popping does not delete the item, only sets unread item
 *  count . A copy of all the elements stored in the  buffer (read and unread items) can be obtained.
 *  This implementation of a thread-safe circular buffer is adapted from here:
 *  http://www.boost.org/doc/libs/1_54_0/libs/circular_buffer/doc/circular_buffer.html#boundedbuffer
 */
//! Thread-safe circular buffer
template <typename ValueType_>
class Buffer
{
 public:
  //! Convenience typedefs
  template<typename T , typename Enable = void>
  struct Container {
    typedef boost::circular_buffer<ValueType_> type;
  };

  template<typename T>
  struct Container<T, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<T>, T>::value>::type>
  {
       typedef boost::circular_buffer<typename ValueType_::Scalar> type;
  };

  typedef typename Container<ValueType_>::type container_type;
  typedef typename container_type::size_type size_type;

  /** Constructor for non-eigen-types
   *  @param window_size
   */
  template<typename V = ValueType_>
  explicit Buffer(V * ptr, typename std::enable_if<!std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type* = 0) :
    ptr_(ptr),
    noUnreadItems_(0),
    noItems_(0),
    isLooping_(true),
    container_(0),
    mutex_(),
    rows_(1),
    cols_(1)
  {

  }

  /** Constructor for eigen-types
   *  @param window_size
   */
  template<typename V = ValueType_>
  explicit Buffer(V * ptr, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type* = 0) :
    ptr_(ptr),
    noUnreadItems_(0),
    noItems_(0),
    isLooping_(true),
    container_(0),
    mutex_(),
    rows_(ptr->rows()),
    cols_(ptr->cols())
  {

  }

  //! Push data into the buffer (if looping or not full).
  bool collect()
  {
    // Lock the circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Non-looping buffers don't allow filling a full buffer
    if( (!isLooping_ && noItems_ >= bufferSize_) || bufferSize_ == 0) { return false; }

    // Add value to the buffer
    pushElementFront(ptr_);

    /*  lastIdx -> iterating through buffer
     *  noUnreadItems -> increases on push decreased on pop
     *  noItems -> increases on push, max out at capacity
     */
    noUnreadItems_ = std::min(++noUnreadItems_, bufferSize_);
    noItems_ = std::min(++noItems_, bufferSize_);

    return true;
  }

  /** Pop an item from the buffer (if buffer not empty).
   *  @param pItem pointer to value in which popped value is stored
   */
  bool read(ValueType_ * pItem)
  {
    // Lock the circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Check if buffer is empty
    if( noUnreadItems_ == 0 ) { return false; }

    // Get item from buffer, Decrement nr of unread items
    readElementAtPosition(pItem, --noUnreadItems_);

    return true;
  }

  ValueType_ copyElementFromBack(std::size_t idx)  const {
    // Lock the circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // read element
    ValueType_ time;
    readElementAtPosition(&time, (noItems_-1)-idx);
    return time;
  }

  /** Make a copy of the complete (valid) buffer entries. The unread counter remains
   *  unchanged, it copies the last noItmes_ items into a vector of value_type.
   *  @return vector containing all buffered items
   */
  template<typename V = ValueType_>
  std::vector<V> copyBuffer(typename std::enable_if<!std::is_same<bool, V>::value>::type* = 0) const
  {
    // Lock circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Fill vector
    std::vector<ValueType_> data_vector(noItems_);
    for(int j = 0; j < noItems_; ++j) {
      readElementAtPosition( &data_vector[j] , (noItems_ - 1) - j);
    }

    return data_vector;
  }

  template<typename V = ValueType_>
  std::vector<bool> copyBuffer(typename std::enable_if<std::is_same<bool, V>::value>::type* = 0) const
  {
    // Lock circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Fill vector
    std::vector<bool> data_vector(noItems_);
    for(int j = 0; j < noItems_; ++j) {
      bool read;
      readElementAtPosition( &read , (noItems_ - 1) - j);
      data_vector[j] = read;
    }

    return data_vector;
  }

  /** Change the buffer size
   *  @param bufferSize new buffer size
   */
  void setBufferSize(const std::size_t bufferSize) {
    // Lock the circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Change capacity of the container
    bufferSize_ = bufferSize;
    container_.set_capacity(bufferSize_ * rows_ * cols_);

    // Clear the buffer and restart filling
    container_.clear();
    noUnreadItems_ = size_type(0);
    noItems_ = size_type(0);
  }

  //! @return number of unread elements
  std::size_t noUnreadItems() const {
    boost::mutex::scoped_lock lock(mutex_);
    return noUnreadItems_;
  }

  //! @return number of items stored in the buffer (read and unread)
  std::size_t noItems() const {
    boost::mutex::scoped_lock lock(mutex_);
    return noItems_;
  }

  //! @return true, iff buffer is looping
  bool isLooping() const {
    boost::mutex::scoped_lock lock(mutex_);
    return isLooping_;
  }

  //! @param isLooping flag indicating if buffer is looping
  void setIsLooping(const bool isLooping) {
    boost::mutex::scoped_lock lock(mutex_);
    isLooping_ = isLooping;
  }

  //! Clear the buffer
  void clear() {
    boost::mutex::scoped_lock lock(mutex_);
    // Clear the buffer and restart filling
    container_.clear();
    noUnreadItems_ = size_type(0);
    noItems_ = size_type(0);
  }

 private:
  //! Disabled copy constructor
  Buffer(const Buffer&);
  //! Disabled assign operator
  Buffer& operator = (const Buffer&);

 private:
  // Version for non-eigen-types
  template<typename V = ValueType_>
  typename std::enable_if<!std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  pushElementFront(ValueType_ * item) {
    container_.push_front(*item);
  }

  // Version for eigen-types
  template<typename V = ValueType_>
  typename std::enable_if<std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  pushElementFront(ValueType_ * item) {
    if(item->rows() != rows_ || item->cols() != cols_) {
      // Error output -> don't push back
      MELO_ERROR_STREAM("Matrix size not consistent" << std::endl << "init_rows = " << rows_ <<
                        "item_rows = " << item->rows() << std::endl << "init_cols = " << cols_ <<
                        "item_cols = " << item->cols());
      return;
    }
    for(std::size_t i = 0; i < item->size(); ++i) {
      container_.push_front( *(item->data() + i) );
    }
  }

  // Version for non-eigen-types
  template<typename V = ValueType_>
  typename std::enable_if<!std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  readElementAtPosition(ValueType_ * item, size_t position)  const {
    if(position < 0 || position >= noItems_) {
      throw std::out_of_range("Can not read element at position " + std::to_string(position));
    }
    *item = container_[position];
  }

  // Version for eigen-types
  template<typename V = ValueType_>
  typename std::enable_if<std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  readElementAtPosition(ValueType_ * item, size_t position)  const {
    if(position < 0 || position >= noItems_) {
      throw std::out_of_range("Can not read element at position " + std::to_string(position));
    }
    item->resize(rows_, cols_);
    // (position+1)*cols_*rows_ -1  refers to the last element of the buffered matrix
    for(std::size_t i = 0; i < item->size(); ++i) {
      *(item->data() + i) = container_[ (position+1)*cols_*rows_ - 1 - i ];
    }
  }

 private:
  //! Pointer to value
  ValueType_* ptr_;
  //! Buffer size w.r.t. to ValueType_ (bufferSize_ is not necessary equal to container_.capacity())
  size_type bufferSize_;
  //! Number of unread items
  size_type noUnreadItems_;
  //! Number of items in the buffer (read and unread)
  size_type noItems_;
  //! Is the buffer "circulating", refreshing old entries with new ones
  bool isLooping_;
  //! Circular buffer
  container_type container_;
  //! Mutex protecting accessing this container
  mutable boost::mutex mutex_;

  //! Eigen specific entries (0 in other cases)
  const std::size_t rows_;
  const std::size_t cols_;
};

} /* namespace signal_logger */
