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
#include <algorithm>

namespace signal_logger {

//! Enum containing possible buffer types
enum class BufferType: unsigned int
{
  FIXED_SIZE = 0,
  LOOPING = 1,
  EXPONENTIALLY_GROWING = 2
};

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
  //! Container typedef (eigen container consists of underlying type)
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

 public:
  /** Constructor for non-eigen-types
   *  @param ptr  pointer to the data to be buffered
   */
  template<typename V = ValueType_>
  explicit Buffer(const V * const ptr, typename std::enable_if<!std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type* = 0) :
  ptr_(ptr),
  bufferSize_(0),
  bufferType_(BufferType::LOOPING),
  noUnreadItems_(0),
  noItems_(0),
  container_(bufferSize_),
  mutex_(),
  rows_(1),
  cols_(1)
  {

  }

  /** Constructor for eigen-types (initializes rows_ and cols_ correctly)
   *  @param ptr  pointer to the data to be buffered
   */
  template<typename V = ValueType_>
  explicit Buffer(const V * const ptr, typename std::enable_if<std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type* = 0) :
  ptr_(ptr),
  bufferSize_(0),
  bufferType_(BufferType::LOOPING),
  noUnreadItems_(0),
  noItems_(0),
  container_(bufferSize_),
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

    // If buffer is exponentially increasing and full -> increase size by factor 2
    if( bufferType_ == BufferType::EXPONENTIALLY_GROWING && noItems_ >= bufferSize_)
    {
      bufferSize_ = std::max(size_type(100), 2*bufferSize_);
      container_.set_capacity(bufferSize_* rows_ * cols_);
    }
    else {
      // Non-looping buffers don't allow filling a full buffer
      if( (bufferType_ != BufferType::LOOPING && noItems_ >= bufferSize_) || bufferSize_ == 0)
      {
        return false;
      }
    }

    // Add value to the buffer
    pushElementFront(ptr_);

    /*  noUnreadItems -> increases on push decreased on pop, max out at capacity
     *  noItems -> increases on push, max out at capacity
     */
    noUnreadItems_ = std::min(++noUnreadItems_, bufferSize_);
    noItems_ = std::min(++noItems_, bufferSize_);

    return true;
  }

  /** Pop an item from the buffer (if buffer not empty). Decrement no unread items.
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

  /** Pop an item from back. Not decrementing unread items.
   *  @param idx pointer to value in which popped value is stored
   *  @return val Copy of value at idx
   */
  ValueType_ readElementAtPosition(std::size_t idx)  const {
    // Lock the circular buffer
    boost::mutex::scoped_lock lock(mutex_);
    // read element
    ValueType_ val;
    readElementAtPosition(&val, idx);
    return val;
  }

  /** Make a copy of the complete (valid) buffer entries. The unread counter remains
   *  unchanged, it copies the last noItmes_ items into a vector of value_type.
   *  @return vector containing all buffered items
   */
  std::vector<ValueType_> copyBuffer() const
  {
    // Lock circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Fill vector
    std::vector<ValueType_> data_vector(noItems_);
    for(int j = 0; j < noItems_; ++j) {
      // In this way no specialization for vector bool is necessary
      ValueType_ read;
      readElementAtPosition( &read , (noItems_ - 1) - j);
      data_vector[j] = read;
    }
    return data_vector;
  }

  //! Allocate buffer size of memory
  void allocate() {
    boost::mutex::scoped_lock lock(mutex_);
    container_.set_capacity(bufferSize_ * rows_ * cols_);
  }

  //! Deallocate memory
  void deallocate() {
    boost::mutex::scoped_lock lock(mutex_);
    container_.set_capacity(0);
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

  //! @return size of the buffer
  std::size_t getBufferSize() const {
    boost::mutex::scoped_lock lock(mutex_);
    return bufferSize_;
  }

  //! @param bufferSize new buffer size
  void setBufferSize(const std::size_t bufferSize) {
    boost::mutex::scoped_lock lock(mutex_);
    bufferSize_ = bufferSize;
    container_.set_capacity(bufferSize_ * rows_ * cols_);
  }

  //! @return type of the buffer
  BufferType getType() const {
    boost::mutex::scoped_lock lock(mutex_);
    return bufferType_;
  }

  //! @param bufferType type of the buffer
  void setType(const BufferType bufferType) {
    boost::mutex::scoped_lock lock(mutex_);
    bufferType_ = bufferType;
  }

  //! Clear the buffer
  void clear() {
    boost::mutex::scoped_lock lock(mutex_);
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
  /** Push an element at front for non-eigen types
   *  @param item item to push at front
   *  @return template specialization by return type
   */
  template<typename V = ValueType_>
  typename std::enable_if<!std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  pushElementFront(const ValueType_ * const item) {
    container_.push_front(*item);
  }

  /** Push an element at front for eigen types
   *  @param item item to push at front of underlying-type buffer
   *  @return template specialization by return type
   */
  template<typename V = ValueType_>
  typename std::enable_if<std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  pushElementFront(const ValueType_ * const item) {
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

  /** Push an element at front for non-eigen types
   *  @param item item to read from buffer
   *  @param idx position in the buffer
   *  @return template specialization by return type
   */
  template<typename V = ValueType_>
  typename std::enable_if<!std::is_base_of<Eigen::MatrixBase<V>, V>::value>::type
  readElementAtPosition(ValueType_ * item, size_t position)  const {
    if(position < 0 || position >= noItems_) {
      throw std::out_of_range("Can not read element at position " + std::to_string(position));
    }
    *item = container_[position];
  }

  /** Push an element at front for eigen types
   *  @param item item to read from buffer
   *  @param idx position in the buffer
   *  @return template specialization by return type
   */
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
  const ValueType_* const ptr_;
  //! Buffer size w.r.t. to ValueType_ (bufferSize_ is not necessary equal to container_.capacity())
  size_type bufferSize_;
  //! Is the buffer "circulating", refreshing old entries with new ones
  BufferType bufferType_;
  //! Number of unread items
  size_type noUnreadItems_;
  //! Number of items in the buffer (read and unread)
  size_type noItems_;
  //! Circular buffer
  container_type container_;
  //! Mutex protecting accessing this container
  mutable boost::mutex mutex_;
  //! Eigen specific entries (1 in other cases)
  const std::size_t rows_;
  const std::size_t cols_;
};

} /* namespace signal_logger */
