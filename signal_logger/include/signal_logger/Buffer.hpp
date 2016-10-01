/*
 * CircularBuffer.hpp
 *
 *  Created on: Sep 21, 2016
 *      Author: gabrielhottiger
 */

#pragma once

// Boost
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

// Signal logger
#include <signal_logger/BufferInterface.hpp>

namespace signal_logger {


// This implementation of a thread-safe circular buffer can be found here:
// http://www.boost.org/doc/libs/1_54_0/libs/circular_buffer/doc/circular_buffer.html#boundedbuffer

template <class T>
class Buffer : public internal::BufferInterface {
 public:

  typedef boost::circular_buffer<T> container_type;
  typedef typename container_type::size_type size_type;
  typedef typename container_type::value_type value_type;
  typedef typename boost::call_traits<value_type>::param_type param_type;

  explicit Buffer(size_type window_size) :
              no_unread_items_(0),
    		      container_(window_size)
  {

  }

  void push_front(param_type item) {
    // Lock the circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Update idx
    newest_idx_ = ++newest_idx_ % container_.capacity();
    no_unread_items_ = std::min(++no_unread_items_, container_.capacity());
    no_items_ = std::min(++no_items_, container_.capacity());

    // Add value to the buffer
    container_.push_front(item);

    // Lock
    lock.unlock();
    not_empty_.notify_one();
  }

  void pop_back(value_type * pItem) {
    // Lock the circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Wait for another thread to fill in a value to pop
    not_empty_.wait(lock, boost::bind(&Buffer<value_type>::is_not_empty, this));

    // Get item from buffer, decrement nr of unread items
    *pItem = container_[--newest_idx_];
    --no_unread_items_;
  }

  std::vector<value_type> read_full_buffer() {
    boost::mutex::scoped_lock lock(mutex_);

    std::vector<value_type> data_vector(no_items_);

    size_type idx = newest_idx_ - no_items_;
    size_type i = idx < 0 ? idx+container_.capacity() : idx;
    for(int j = 0; j < no_items_; ++j)
    {
      data_vector.at((i+j)%container_.capacity()) = container_[(i+j)%container_.capacity()];
    }
    return data_vector;
  }

  void set_capacity(std::size_t new_capacity) {
    // Lock the circular buffer
    boost::mutex::scoped_lock lock(mutex_);

    // Change capacity of the container
    container_.set_capacity(new_capacity);

    // Clear the buffer and restart filling
    container_.clear();
    no_unread_items_ = size_type(0);
    no_items_ = size_type(0);
    newest_idx_ = size_type(0);
  }

  //! Unread data in the buffer
  std::size_t get_no_unread_items() {
    boost::mutex::scoped_lock lock(mutex_);
    return no_unread_items_;
  }

  //! Get the number of "valid" items in the buffer
  std::size_t get_no_items() {
    boost::mutex::scoped_lock lock(mutex_);
    return no_items_;
  }


 private:
  Buffer(const Buffer&);              // Disabled copy constructor
  Buffer& operator = (const Buffer&); // Disabled assign operator

  // Helpers
  bool is_not_empty() const { return no_unread_items_ > 0; }

  // Count unread variables
  size_type newest_idx_;
  // Count unread variables
  size_type no_unread_items_;
  // Count all variables
  size_type no_items_;


  // Circular buffer
  container_type container_;
  // Mutex protecting accessing this container
  boost::mutex mutex_;
  // Condition for thread synchronisation
  boost::condition not_empty_;
};

} /* namespace signal_logger */
