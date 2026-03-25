# Buffer Behavior

## Implementation

The signal logger buffer is built on top of a thread-safe
[Boost circular buffer](http://www.boost.org/doc/libs/1_54_0/libs/circular_buffer/doc/circular_buffer.html#boundedbuffer).

## Buffer Size

The configured buffer size defines the initial capacity.

## Buffer Types

- **Fixed size**: once full, no new elements are added.
- **Looping**: once full, the oldest element is overwritten.
- **Growing**: once full, the buffer grows exponentially.

Use the growing buffer type carefully: reallocating the buffer is expensive and
not suitable for time-critical code.
