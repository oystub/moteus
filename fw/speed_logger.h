#pragma once

#include <array>
#include <cstring>

#include "mjlib/micro/pool_array.h"

#include "fw/ccm.h"

class SpeedLogger {
public:
  struct Config {
    uint32_t size{1024};
    uint32_t decimation{1};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(size));
      a->Visit(MJ_NVP(decimation));
    }
  };

  struct Status {
    bool running{false};
    uint32_t allocated{0};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(running));
      a->Visit(MJ_NVP(allocated));
    }
  };

  SpeedLogger(mjlib::micro::Pool* pool) : pool_(pool) {}

  void start() {
    // The pool can only allocate once, so size is fixed first time record is started.
    if (data_ == nullptr) {
      // We also cap it at the available pool size.
      size_ = std::min(static_cast<uint32_t>(pool_->available()), config_.size);
      data_ = static_cast<uint8_t*>(pool_->Allocate(static_cast<size_t>(size_), alignof(uint8_t)));
      if (data_ == nullptr) {
        // Failed to allocate.
        size_ = 0;
      }
      status_.allocated = size_;
    }
    // Reset state.
    head_ = 0;
    count_ = 0;
    decimation_counter_ = 0; // For avoiding modulus in isrStep().
    status_.running = true; // Set last to avoid tearing in interrupt context.
  }

  void stop() {
    status_.running = false; // Only one flag, no risk of tearing.
  }
  
  // Drains up to max_n bytes from the buffer, fifo order, into dst.
  // Returns the number of bytes actually read.
  // Only returns data if the logger is stopped.
  uint32_t drain(uint8_t* dst, uint32_t max_n) {
    if (status_.running) return 0;
    return drainBytes(dst, max_n);
  }

  // Remaining bytes that can be drained (decreases as drain() is called).
  uint32_t drainAvailable() const {
    if (status_.running) return 0;
    return count_;
  }

  Config* config() { return &config_; }
  Status* status() { return &status_; }

  // Must be called from ISR context, returns true if it is time to log
  inline bool isrStep() {
    if (!status_.running) return false;
    if (++decimation_counter_ >= config_.decimation) {
      decimation_counter_ = 0;
      return true;
    }
    return false;
  }

  // Must be called from an ISR context, and only if isrStep() returned true.
  template <typename T>
  void isrLogValue(const T& value) {
    if (size_ == 0) return;
    writeBytes(reinterpret_cast<const uint8_t*>(&value), static_cast<uint32_t>(sizeof(T)));
  }

  // Must be called from an ISR context, and only if isrStep() returned true.
  void isrLogBytes(const void* src, uint32_t n) {
    if (size_ == 0) return;
    writeBytes(reinterpret_cast<const uint8_t*>(src), n);
  }

private:
  uint32_t drainBytes(uint8_t* dst, uint32_t max_n) {
    uint32_t n = (max_n > count_) ? count_ : max_n;
    if (n == 0) return 0;

    // Wrap around once if needed.
    uint32_t tail = (head_ >= count_) ? (head_ - count_) : (head_ + size_ - count_);

    const uint32_t to_end = size_ - tail;
    const uint32_t first = (n <= to_end) ? n : to_end;

    std::memcpy(dst, &data_[tail], first);
    if (n > first) {
      std::memcpy(dst + first, &data_[0], n - first);
    }

    count_ -= n;
    return n;
  }

  uint32_t writeBytes(const uint8_t* src, uint32_t n) {
    if (n == 0 || size_ == 0) return 0;
    if (n > size_) {
      // Edge case: If a single write exceeds ring size, keep the last "size_" bytes.
      src += (n - size_);
      n = size_;
    }

    // Drop old data that doesn't fit.
    uint32_t new_count = count_ + n;
    count_ = (new_count > size_) ? size_ : new_count;

    const uint32_t to_end = size_ - head_;
    const uint32_t first = (n <= to_end) ? n : to_end;
    std::memcpy(&data_[head_], src, first);
    if (n > first) {
      // Wrapped around, write the rest to the beginning.
      std::memcpy(&data_[0], src + first, n - first);
    }

    advanceIndex(head_, n);
    return n;
  }

  inline void advanceIndex(uint32_t& idx, uint32_t n) {
    // Conditional subtraction to avoid modulus.
    idx += n;
    if (idx >= size_) idx -= size_;
  }


  // State
  Status status_{};
  Config config_{};
  uint8_t* data_{nullptr};
  mjlib::micro::Pool* pool_{nullptr};

  uint32_t size_{0};
  uint32_t head_{0};
  uint32_t count_{0};
  uint32_t decimation_counter_{0};
};
