#pragma once

#include <array>
#include <cstring>

#include "mjlib/micro/pool_array.h"

#include "fw/ccm.h"

class SpeedLogger {
public:
  struct Config {
    uint16_t size{1024};
    uint16_t decimation{1};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(size));
      a->Visit(MJ_NVP(decimation));
    }
  };

  struct Status {
    bool running{false};
    uint16_t allocated{0};
    uint32_t cycles{0};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(running));
      a->Visit(MJ_NVP(allocated));
      a->Visit(MJ_NVP(cycles));
    }
  };

  SpeedLogger(mjlib::micro::Pool* pool) : pool_(pool) {}

  void start() {
    if (data_ == nullptr) {
      size_ = std::min(pool_->available(), static_cast<size_t>(config_.size));
      data_ = static_cast<uint8_t*>(pool_->Allocate(size_, alignof(uint8_t)));
      status_.allocated = size_;
    }
    __disable_irq();
      head_ = 0;
      tail_ = 0;
      count_ = 0;
      read_pos_ = 0;
      read_remaining_ = 0;
      status_.running = true;
    __enable_irq();
  }

  void stop() {
    __disable_irq();
      status_.running = false;
      if (count_ == 0) {
        read_pos_ = 0;
        read_remaining_ = 0;
      } else {
        // newest byte is just before head_
        read_pos_ = (head_ == 0) ? static_cast<uint16_t>(size_ - 1) : static_cast<uint16_t>(head_ - 1);
        read_remaining_ = count_;
      }
    __enable_irq();
  }

  // Should be called from an ISR context
  void poll() {
    if (!status_.running) return;
    ++status_.cycles;
  }

  // Should be called from an ISR context
  bool shouldLog() const {
    if (!status_.running) return false;
    return (status_.cycles % config_.decimation) == 0;
  }

  // Should be called from an ISR context
  template <typename T>
  void logValue(const T& value) {
    if (!status_.running || data_ == nullptr || size_ == 0) return;
    writeBytes(reinterpret_cast<const uint8_t*>(&value), static_cast<uint16_t>(sizeof(T)));
  }

  uint16_t readBytes(uint8_t* dst, uint16_t max_n) {
    if (status_.running || data_ == nullptr || max_n == 0) return 0;
    uint16_t n = (max_n > read_remaining_) ? read_remaining_ : max_n;
    if (n == 0) return 0;

    for (uint16_t i = 0; i < n; ++i) {
      dst[i] = data_[read_pos_];
      retreatIndex(read_pos_, 1);
    }
    read_remaining_ = static_cast<uint16_t>(read_remaining_ - n);
    return n;
  }

  uint16_t bytesPending() const { return read_remaining_; }

  Config* config() { return &config_; }
  Status* status() { return &status_; }

private:
    void writeBytes(const uint8_t* src, uint16_t n) MOTEUS_CCM_ATTRIBUTE {
    if (n == 0 || size_ == 0) return;
    if (n > size_) {
      // If a single record exceeds ring size, keep the last 'size_' bytes.
      src += (n - size_);
      n = size_;
    }

    // If not enough free space, advance tail (drop oldest).
    const uint16_t free_space = static_cast<uint16_t>(size_ - count_);
    if (n > free_space) {
      const uint16_t drop = static_cast<uint16_t>(n - free_space);
      advanceIndex(tail_, drop);
      count_ = std::min<uint16_t>(size_, static_cast<uint16_t>(count_ + n));
    } else {
      count_ = static_cast<uint16_t>(count_ + n);
    }

    // Write potentially in two segments (to end, then wrap)
    const uint16_t to_end = static_cast<uint16_t>(size_ - head_);
    const uint16_t first = (n <= to_end) ? n : to_end;
    std::memcpy(&data_[head_], src, first);
    if (n > first) {
      std::memcpy(&data_[0], src + first, static_cast<size_t>(n - first));
    }

    advanceIndex(head_, n);
  }

  inline void advanceIndex(uint16_t& idx, uint16_t n) {
    idx = static_cast<uint16_t>((idx + n) % size_);
  }

  inline void retreatIndex(uint16_t& idx, uint16_t n) {
    // Todo: maybe assume power of two size_ and use bitmask to speed up?
    idx = static_cast<uint16_t>((idx + size_ - (n % size_)) % size_);
  }

  // State
  Status status_{};
  Config config_{};
  uint8_t* data_{nullptr};
  mjlib::micro::Pool* pool_{nullptr};

  uint16_t size_{0};
  uint16_t head_{0};
  uint16_t tail_{0};
  uint16_t count_{0};

  uint16_t read_pos_{0};
  uint16_t read_remaining_{0};
};