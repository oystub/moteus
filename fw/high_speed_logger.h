#include "mjlib/micro/pool_array.h"
#include "fw/ccm.h"

#include <optional>

#pragma once

template <typename T>
class HighSpeedLogger {
public:
    HighSpeedLogger(mjlib::micro::Pool* pool, size_t size)
        : size_(size), data_(pool, size_) {
    }

    bool StartCapture() {
        if (is_capturing_) {
            return false; // Already active
        }
        is_capturing_ = true;
        data_ready_ = false;
        idx_ = 0;
        return true;
    }

    std::optional<T> GetData() {
        if (!data_ready_) {
            return std::nullopt;
        }
        if (idx_ >= size_) {
            return std::nullopt;
        }
        T result = data_[idx_];
        ++idx_;
        return result;
    }

    void Log(const T& data) MOTEUS_CCM_ATTRIBUTE {
        if (is_capturing_) {
            if (count_++ % skip != 0) {
                return;
            }
            data_[idx_] = data;
            ++idx_;
            if (idx_ >= size_) {
                is_capturing_ = false;
                data_ready_ = true;
                idx_ = 0;
            }
        }
    }

    size_t GetIndex() const {
        return idx_;
    }

    size_t size() const {
        return size_;
    }

private:
    size_t size_;
    size_t idx_{0};
    size_t count_{0};
    size_t skip{25};
    bool is_capturing_{false};
    bool data_ready_{false};
    mjlib::micro::PoolArray<T> data_;
};