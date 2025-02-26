#pragma once

#include <string>
#include <vector>
#include <optional>
#include <functional>
#include <limits>
#include <algorithm>

#include <uavcan.protocol.param.GetSet.h>

/// A templated class for DroneCAN parameters supporting integers, floats, and bools.
template <typename T>
class Parameter {
public:
    static_assert(std::is_integral_v<T> || std::is_floating_point_v<T> || std::is_same_v<bool, T>,
                  "Parameter type must be integral, floating point, or boolean.");

    using PreUpdateCallback = std::function<bool(Parameter<T>&, const T&)>;
    using PostUpdateCallback = std::function<void(const Parameter<T>&, const T&)>;

    Parameter(const std::string& name, T& value, T default_value = T{},
              T min_value = std::numeric_limits<T>::lowest(),
              T max_value = std::numeric_limits<T>::max(),
              PreUpdateCallback pre_update = nullptr,
              PostUpdateCallback post_update = nullptr)
        : name_(name), value_(value), default_value_(default_value),
          min_value_(min_value), max_value_(max_value),
          pre_update_(pre_update), post_update_(post_update) {}

    /// Attempts to set the parameter's value, enforcing constraints and calling callbacks.
    bool Set(T new_value) {
        if (new_value < min_value_ || new_value > max_value_) {
            return false;
        }

        T modified_value = new_value;
        if (pre_update_ && !pre_update_(*this, modified_value)) {
            return false; // Pre-update rejected the change
        }

        T old_value = value_;
        value_ = modified_value;

        if (post_update_) {
            post_update_(*this, old_value);
        }
        return true;
    }

    bool Set(const uavcan_protocol_param_Value& field) {
        auto val = getValue(field);
        if (!val.has_value()) {
            return false;
        }
        return Set(val.value());
    }

    uavcan_protocol_param_GetSetResponse Get() const {
        uavcan_protocol_param_GetSetResponse response{};
        response.name.len = std::min(name_.size(), sizeof(response.name.data));
        std::copy(name_.begin(), name_.begin() + response.name.len, response.name.data);
        setValue(response.value, value_);
        setValue(response.default_value, default_value_);
        setValue(response.max_value, max_value_);
        setValue(response.min_value, min_value_);
        return response;
    }

    void Reset() {
        value_ = default_value_;
    }

    const std::string& GetName() const { return name_; }
    const T& GetValue() const { return value_; }
    T GetDefault() const { return default_value_; }
    T GetMin() const { return min_value_; }
    T GetMax() const { return max_value_; }

private:
    std::string name_;
    T& value_; // Reference to actual parameter storage
    T default_value_;
    T min_value_;
    T max_value_;
    PreUpdateCallback pre_update_;
    PostUpdateCallback post_update_;

    static std::optional<T> getValue(const uavcan_protocol_param_Value& field) {
        if constexpr (std::is_same_v<T, bool>){
            if (field.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE) {
            return static_cast<T>(field.boolean_value);
            } else if (field.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
                // PX4 uses int 0 and 1 for bools
                if (field.integer_value == 0) {
                    return false;
                } else if (field.integer_value == 1) {
                    return true;
                }
            }
        } else if (std::is_integral_v<T>) {
            if (field.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
            return static_cast<T>(field.integer_value);
            }
        } else if (std::is_floating_point_v<T>) {
            if (field.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
            return static_cast<T>(field.real_value);
            }
        }
        return std::nullopt;
    }

    static void setValue(uavcan_protocol_param_Value& field, const std::optional<T>& val) {
        field.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY;
        if (!val.has_value()) {
            return;
        }

        if constexpr(std::is_same_v<T, bool>){
            field.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
            field.boolean_value = static_cast<uint8_t>(val.value());
        } else if (std::is_integral_v<T>) {
            field.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            field.integer_value = static_cast<int64_t>(val.value());
        } else if (std::is_floating_point_v<T>) {
            field.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            field.real_value = static_cast<float>(val.value());
        }
    }
    
    static void setValue(uavcan_protocol_param_NumericValue& field, const std::optional<T>& val) {
        field.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_EMPTY;
        if (!val.has_value()) {
            return;
        }
        if constexpr (std::is_integral_v<T>) {
            field.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
            field.integer_value = static_cast<int64_t>(val.value());
        } else if (std::is_floating_point_v<T>) {
            field.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
            field.real_value = static_cast<float>(val.value());
        }
    }
};

/// Macro to define a parameter inside a struct
#define DRONECAN_PARAMETER(name, variable, ...) \
    store.Register(Parameter<std::decay_t<decltype(variable)>>(#name, variable, ##__VA_ARGS__))
