#pragma once

#include <uavcan.protocol.param.GetSet.h>
#include "mjlib/micro/persistent_config.h"

#include <string>
#include <vector>
#include <optional>
#include <algorithm>
#include <cstring>
#include <functional>


struct DroneCanParamBase {
  DroneCanParamBase(const std::string name) : name(name) {}
  // Dronecan param name, should be max 16 characters to conform with PX4 standards
  const std::string name;
  virtual uavcan_protocol_param_GetSetResponse get() = 0;
  virtual bool set(const uavcan_protocol_param_GetSetRequest&){
    return false;
  }
  virtual void reset() = 0;
};

template <typename T>
class DroneCanParamMoteus : public DroneCanParamBase {
public:
  DroneCanParamMoteus() = delete;
  DroneCanParamMoteus(const DroneCanParamMoteus&) = delete;

  DroneCanParamMoteus(mjlib::micro::PersistentConfig** config, const std::string name, const std::string moteus_name, const T& default_value, const T& min_value, const T& max_value)
      : DroneCanParamBase(name), moteus_name_(moteus_name), default_value_(default_value), min_value_(min_value), max_value_(max_value), config_(config) {}

  DroneCanParamMoteus(mjlib::micro::PersistentConfig** config, const std::string name, const std::string moteus_name, const T& default_value)
      : DroneCanParamBase(name), moteus_name_(moteus_name), default_value_(default_value), config_(config) {}

  const std::string moteus_name_;
  std::optional<const T> default_value_;
  std::optional<const T> min_value_;
  std::optional<const T> max_value_;

  uavcan_protocol_param_GetSetResponse get() override {
    uavcan_protocol_param_GetSetResponse response{};
    memcpy(response.name.data, name.c_str(), std::min(name.size(), sizeof(response.name.data)));
    response.name.len = name.size();

    setValue(response.value, getMoteusParam());
    setValue(response.default_value, default_value_);
    setValue(response.min_value, min_value_);
    setValue(response.max_value, max_value_);
    return response;
  }

  bool set(const uavcan_protocol_param_GetSetRequest& req) override {
    if (config_ == nullptr || *config_ == nullptr) {
      return false;
    }

    auto val = getValue(req.value);
    if (!val.has_value()) {
      return false;
    }


    if (min_value_.has_value() && val.value() < min_value_.value()) {
      return false;
    }
    if (max_value_.has_value() && val.value() > max_value_.value()) {
      return false;
    }

    // We must serialize the value to a string to store it in the persistent config.
    return (*config_)->Set(moteus_name_, std::to_string(val.value()));
  }

  void reset() override {
    if (config_ == nullptr || *config_ == nullptr) {
      return;
    }
    if (default_value_.has_value()) {
      (*config_)->Set(moteus_name_, std::to_string(default_value_.value()));
    }
  }


private:
  mjlib::micro::PersistentConfig** config_;

  std::optional<T> getMoteusParam() {
    if (config_ == nullptr || *config_ == nullptr) {
      return std::nullopt;
    }
    // We get the parameter value from the persistent config as a string.
    std::string param_str = (*config_)->Get(moteus_name_);
    if (param_str.empty()) {
      return std::nullopt;
    }
    if constexpr(std::is_same_v<T, bool>){
      return param_str == "1";
    } else if (std::is_integral_v<T>) {
      return static_cast<T>(std::stoll(param_str));
    } else if (std::is_floating_point_v<T>) {
      return static_cast<T>(std::stof(param_str));
    } else {
      return std::nullopt;
    }
  }

  static std::optional<T> getValue(const uavcan_protocol_param_Value& field) {
    if constexpr (std::is_same_v<T, bool>){
      if (field.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE) {
        return static_cast<T>(field.boolean_value);
      } else if (field.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
        // PX4 uses 0 and 1 for bools
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

class DroneCanParamManager {
public:
  void registerParam(DroneCanParamBase* param) {
    params_.push_back(param);
  }

  uavcan_protocol_param_GetSetResponse getSet(const uavcan_protocol_param_GetSetRequest& req) {
    bool is_set_request = (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY);

    // Passed param name is not neccessarily null-terminated
    uint16_t name_len = std::min(static_cast<uint16_t>(req.name.len),
                                 static_cast<uint16_t>(sizeof(req.name.data)));
    std::string dronecan_name(reinterpret_cast<const char*>(req.name.data), name_len);

    std::optional<uint16_t> index;
    if (!dronecan_name.empty()) {
      index = getIndexByName(dronecan_name);
    } else {
      index = req.index;
    }
    if (!index.has_value()) {
      // No parameter found with this name
      return uavcan_protocol_param_GetSetResponse();
    }

    // Look up the parameter by its index
    auto param_opt = getParamByIndex(index.value());
    if (!param_opt.has_value()) {
      // No parameter found with this index
      return uavcan_protocol_param_GetSetResponse();
    }
    auto param = param_opt.value();

    if (is_set_request) {
      param->set(req);
    }

    return param->get();
  }

  void ResetAll() {
    for (auto param : params_) {
      param->reset();
    }
  }

private:
  std::optional<uint16_t> getIndexByName(const std::string& name) const {
    for (uint16_t i = 0; i < params_.size(); i++) {
      if (params_[i]->name == name) {
        return i;
      }
    }
    return std::nullopt;
  }

  std::optional<DroneCanParamBase*> getParamByIndex(uint16_t index) const {
    if (index < params_.size()) {
      return params_[index];
    }
    return std::nullopt;
  }

  std::vector<DroneCanParamBase*> params_;
  mjlib::micro::PersistentConfig* persistent_config_{nullptr};
};
