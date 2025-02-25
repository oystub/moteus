#pragma once

#include <vector>
#include <string>
#include <optional>
#include <algorithm>
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/persistent_config.h"

#include "dronecan_param.h"

#include <uavcan.protocol.param.GetSet.h>

/// DroneCAN Parameter Store to manage and iterate over registered parameters.
class DronecanParamStore {
public:

    struct BaseParamWrapper {
        virtual ~BaseParamWrapper() = default;
        virtual const std::string& GetName() const = 0;
        virtual uavcan_protocol_param_GetSetResponse Get() = 0;
        virtual bool Set(const uavcan_protocol_param_Value&) = 0;
        virtual void Reset() = 0;
        size_t index;
    };
        /// Constructor initializes the memory pool
    DronecanParamStore(mjlib::micro::Pool* pool) : pool_(pool) {}

    /// Registers a new parameter in the store and assigns an index.
    template <typename T>
    void Register(Parameter<T> param) {
        auto ptr = mjlib::micro::PoolPtr<ParamWrapper<T>>(pool_, param, next_index_++);
        params_.push_back(ptr.get());  // Store the base pointer for polymorphic access
    }

    /// Registers all parameters within a struct that calls DRONECAN_PARAMETER inside `RegisterParameters()`
    template <typename T>
    void Register(T& object) {
        object.RegisterParameters(*this);
    }

    /// Retrieves a parameter by index, returning std::optional
    std::optional<BaseParamWrapper*> GetByIndex(size_t index) {
        auto it = std::find_if(params_.begin(), params_.end(),
                               [index](const auto& p) { return p->index == index; });
        if (it != params_.end()) {
            return *it;
        }
        return std::nullopt;
    }

    /// Retrieves index by name, returns std::optional<size_t>
    std::optional<size_t> GetIndexByName(const std::string& name) {
        auto it = std::find_if(params_.begin(), params_.end(),
                               [&name](const auto& p) { return p->GetName() == name; });
        if (it != params_.end()) {
            return (*it)->index;
        }
        return std::nullopt;
    }

    /// Sets a parameter's value by index
    template <typename T>
    bool SetValueByIndex(size_t index, T new_value) {
        auto param_opt = GetByIndex(index);
        if (!param_opt) return false;
        
        return param_opt.value()->Set(new_value);
    }

    // Handle a GetSet request
    uavcan_protocol_param_GetSetResponse GetSet(const uavcan_protocol_param_GetSetRequest& req) {
        bool is_set_request = (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY);

        // Passed param name is not neccessarily null-terminated
        uint16_t name_len = std::min(static_cast<uint16_t>(req.name.len),
                                     static_cast<uint16_t>(sizeof(req.name.data)));
        std::string dronecan_name(reinterpret_cast<const char*>(req.name.data), name_len);

        std::optional<uint16_t> index;
        if (!dronecan_name.empty()) {
            index = GetIndexByName(dronecan_name);
        } else {
            index = req.index;
        }
        if (!index.has_value()) {
            // No parameter found with this name
            return uavcan_protocol_param_GetSetResponse();
        }

        // Look up the parameter by its index
        auto param_opt = GetByIndex(index.value());
        if (!param_opt.has_value()) {
            // No parameter found with this index
            return uavcan_protocol_param_GetSetResponse();
        }
        auto param = param_opt.value();

        if (is_set_request) {
            param->Set(req.value);
        }

        return param->Get();
    }

    /// Resets all parameters to their default values
    void ResetAll() {
        for (auto param : params_) {
            param->Reset();
        }
    }

    /// Sorts parameters alphabetically and reassigns consecutive indexes
    void Sort() {
        std::sort(params_.begin(), params_.end(),
                  [](const auto& a, const auto& b) { return a->GetName() < b->GetName(); });
        
        // Reassign indexes sequentially
        for (size_t i = 0; i < params_.size(); ++i) {
            params_[i]->index = i;
        }
    }

private:


    template <typename T>
    struct ParamWrapper : public BaseParamWrapper {
        explicit ParamWrapper(Parameter<T> param, size_t idx) 
            : param_(param) { index = idx; }

        const std::string& GetName() const override { return param_.GetName(); }

        uavcan_protocol_param_GetSetResponse Get() override {
            return param_.Get();
        }

        bool Set(const uavcan_protocol_param_Value& req) override {
            return param_.Set(req);
        }

        void Reset() override {
            param_.Reset();
        }

    private:
        Parameter<T> param_;
    };

    /// Using `PoolPtr` instead of `shared_ptr`
    mjlib::micro::Pool* pool_;
    std::vector<BaseParamWrapper*> params_;
    size_t next_index_ = 0;
};
