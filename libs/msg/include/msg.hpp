#pragma once

#include <cstring>
#include <type_traits>

#include "tx_api.h"
#include "stdint.h"

#include "status.hpp"

namespace msg
{
    
inline constexpr uint8_t max_subscribers = 8U;
inline constexpr uint8_t invalid_index = UINT8_MAX;

template <typename T>
class subscriber;

template <typename T>
class topic final 
{
    static_assert(std::is_trivially_copyable_v<T>);

    // Allow subscriber to access private members of topic
    friend class subscriber<T>;

    struct slot
    {
        bool active = false;
        bool pending = false;
    };

public:

    topic() noexcept = default;
    topic(const topic&) = delete;
    topic& operator=(const topic&) = delete;

    // Init a topic with a given struct.
    pnxtypes::status init() noexcept
    {
        if (initialized_)
            return pnxtypes::status::ok;
        
        if (tx_mutex_create(&mutex_, const_cast<CHAR*>("msg"), TX_INHERIT) != TX_SUCCESS)
            return pnxtypes::status::error;

        initialized_ = true;
        return pnxtypes::status::ok;
    }

    // Deinit a topic.
    pnxtypes::status deinit() noexcept
    {
        if (!initialized_)
            return pnxtypes::status::invalid_arg;

        if (tx_mutex_delete(&mutex_) != TX_SUCCESS)
            return pnxtypes::status::error;

        initialized_ = false;
        subcount_ = 0;

        for (auto& sub : subscribers_)
            sub = {};

        return pnxtypes::status::ok;
    }

    // Subscribe to a topic. Returns the index of the subscriber.
    subscriber<T> subscribe() noexcept
    {
        if (!initialized_)
            return {};

        if (tx_mutex_get(&mutex_, TX_NO_WAIT) != TX_SUCCESS)
            return {};

        uint8_t index = invalid_index;

        if (subcount_ < max_subscribers)
        {
            index = subcount_++;
            subscribers_[index] = {true, false};
        }

        tx_mutex_put(&mutex_);

        if (index == invalid_index)
            return {};

        return subscriber<T>{this, index};
    }

    // Publish a message to all subscribers.
    pnxtypes::status publish(const T& data) noexcept
    {
        if (!initialized_)
            return pnxtypes::status::invalid_arg;

        if (tx_mutex_get(&mutex_, TX_NO_WAIT) != TX_SUCCESS)
            return pnxtypes::status::busy;

        data_ = data;

        for (uint8_t i = 0; i < subcount_; ++i)
            subscribers_[i].pending = true;

        tx_mutex_put(&mutex_);

        return pnxtypes::status::ok;
    }

private:
    
    // Check if the subscriber has a pending message
    [[nodiscard]] bool available(uint8_t index) noexcept
    {
        if (!initialized_ || index >= subcount_)
            return false;

        if (tx_mutex_get(&mutex_, TX_NO_WAIT) != TX_SUCCESS)
            return false;

        const bool pending = subscribers_[index].active && subscribers_[index].pending;

        tx_mutex_put(&mutex_);

        return pending;
    }

    // Read the message for the subscriber
    pnxtypes::status read(uint8_t index, T& output) noexcept
    {
        if (!initialized_ || index >= subcount_)
            return pnxtypes::status::invalid_arg;

        if (tx_mutex_get(&mutex_, TX_NO_WAIT) != TX_SUCCESS)
            return pnxtypes::status::busy;

        auto& sub = subscribers_[index];

        if (!sub.active || !sub.pending)
        {
            tx_mutex_put(&mutex_);
            return pnxtypes::status::busy;
        }

        std::memcpy(&output, &data_, sizeof(T));
        sub.pending = false;

        tx_mutex_put(&mutex_);

        return pnxtypes::status::ok;
    }

    TX_MUTEX mutex_{};
    T data_{};

    slot subscribers_[max_subscribers]{};
    uint8_t subcount_ = 0;

    bool initialized_ = false;
};


template <typename T>
class subscriber
{
    friend class topic<T>;

public:
    subscriber() = default;

    [[nodiscard]] bool valid() const noexcept
    {
        return owner_ != nullptr && index_ != invalid_index;
    }

    [[nodiscard]] explicit operator bool() const noexcept
    {
        return valid();
    }

    [[nodiscard]] bool available() const noexcept
    {
        if (!valid())
            return false;

        return owner_->available(index_);
    }

    pnxtypes::status read(T& output) noexcept
    {
        if (!valid())
            return pnxtypes::status::invalid_arg;

        return owner_->read(index_, output);
    }

private:
    subscriber(topic<T>* owner, uint8_t index) noexcept
        : owner_(owner), index_(index)
    {
    }

    topic<T>* owner_ = nullptr;
    uint8_t index_ = invalid_index;
};

} // namespace msg
