#pragma once

#include <atomic>
#include <cstdint>

namespace pnx::f407::usb_detail
{

inline constexpr std::uint16_t cdc_full_speed_max_packet_size = 64U;
inline constexpr std::uint32_t connection_event_none = 0U;
inline constexpr std::uint32_t connection_event_disconnected = 1U;
inline constexpr std::uint32_t connection_event_connected = 2U;

constexpr std::uint32_t merge_connection_event(
    std::uint32_t pending, bool connected) noexcept
{
    if (connected)
    {
        return pending | connection_event_connected;
    }
    return (pending | connection_event_disconnected) &
           ~connection_event_connected;
}

enum class startup_phase : std::uint8_t
{
    idle,
    scheduled,
    controller_started,
    transport_ready,
    fault,
};

// Board-private asynchronous startup policy. It is intentionally limited to
// the state shared between the application thread, USB worker, and USBX CDC
// callbacks; public telemetry remains protected by the board mutex.
class startup_lifecycle
{
public:
    void reset() noexcept
    {
        phase_.store(startup_phase::idle, std::memory_order_release);
    }

    void schedule() noexcept
    {
        phase_.store(startup_phase::scheduled, std::memory_order_release);
    }

    [[nodiscard]] bool mark_controller_started() noexcept
    {
        startup_phase expected = startup_phase::scheduled;
        return phase_.compare_exchange_strong(
            expected, startup_phase::controller_started,
            std::memory_order_acq_rel, std::memory_order_acquire);
    }

    [[nodiscard]] bool mark_transport_ready() noexcept
    {
        startup_phase expected = startup_phase::controller_started;
        return phase_.compare_exchange_strong(
            expected, startup_phase::transport_ready,
            std::memory_order_acq_rel, std::memory_order_acquire);
    }

    void disconnect() noexcept
    {
        startup_phase expected =
            phase_.load(std::memory_order_acquire);
        while (expected == startup_phase::transport_ready &&
               !phase_.compare_exchange_weak(
                   expected, startup_phase::controller_started,
                   std::memory_order_acq_rel,
                   std::memory_order_acquire))
        {
        }
    }

    void fail() noexcept
    {
        phase_.store(startup_phase::fault, std::memory_order_release);
    }

    [[nodiscard]] bool scheduled() const noexcept
    {
        return phase_.load(std::memory_order_acquire) ==
               startup_phase::scheduled;
    }

    [[nodiscard]] bool controller_active() const noexcept
    {
        const startup_phase phase =
            phase_.load(std::memory_order_acquire);
        return phase == startup_phase::controller_started ||
               phase == startup_phase::transport_ready;
    }

    [[nodiscard]] bool transport_usable() const noexcept
    {
        return phase_.load(std::memory_order_acquire) ==
               startup_phase::transport_ready;
    }

    [[nodiscard]] bool faulted() const noexcept
    {
        return phase_.load(std::memory_order_acquire) ==
               startup_phase::fault;
    }

private:
    std::atomic<startup_phase> phase_{startup_phase::idle};
};

static_assert(
    std::atomic<startup_phase>::is_always_lock_free,
    "F407 USB startup lifecycle must be lock-free");

template <typename Config>
[[nodiscard]] bool same_config(
    const Config& lhs, const Config& rhs) noexcept
{
    return lhs.read_priority == rhs.read_priority &&
           lhs.write_priority == rhs.write_priority &&
           lhs.period_ticks == rhs.period_ticks &&
           lhs.min_rx_size == rhs.min_rx_size &&
           lhs.max_tx_size == rhs.max_tx_size &&
           lhs.on_rx == rhs.on_rx &&
           lhs.fill_tx == rhs.fill_tx &&
           lhs.on_tx_done == rhs.on_tx_done &&
           lhs.user == rhs.user;
}

enum class tx_next_action : std::uint8_t
{
    complete,
    send_zlp,
};

struct tx_completion_result
{
    tx_next_action next = tx_next_action::complete;
    std::uint16_t requested = 0U;
    std::uint16_t actual = 0U;
    bool success = false;
};

struct tx_ticket
{
    void* instance = nullptr;
    std::uint32_t generation = 0U;

    [[nodiscard]] bool valid() const noexcept
    {
        return instance != nullptr && generation != 0U;
    }
};

struct guarded_tx_completion
{
    bool accepted = false;
    tx_completion_result completion{};
};

class tx_completion_state
{
public:
    void start(std::uint16_t requested) noexcept
    {
        requested_ = requested;
        actual_ = 0U;
        phase_ = phase::data;
    }

    [[nodiscard]] bool busy() const noexcept
    {
        return phase_ != phase::idle;
    }

    tx_completion_result on_callback(
        bool success, std::uint16_t actual) noexcept
    {
        if (phase_ == phase::data)
        {
            actual_ = actual;
            const bool complete =
                success && actual_ == requested_;
            if (complete && requested_ != 0U &&
                requested_ % cdc_full_speed_max_packet_size == 0U)
            {
                phase_ = phase::zlp;
                return {
                    tx_next_action::send_zlp,
                    requested_,
                    actual_,
                    true};
            }
            return finish(complete);
        }
        return finish(success);
    }

    tx_completion_result abort() noexcept
    {
        return finish(false);
    }

    void reset() noexcept
    {
        requested_ = 0U;
        actual_ = 0U;
        phase_ = phase::idle;
    }

private:
    enum class phase : std::uint8_t
    {
        idle,
        data,
        zlp,
    };

    tx_completion_result finish(bool success) noexcept
    {
        const tx_completion_result result{
            tx_next_action::complete,
            requested_,
            actual_,
            success};
        reset();
        return result;
    }

    std::uint16_t requested_ = 0U;
    std::uint16_t actual_ = 0U;
    phase phase_ = phase::idle;
};

// Board-private lifecycle policy. Callers serialize access. USBX 6.1.10
// aborts endpoint transfers and suspends its bulk threads before invoking the
// application deactivate callback; after that quiescence point the generation
// prevents retired tickets/callbacks from mutating the current TX state even
// when USBX later reuses the same CDC class-instance pointer.
class tx_session_state
{
public:
    bool activate(void* instance) noexcept
    {
        if (instance == nullptr)
        {
            return false;
        }
        active_instance_ = instance;
        advance_generation();
        tx_generation_ = 0U;
        tx_.reset();
        return true;
    }

    bool deactivate(void* instance) noexcept
    {
        if (instance == nullptr || instance != active_instance_)
        {
            return false;
        }
        active_instance_ = nullptr;
        advance_generation();
        tx_generation_ = 0U;
        tx_.reset();
        return true;
    }

    [[nodiscard]] tx_ticket begin_tx() const noexcept
    {
        return active_instance_ != nullptr && !tx_.busy()
                   ? tx_ticket{active_instance_, generation_}
                   : tx_ticket{};
    }

    bool start(tx_ticket ticket, std::uint16_t requested) noexcept
    {
        if (!current(ticket) || tx_.busy())
        {
            return false;
        }
        tx_generation_ = ticket.generation;
        tx_.start(requested);
        return true;
    }

    guarded_tx_completion on_callback(
        void* instance, bool success, std::uint16_t actual) noexcept
    {
        if (instance == nullptr || instance != active_instance_ ||
            tx_generation_ != generation_ || !tx_.busy())
        {
            return {};
        }
        return {true, tx_.on_callback(success, actual)};
    }

    guarded_tx_completion abort(tx_ticket ticket) noexcept
    {
        if (!current(ticket) || tx_generation_ != generation_ ||
            !tx_.busy())
        {
            return {};
        }
        return {true, tx_.abort()};
    }

    guarded_tx_completion abort(void* instance) noexcept
    {
        if (instance == nullptr || instance != active_instance_ ||
            tx_generation_ != generation_ || !tx_.busy())
        {
            return {};
        }
        return {true, tx_.abort()};
    }

    [[nodiscard]] bool current(tx_ticket ticket) const noexcept
    {
        return ticket.valid() &&
               ticket.instance == active_instance_ &&
               ticket.generation == generation_;
    }

    [[nodiscard]] bool busy() const noexcept
    {
        return tx_.busy();
    }

private:
    void advance_generation() noexcept
    {
        ++generation_;
        if (generation_ == 0U)
        {
            generation_ = 1U;
        }
    }

    void* active_instance_ = nullptr;
    std::uint32_t generation_ = 0U;
    std::uint32_t tx_generation_ = 0U;
    tx_completion_state tx_{};
};

} // namespace pnx::f407::usb_detail
