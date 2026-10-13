#pragma once 

#include <functional>
#include <type_traits>
#include <utility>

namespace pnxtypes
{

template <typename Signature>
class callback;

template <typename Result, typename... Args>
class callback<Result(Args...)>
{
public:
    using entry_type = Result (*)(void*, Args...);

    constexpr callback() noexcept = default;

    template <auto Function>
    static constexpr callback bind() noexcept
    {
        static_assert(std::is_invocable_r_v<Result, decltype(Function), Args...>);

        return {nullptr, [](void*, Args... args) -> Result
        {
            return std::invoke(Function, std::forward<Args>(args)...);
        }};
    }

    template <auto Method, typename Object>
    static constexpr callback bind(Object* object) noexcept
    {
        static_assert(std::is_member_function_pointer_v<decltype(Method)>);
        static_assert(std::is_invocable_r_v<Result, decltype(Method), Object&, Args...>);

        return object ? callback{const_cast<void*>(
            static_cast<const void*>(object)),
            [](void* context, Args... args) -> Result
            {
                return std::invoke(
                    Method,
                    *static_cast<Object*>(context),
                    std::forward<Args>(args)...
                );
            }} : callback{};
    }

    [[nodiscard]] constexpr bool valid() const noexcept
    {
        return entry_ != nullptr;
    }

    constexpr explicit operator bool() const noexcept
    {
        return valid();
    }

    Result operator()(Args... args) const
    {
        return entry_(context_, std::forward<Args>(args)...);
    }

    friend constexpr bool operator==(callback lhs, callback rhs) noexcept
    {
        return lhs.context_ == rhs.context_ && lhs.entry_ == rhs.entry_;
    }

    friend constexpr bool operator!=(callback lhs, callback rhs) noexcept
    {
        return !(lhs == rhs);
    }

private:
    constexpr callback(void* context, entry_type entry) noexcept
        : context_(context), entry_(entry)
    {
    }

    void* context_ = nullptr;
    entry_type entry_ = nullptr;
};

} // namespace pnxtypes
