#pragma once

#include <array>
#include <functional>
#include <atomic>

namespace safe_array
{
    // it is for multithreading environment
    // it is safe for adding and poping elements
    template <typename T, std::size_t count>
    class SafeArray
    {
    private:
        struct Item
        {
            T value;
            std::atomic<uint8_t> status;
        };
        enum class Status : uint8_t
        {
            Empty = 0,
            Used = 1,
        };

        std::array<Item, count> m_array;
        // if compare is true, then first element is bigger than second
        std::function<bool(const T &, const T &)> compare;

    public:
        SafeArray(std::function<bool(const Item<T> &, const Item<T> &)> compare) : compare(compare) {}
        // this function is not used from multiple threads
        bool add(const T &value)
        {
            for (auto &item : m_array)
            {
                if (item.status == static_cast<uint8_t>(Status::Empty))
                {
                    item.value = T(value);
                    item.status = static_cast<uint8_t>(Status::Used);
                    return true;
                }
            }
            return false;
        }

        // this function is not used from multiple threads
        bool pop(T &value)
        {
            for (auto &item : m_array)
            {

                if (item.status == static_cast<uint8_t>(Status::Used))
                {
                    value = item.value;
                    item.status = static_cast<uint8_t>(Status::Empty);
                    return true;
                }
            }
            return false;
        }
    };
} // namespace safe_list
