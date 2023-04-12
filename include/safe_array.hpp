#pragma once

#include <array>
#include <atomic>
#include <utility>

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
            std::atomic<bool> has_element{false};
        };

        std::array<Item, count> m_array;

    public:
        // you MUST NOT call this concurrently.
        bool add(const T &value)
        {
            for (auto &item : m_array)
            {
                if (!item.has_element)
                {
                    item.value = value;
                    item.has_element = true;
                    return true;
                }
            }
            return false;
        }

        // you MUST NOT call this concurrently.
        bool pop(T &value)
        {
            for (auto &item : m_array)
            {
                if (item.has_element)
                {
                    value = std::move(item.value);
                    item.has_element = false;
                    return true;
                }
            }
            return false;
        }
    };
}
