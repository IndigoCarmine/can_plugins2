/// @todo statusの読み出しと書き込みについて、atomicに行われることを保証すべき

#include <array>

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
            uint8_t status;
        };
        enum class Status : uint8_t
        {
            Empty = 0,
            Used = 1,
        };

        std::array<Item, count> m_array;

    public:
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
                    value = T(item.value);
                    item.status = static_cast<uint8_t>(Status::Empty);
                    return true;
                }
            }
            return false;
        }
    };
}
