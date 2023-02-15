#include <cstdint>
#include <array>

namespace cobs
{
    //you should output buffer size is 2 + input buffer size.
    void encode(const uint8_t data[], uint8_t return_data[], int data_size){
        int zero_index = data_size + 1;//this is return_data index
        return_data[zero_index] = 0x00;
        for(int i = data_size; i >0; i--){
            if(data[i-1] == 0x00){
                return_data[i] = (uint8_t)(zero_index - i);
                zero_index = i;
            }else{
                return_data[i] = data[i-1];
            }
        }
        return_data[0] = zero_index;
    }

    template <std::size_t N>
    std::array<uint8_t,N+2> encode(const std::array<uint8_t,N> data){
        std::array<uint8_t,N+2> return_data;
        int zero_index = N + 1;//this is return_data index
        return_data[zero_index] = 0x00;
        for(int i = N; i >0; i--){
            if(data[i-1] == 0x00){
                return_data[i] = (uint8_t)(zero_index - i);
                zero_index = i;
            }else{
                return_data[i] = data[i-1];
            }
        }
        return_data[0] = zero_index;
        return return_data;
    }

    //Consistent Overhead Byte Suffiing
    //data_size is data array size
    //return data array size should be data_size - 2
    void decode(const uint8_t data[], uint8_t return_data[], int data_size){
        uint8_t zero_index = data[0];
        //i is data index
        //and lastdata (data_size - 1)-1 is 0x00
        for(int i = 1; i < data_size - 1; i++){
            if(i == zero_index){
                return_data[i-1] = 0x00;
                zero_index = zero_index + data[i];
            }else{
                return_data[i-1] = data[i];
            }
        }
        return;
    }
    
    template <std::size_t N>
    std::array<uint8_t,N-2> decode(const std::array<uint8_t,N> data){
        if(N < 3){
            return {0x00};
        }
        std::array<uint8_t,N-2> return_data;

        uint8_t zero_index = data[0];
        //i is data index
        //and lastdata (data_size - 1)-1 is 0x00
        for(int i = 1; i < N - 1; i++){
            if(i == zero_index){
                return_data[i-1] = 0x00;
                zero_index = zero_index + data[i];
            }else{
                return_data[i-1] = data[i];
            }
        }
        return return_data;
    }

    
} // namespace cobs