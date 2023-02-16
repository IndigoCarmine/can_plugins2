#include <cstdint>
#include <vector>

namespace cobs
{
    //you should output buffer size is 2 + input buffer size.
    void encode(const uint8_t data[], uint8_t return_data[],std::size_t data_size){
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

    std::vector<uint8_t> encode(const std::vector<uint8_t> data){
        std::vector<uint8_t> return_data(data.size() + 2);
        encode(data.data(), return_data.data(), data.size());
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

    std::vector<uint8_t> decode(const std::vector<uint8_t> data){
        std::vector<uint8_t> return_data(data.size() - 2);
        decode(data.data(), return_data.data(), data.size());
        return return_data;
    }   
} // namespace cobs



//this is test code. it can work.

// bool is_equal(uint8_t*a,uint8_t*b,int len){
//     for(int i = 0;i<len;i++){
//         if(a[i]!=b[i])return false;
//     }
//     return true;
// }

// int main(void){
//     // Your code here!
//     uint8_t HELLO[] = {'H','E','l',0x00,1};
//     uint8_t en[7];
//     uint8_t de[5];
//     cobs::encode(HELLO,en,5);
//     cobs::decode(en,de,7);
    
//     if(is_equal(HELLO,de,5))cout<<"True";
// }