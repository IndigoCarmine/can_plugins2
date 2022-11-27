#include <cstdint>

namespace cobs
{
    //you should output buffer size is 2 + input buffer size.
    void encode(const uint8_t input[], uint8_t output[], const uint8_t input_size)
    {
        uint8_t code_index = 1;
        uint8_t code = 1;
        for (uint8_t i = 0; i < input_size; i++)
        {
            if (input[i] == 0)
            {
                output[code_index] = 0;
                code_index = i + code + 1;
                code = 1;
            }
            else
            {
                output[code_index] = input[i];
                code++;
                if (code == 0xFF)
                {
                    output[code_index] = 0;
                    code_index = i + code + 1;
                    code = 1;
                }
            }
        }
        output[0] = code;
    }
    
    //you should output buffer size is input buffer size - 2.
    void decode(const uint8_t input[], uint8_t output[], const uint8_t input_size)
    {
        uint8_t code_index = 1;
        uint8_t code = input[0];
        uint8_t i = 0;
        while (code_index < input_size)
        {
            if (code != 0xFF && code_index + code < input_size)
            {
                for (uint8_t j = 1; j < code; j++)
                {
                    output[i] = input[code_index + j];
                    i++;
                }
                output[i] = 0;
                i++;
            }
            else
            {
                for (uint8_t j = 1; j < code; j++)
                {
                    output[i] = input[code_index + j];
                    i++;
                }
            }
            code_index = code_index + code + 1;
            code = input[code_index - 1];
        }
    }

    
} // namespace cobs
