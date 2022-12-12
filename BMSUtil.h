#include <Arduino.h>
#include "CONFIG.H"
#include "Logger.h"

class BMSUtil {    
public:
    
    static uint8_t genCRC(uint8_t *input, int lenInput)
    {
        uint8_t generator = 0x07;
        uint8_t crc = 0;
  
        for (int x = 0; x < lenInput; x++)
        {
            crc ^= input[x]; /* XOR-in the next input byte */

            for (int i = 0; i < 8; i++)
            {
                if ((crc & 0x80) != 0)
                {
                    crc = (uint8_t)((crc << 1) ^ generator);
                }
                else
                {
                    crc <<= 1;
                }
            }
        }

        return crc;
    }

  
};
