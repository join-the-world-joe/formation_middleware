
#include <stdint.h>
#include <stdio.h>

void dump_data8(uint8_t *data, uint16_t length)
{
    int i;

    for(i=0; i<length; i++) {
        if(i!=0 && i%16==0)
            printf("\n");
        printf("0x%x  ", data[i]);
    }
}

void dump_data16(uint16_t *data, uint16_t length)
{
    int i;

    for(i=0; i<length; i++) {
        if(i!=0 && i%16==0)
            printf("\n");
        printf("0x%x  ", data[i]);
    }
}


