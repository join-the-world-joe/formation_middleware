
#include <stdint.h>

#include "platform_endian.h"

int platform_endian()
{
    int ret = -1;

    union {
        uint16_t s;
	    char c[sizeof(uint16_t)];
    }un;

    un.s = 0x0102;

    if(un.c[0]==2&&un.c[1]==1) {
	    return __LITTLE_ENDIAN__;
    } else if(un.c[0]==1&&un.c[1]==2) {
	    return __BIG_ENDIAN__;
    } else {	
	    return -1;
    }

    return ret;
}


