#pragma once

#include "usbd_cdc_vcp.h"

#define DEBUG   1

#if DEBUG
    #define DEBUG_PRINTF      usb_printf    
#else
    #define DEBUG_PRINTF(fmt, ...)     
#endif


