#if HAL_BRD_VERSION == 1
#include "board_gmmv1.c"
#elif HAL_BRD_VERSION == 2
#include "board_gmmv2.c"
#else
#error "Wrong HAL version defined"
#endif
