#if HAL_BRD_VERSION == 1
#include "mcuconf_gmmv1.h"
#elif HAL_BRD_VERSION == 2
#include "mcuconf_gmmv2.h"
#else
#error "Wrong HAL version defined"
#endif
