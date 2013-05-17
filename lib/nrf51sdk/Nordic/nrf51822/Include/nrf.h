#ifndef NRF_H
#define NRF_H

#ifndef _WIN32

#include "compiler_abstraction.h"

#ifdef NRF51
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf51_deprecated.h"
#else
#error "Device is not supported."
#endif /* NRF51 */

#endif /* _WIN32 */ 

#endif /* NRF_H */

