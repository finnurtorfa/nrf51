/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
 
#include <stdint.h>
#include "nrf51.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_error.h"
#include "app_util.h"
#include "rpc_transport.h"
#include "ble_rpc_cmd_encoder.h"
#include "ble_rpc_event_decoder.h"


static void lfclk_config(void)
{
  NRF_CLOCK->LFCLKSRC = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_LFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
  {
  }
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
}


uint32_t sd_softdevice_enable(nrf_clock_lfclksrc_t           clock_source,
                              softdevice_assertion_handler_t assertion_handler)
{
    uint32_t err_code;
    
    lfclk_config();

    // Initialized Event Decoder
    err_code = ble_rpc_event_decoder_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Initialize Command Encoder
    err_code = ble_rpc_cmd_encoder_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    
    return NRF_SUCCESS;
}


uint32_t sd_app_event_wait(void)
{
    // Use directly __WFE and __SEV macros since the SoftDevice is not available
    // Wait for event
    __WFE();

    // Clear Event Register
    __SEV();
    __WFE();

    return NRF_SUCCESS;
}


uint32_t sd_nvic_EnableIRQ(IRQn_Type IRQn)
{
    uint32_t prio = NVIC_GetPriority(IRQn);
    
    if (prio == 0)
    {
        NVIC_SetPriority(IRQn, APP_IRQ_PRIORITY_LOW);
    }
    
    // Enable the IRQ
    NVIC_EnableIRQ(IRQn);
    
    return NRF_SUCCESS;
}


uint32_t sd_nvic_ClearPendingIRQ(IRQn_Type IRQn)
{
    // Disable the IRQ
    NVIC_DisableIRQ(IRQn);
    
    return NRF_SUCCESS;
}


uint32_t sd_radio_notification_cfg_set(nrf_radio_notification_type_t     type,
                                       nrf_radio_notification_distance_t distance)
{
	return NRF_SUCCESS;
}


uint32_t sd_nvic_critical_region_enter(uint8_t * p_is_nested_critical_region)
{
    return NRF_SUCCESS;
}


uint32_t sd_nvic_critical_region_exit(uint8_t is_nested_critical_region)
{
    return NRF_SUCCESS;
}


uint32_t sd_nvic_SetPriority(IRQn_Type IRQn, nrf_app_irq_priority_t priority)
{
    NVIC_SetPriority(IRQn, (uint32_t)priority);
    return NRF_SUCCESS;
}
