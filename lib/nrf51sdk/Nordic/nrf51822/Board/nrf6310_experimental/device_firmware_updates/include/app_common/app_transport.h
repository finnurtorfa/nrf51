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
 
/** @file
 *
 * @defgroup app_transport Transport module
 * @{
 * @ingroup app_common_serialization
 *
 * @brief Transport module implementation.
 */
 
#ifndef APP_TRANSPORT_H__
#define APP_TRANSPORT_H__

#include <stdint.h>
#include "nrf_error.h"

/**@brief Event types for callback interface. */
typedef enum
{
    APP_TRANSPORT_RX_DATA_READY,    /**< An event indicating that RX data is available for read. */
    APP_TRANSPORT_ERROR,            /**< An event indicating that an error has occured. */
    APP_TRANSPORT_MAX               /**< Upper bound. */
} app_transport_evt_type_t;

/**@brief app_transport event handler callback function.
 *
 * @param[in] event The event occured.
 */
typedef void (*app_transport_event_handler_t)(app_transport_evt_type_t event);

/**@brief Open the transport channel.
 *
 * @param[in] event_handler Function to be called in case of an event.
 *
 * @retval    NRF_SUCCESS        Operation success.
 * @retval    NRF_ERROR_NULL     Operation failure. NULL pointer supplied.
 * @retval    NRF_ERROR_INTERNAL Operation failure. Internal error ocurred.
 */
uint32_t app_transport_open(app_transport_event_handler_t event_handler);

/**@brief Close the transport channel.
 *
 * @return NRF_SUCCESS.
 */
uint32_t app_transport_close(void);

/**@brief Write to transmit queue.
 *
 * @note Completion of this method does not guarantee that peripheral transmission has occurred.
 *
 * @note Issuing this method for an unopened channel results in undefined behavior.
 *
 * @param[in] p_buffer              Write buffer.
 * @param[in] length                Number of bytes to write to transmit queue. 
 * 
 * @retval    NRF_SUCCESS           Operation success.
 * @retval    NRF_ERROR_NULL        Operation failure. NULL pointer supplied.  
 * @retval    NRF_ERROR_INTERNAL    Operation failure. Internal error ocurred.
 */
uint32_t app_transport_write(const uint8_t * p_buffer, uint32_t length);

/**@brief Read from transmit queue.
 *
 * @note Issuing this method for an unopened channel results in undefined behavior.
 *
 * @param[in]  p_buffer             Read buffer.
 * @param[in]  length               Maxium number of bytes to read from read queue. 
 * @param[out] p_actual_length      Number of bytes that were available for read and were actually
 *                                  read from read queue. 
 * 
 * @retval    NRF_SUCCESS           Operation success. Actual read amount is equal to requested 
 *                                  read amount.
 * @retval    NRF_ERROR_NULL        Operation failure. NULL pointer supplied. 
 * @retval    NRF_ERROR_DATA_SIZE   Operation failure. Actual read amount is not equal to requested
 *                                  read amount. User should wait for APP_TRANSPORT_RX_DATA_READY
 *                                  event prior issuing this operation again.
 */
uint32_t app_transport_read(uint8_t *  p_buffer, 
                            uint32_t   length, 
                            uint32_t * p_actual_length);

#endif // APP_TRANSPORT_H__

/** @} */
