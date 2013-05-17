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
 * @defgroup rpc_event_encoder Events Encoder
 * @{
 * @ingroup ble_sdk_lib_serialization
 *
 * @brief Event encoder for S110 SoftDevice serialization.
 *
 * @details This module provides functions for serializing S110 SoftDevice events.
 *
 */
#ifndef RPC_EVENT_ENCODER_H__
#define RPC_EVENT_ENCODER_H__

#include "ble.h"

/**@brief Encode a ble_evt_t. The function will call the transport layer with the serialized
 *        byte stream.
 *
 * @param[in]   p_ble_evt    S110 SoftDevice event to serialize.
 *
 */
void rpc_event_encoder_write(ble_evt_t * p_ble_evt);

#endif

/** @} */
