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
 * @defgroup rpc_cmd_decoder Commands Decoder
 * @{
 * @ingroup ble_sdk_lib_serialization
 *
 * @brief   Decoder for serialized commands from Application Chip.
 *
 * @details This file contains the declaration of the function that decodes the serialized command
 *          from Application Chip and call the appropriate BLE stack API.
 */

#ifndef RPC_CMD_DECODER_H__
#define RPC_CMD_DECODER_H__

#include <stdint.h>

/**@brief Schedule an RPC command event to be processed in main-thread.
 *
 * @details The function will read the arrived packet from the transport layer
 *          which is passed for decoding by the rpc_cmd_decoder module.
 *
 * @param[in] p_event_data  Event data. This will be NULL as rpc_evt_schedule
 *                          does not set any data.
 * @param[in] event_size    Event data size. This will be 0 as rpc_evt_schedule
 *                          does not set any data.
 */
void rpc_cmd_handle(void * p_event_data, uint16_t event_size);

/**@brief       Function to process the encoded command from application chip.
 *
 * @details     This function will decode the encoded command and call the appropriate BLE Stack
 *              API. It will then create a command response packet with the return value from the
 *              stack API encoded in it and will send it to the transport layer for transmission to
 *              the application controller chip.

 * @param[in]   p_command The encoded command.
 * @param[in]   command_len Length of the encoded command.
 *
 * @return      NRF_SUCCESS if the decoding of the command was successful, the soft device API was
 *              called, and the command response was sent to peer, otherwise an error code.
 *              If the transport layer returns an error code while sending the Command Response, the
 *              same error code will be returned by this function (see @ref
 *              rpc_transport_packet_write for the list of error codes).
 */
uint32_t command_process(uint8_t * p_command, uint8_t command_len);

#endif
