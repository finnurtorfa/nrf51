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
 * @defgroup rpc_cmd_defines Defines related to serialized BLE commands.
 * @{
 * @ingroup ble_sdk_lib
 *
 * @brief Defines for serialized BLE commands.
 *
 */

 
#ifndef RPC_CMD_DEFINES_H__
#define RPC_CMD_DEFINES_H__

#define RPC_CMD_OP_CODE_POS                     0   /**< Position of the Op Code in the command buffer.*/
#define RPC_CMD_DATA_POS                        1   /**< Position of the data in the command buffer.*/

#define RPC_MAX_CMD_RSP_LEN                     5   /**< Maximum length of the command response packet sent to the transport layer.*/
#define RPC_MAX_CMD_RSP_DATA_LEN                128 /**< Maximum length of the command response packet with additional data sent to the transport layer.*/

#define RPC_CMD_RESP_OP_CODE_POS                0   /**< Position of the Op Code in the command response buffer.*/
#define RPC_CMD_RESP_STATUS_POS                 1   /**< Position of the status field in the command response buffer.*/

#define MAX_EVENT_LEN                           56  /**< Maximum possible length, in bytes, of an encoded BLE Event. (Must be a multiple of 4, to make sure memory is aligned)*/

#endif // RPC_CMD_DEFINES_H__

/** @} */
