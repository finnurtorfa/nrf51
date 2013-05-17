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
 
#define RPC_BLE_FIELD_PRESENT       0x01 /**< Value to indicate that an optional field is encoded in the serialized packet, e.g. white list. */
#define RPC_BLE_FIELD_NOT_PRESENT   0x00 /**< Value to indicate that an optional field is not encoded in the serialized packet. */
#define RPC_BLE_FIELD_LEN           1u   /**< Optional field length size in bytes. */

#define BLE_ERR_CODE_SIZE           4u   /**< BLE API err_code size in bytes. */
