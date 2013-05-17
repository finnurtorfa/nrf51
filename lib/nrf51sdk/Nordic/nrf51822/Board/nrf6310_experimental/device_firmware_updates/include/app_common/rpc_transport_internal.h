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
 
/** @file RPC transport module internal definitions.
 *
 * @defgroup rpc_transport RPC Transport Module
 * @{
 * @ingroup app_common_serialization 
 *
 */
 
#ifndef RPC_TRANSPORT_INTERNAL_H__
#define RPC_TRANSPORT_INTERNAL_H__
 
#define RPC_LENGTH_FIELD_SIZE      4u                                                   /**< Transport frame length field size in bytes. */
#define RPC_PACKET_TYPE_FIELD_SIZE 4u                                                   /**< Transport frame type field size in bytes. */ 
#define RPC_HDR_SIZE               (RPC_LENGTH_FIELD_SIZE + RPC_PACKET_TYPE_FIELD_SIZE) /**< Transport frame header size in bytes. */ 

#define RPC_TX_BUF_SIZE            256u                                                 /**< TX buffer size in number of bytes. */ 
#define RPC_RX_BUF_SIZE            RPC_TX_BUF_SIZE                                      /**< RX buffer size in number of bytes. */ 

#endif // RPC_TRANSPORT_INTERNAL_H__ 

/** @} */
