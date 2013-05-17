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
 * @defgroup rpc_transport RPC Transport Module
 * @{
 * @ingroup app_common_serialization 
 *
 * @brief Transport module implementation for serialized messages.
 *
 * @details This module is responsible for transmitting and receiving messages between the user 
 *          (upper layer) and the lower layer, which is typically a hardware abstraction layer, 
 *          for example, a UART driver.
 *
 * @note    No support for re-entrant access unless explicitly stated in the interface 
 *          documentation.
 */
 
#ifndef RPC_TRANSPORT_H__
#define RPC_TRANSPORT_H__

#include <stdint.h>
#include "nrf_error.h"

/**@brief The types of events from the transport layer to the user. */
typedef enum
{
    /**< RPC_TRANSPORT_TX_PIPELINE_RESTART @todo: future enhancement. Tx-pipeline can be restarted after Tx-queue overflow */    
    
    RPC_TRANSPORT_CMD_READY,        /**< An event indicating that a Command has been received from the peer and is ready to be fetched by the user. */
    RPC_TRANSPORT_RESP_READY,       /**< An event indicating that a Command Response has been received from the peer and is ready to be fetched by the user. */
    RPC_TRANSPORT_EVT_READY,        /**< An event indicating that a Event has been received from the peer and is ready to be fetched by the user. */
#if 0 /**< @todo: redesign me */ 
    RPC_TRANSPORT_PKT_LOSS,         /**< An event indicating that a receive queue overflow has ocurred in the transport layer and incoming receive data was discarded. */
    RPC_TRANSPORT_ERROR,            /**< An event indicating that error has occured. */
#endif // 0    
    RPC_TRANSPORT_EVT_TYPE_MAX      /**< Upper bound. */  
} rpc_transport_evt_type_t;

/**@brief The types of packets. */
typedef enum
{
    RPC_TRANSPORT_CMD,              /**< Command packet type. */
    RPC_TRANSPORT_RESP,             /**< Command Response packet type. */
    RPC_TRANSPORT_EVT,              /**< Event packet type. */
    RPC_TRANSPORT_BOOTLOADER,       /**< Packet for Bootloader implementation. */
    RPC_TRANSPORT_PKT_TYPE_MAX      /**< Upper bound. */  
} rpc_transport_pkt_type_t;

/**@brief rpc_transport event handler callback function.
 *
 * @param[in] event The event occured.
 */
typedef void (*rpc_transport_event_handler_t)(rpc_transport_evt_type_t event);

/** @brief Register a user with the transport layer.
 *
 * The user can use this function to register for call backs in case a packet of interest was
 * received by the transport layer.
 *
 * @note Multiple registration requests for a same packet type will overwrite any possible existing 
 * registration.
 *
 * @param[in] pkt_type              The type of packet the user registers for.
 * @param[in] event_handler         The function that the transport layer calls when a packet is 
 *                                  received.
 *
 * @return                          NRF_SUCCESS.
 */
uint32_t rpc_transport_register(rpc_transport_pkt_type_t      pkt_type,
                                rpc_transport_event_handler_t event_handler);

/**@brief Open a transport channel and initialize the transport layer.
 *
 * @note Can be called multiple times. 
 *
 * @param[out] p_channel_open_count The amount of currently open channels.
 * 
 * @retval NRF_SUCCESS              Operation success.
 * @retval NRF_ERROR_INTERNAL       Operation failure. Internal error ocurred. 
 */
uint32_t rpc_transport_open(uint32_t * p_channel_open_count);

/**@brief Close a transport channel.
 *
 * @note Can be called multiple times for an open or unopen channel.
 * 
 * @param[out] p_channel_open_count The number of channels that are currently open.
 * 
 * @retval NRF_SUCCESS. 
 */
uint32_t rpc_transport_close(uint32_t * p_channel_open_count);

/**@brief Writes a single packet to the transmission queue.
 *
 * @note Successful execution of this function does not guarantee that peripheral transmission will 
 *       occur.
 *
 * @note In the case of a 0 byte packet length write request, a message will only consist of 
 *       transport module specific headers.
 * 
 * @param[in] pkt_type              Type of packet to send.
 * @param[in] p_buffer              The packet to send.
 * @param[in] length                Packet length (stored in p_buffer) in bytes.
 *
 * @retval NRF_SUCCESS              Operation success. Packet was written to the transmission queue.
 *
 * @retval NRF_ERROR_NO_MEM         @todo NOT SUPPORTED Operation failure. Transmission queue is full.
 *
 *                                  @todo User should wait for RPC_TRANSPORT_TX_PIPELINE_RESTART 
 *                                  event prior to issuing this operation again.
 *
 * @retval NRF_ERROR_DATA_SIZE      Operation failure. Packet size exceeds limit.   
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.  
 * @retval NRF_ERROR_INTERNAL       Operation failure. Internal error ocurred.
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. Channel is not open.
 */
uint32_t rpc_transport_packet_write(rpc_transport_pkt_type_t pkt_type,
                                    const uint8_t *          p_buffer,
                                    uint32_t                 length);

/**@brief Read a single packet, that has already been received, from receive queue.
 *  
 * @param[in]  pkt_type             Type of packet to read.
 * @param[in]  length               Size of the read buffer in bytes.
 * @param[out] p_buffer             Read buffer.
 * @param[out] p_read_length        Length of the packet, copied to the read buffer, in bytes.
 *
 * @retval NRF_SUCCESS              Operation success. Packet was read to the read buffer.
 * @retval NRF_ERROR_NULL           Operation failure. NULL pointer supplied.   
 * @retval NRF_ERROR_NOT_FOUND      Operation failure. No packet was available for read.
 * @retval NRF_ERROR_INVALID_STATE  Operation failure. Channel is not open. 
 * @retval NRF_ERROR_DATA_SIZE      Operation failure. 
 *                                  Packet did not fit in the user provided buffer.
 */
uint32_t rpc_transport_packet_read(rpc_transport_pkt_type_t pkt_type,
                                   uint32_t                 length,
                                   uint8_t *                p_buffer,
                                   uint32_t *               p_read_length);

#endif // RPC_TRANSPORT_H__

/** @} */
