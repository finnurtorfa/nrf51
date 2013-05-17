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
 * @defgroup rpc_op_codes Operation Codes
 * @{
 * @ingroup ble_sdk_lib_serialization
 *
 * @brief Header file specifying Operation codes for mapping RPC calls to SoftDevice functions.
 *
 */

/**@enum rpc_ble_op_code_t
 *
 * @details Operation Codes enumeration.<BR>
 *          This enumeration contains the definitions for en-/ decoding a SoftDevice API call when serializing a function.
 */
typedef enum
{
    RPC_SD_BLE_INVALID_OPCODE                                  = 0x00, /**< Identifies an invalid Operation Code. */

    // @todo Add offset to BLE command group,                          /**< */
    // look at H5 to get group bits.                                   /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_TX_BUFFER_COUNT_GET,                    /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_UUID_VS_ADD,                            /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_UUID_DECODE,                            /**< */
    RPC_SD_BLE_UUID_ENCODE,                                            /**< Value identifying the SoftDevice call @ref sd_ble_uuid_encode. */
    NOT_IMPLEMENTED_RPC_SD_BLE_VERSION_GET,                            /**< */

    // @todo Add offset to BLE command group,                          /**< */
    // look at H5 to get group bits.                                   /**< */
    RPC_SD_BLE_GAP_ADDRESS_SET                                 = 0x10, /**< Value identifying the SoftDevice call @ref sd_ble_gap_address_set. */
    RPC_SD_BLE_GAP_ADDRESS_GET,                                        /**< Value identifying the SoftDevice call @ref sd_ble_gap_address_get. */
    RPC_SD_BLE_GAP_ADV_DATA_SET,                                       /**< Value identifying the SoftDevice call @ref sd_ble_gap_adv_data_set. */
    RPC_SD_BLE_GAP_ADV_START,                                          /**< Value identifying the SoftDevice call @ref sd_ble_gap_adv_start. */
    NOT_IMPLEMENTED_RPC_SD_BLE_GAP_ADV_STOP,                           /**< */
    RPC_SD_BLE_GAP_CONN_PARAM_UPDATE,                                  /**< Value identifying the SoftDevice call @ref sd_ble_gap_conn_param_update.*/
    RPC_SD_BLE_GAP_DISCONNECT,                                         /**< Value identifying the SoftDevice call @ref sd_ble_gap_disconnect. */
    NOT_IMPLEMENTED_RPC_SD_BLE_GAP_TX_POWER_SET,                       /**< */
    RPC_SD_BLE_GAP_APPEARANCE_SET,                                     /**< Value identifying the SoftDevice call @ref sd_ble_gap_appearance_set. */
    RPC_SD_BLE_GAP_APPEARANCE_GET,                                     /**< Value identifying the SoftDevice call @ref sd_ble_gap_appearance_get. */
    RPC_SD_BLE_GAP_PPCP_SET,                                           /**< Value identifying the SoftDevice call @ref sd_ble_gap_ppcp_set. */
    RPC_SD_BLE_GAP_PPCP_GET,                                           /**< Value identifying the SoftDevice call @ref sd_ble_gap_ppcp_get.*/
    RPC_SD_BLE_GAP_DEVICE_NAME_SET,                                    /**< Value identifying the SoftDevice call @ref sd_ble_gap_device_name_set. */
    RPC_SD_BLE_GAP_DEVICE_NAME_GET,                                    /**< Value identifying the SoftDevice call @ref sd_ble_gap_device_name_get. */
    NOT_IMPLEMENTED_RPC_SD_BLE_GAP_AUTHENTICATE,                       /**< */
    RPC_SD_BLE_GAP_SEC_PARAMS_REPLY,                                   /**< Value identifying the SoftDevice call @ref sd_ble_gap_sec_params_reply. */
    NOT_IMPLEMENTED_RPC_SD_BLE_GAP_AUTH_KEY_REPLY,                     /**< */
    RPC_SD_BLE_GAP_SEC_INFO_REPLY,                                     /**< Value identifying the SoftDevice call @ref sd_ble_gap_sec_info_reply. */
    NOT_IMPLEMENTED_RPC_SD_BLE_GAP_CONN_SEC_GET,                       /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GAP_RSSI_START,                         /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GAP_RSSI_STOP,                          /**< */

    // @todo Add offset to BLE command group,
    // look at H5 to get group bits.
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_PRIMARY_SERVICES_DISCOVER = 0x30, /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_RELATIONSHIPS_DISCOVER,           /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_CHARACTERISTICS_DISCOVER,         /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_DESCRIPTORS_DISCOVER,             /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_CHAR_VALUE_BY_UUID_READ,          /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_READ,                             /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_CHAR_VALUES_READ,                 /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_WRITE,                            /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTC_HV_CONFIRM,                       /**< */

    // @todo Add offset to BLE command group,
    // look at H5 to get group bits.
    RPC_SD_BLE_GATTS_SERVICE_ADD                               = 0x40, /**< Value identifying the SoftDevice call @ref sd_ble_gatts_service_add. */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTS_INCLUDE_ADD,                      /**< */
    RPC_SD_BLE_GATTS_CHARACTERISTIC_ADD,                               /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTS_DESCRIPTOR_ADD,                   /**< */
    RPC_SD_BLE_GATTS_VALUE_SET,                                        /**< Value identifying the SoftDevice call @ref sd_ble_gatts_value_set. */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTS_VALUE_GET,                        /**< */
    RPC_SD_BLE_GATTS_HVX,                                              /**< Value identifying the SoftDevice call @ref sd_ble_gatts_hvx. */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTS_SERVICE_CHANGED,                  /**< */
    NOT_IMPLEMENTED_RPC_SD_BLE_GATTS_RW_AUTHORIZE_REPLY,               /**< */
    RPC_SD_BLE_GATTS_SYS_ATTR_SET,                                     /**< Value identifying the SoftDevice call @ref sd_ble_gatts_sys_attr_set. */
    RPC_SD_BLE_GATTS_SYS_ATTR_GET,                                     /**< Value identifying the SoftDevice call @ref sd_ble_gatts_sys_attr_get. */

    RPC_SD_POWER_SYSTEM_OFF                                       = 0x50, /**< Value identifying the SoftDevice call @ref sd_power_system_off. */
} rpc_ble_op_code_t;

#define BLE_OP_CODE_SIZE    1u /**< Operation code field size in bytes. */

/** @} */
