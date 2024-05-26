/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_STAT_PRINT_H_
#define CMS_V2X_STAT_PRINT_H_

/** @file
@brief Statistics print helpers
*/

#include <stdint.h>

#include <cms_v2x/fwd.h>
#include <cms_v2x/common_types.h>
#include <cms_v2x/stat_types.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
Print counters
@ingroup stat
Prints statistics counters in JSON format
@experimentalapi
@param counters         Statistic counters to be printed
@param length           Length of counters array
@param[inout] buffer    Buffer containing the counters deparsed to JSON format.
                        If it contains any string it tries to parse it as a JSON string and
                        append the counters to the JSON.
@return true in case of error
*/
bool cms_stat_print_counters(const cms_stat_item_t* counters,
                             uint32_t length,
                             cms_mutable_buffer_t* buffer);

/**
Pretty print counters
@ingroup stat
Prints statistics counters in JSON format with 2 spaces indentation
@experimentalapi
@param counters         Statistic counters to be printed
@param length           Length of counters array
@param[inout] buffer    Buffer containing the counters deparsed to JSON format.
                        If it contains any string it tries to parse it as a JSON string and
                        append the counters to the JSON.
@return true in case of error
*/
bool cms_stat_pretty_print_counters(const cms_stat_item_t* counters,
                                    uint32_t length,
                                    cms_mutable_buffer_t* buffer);

/**
Print device status
@ingroup stat
@experimentalapi
Prints device and miscellaneous data in JSON format
@param device_status    Device and miscellaneous data to be printed
@param[inout] buffer    Buffer containing the device and miscellaneous data deparsed to JSON format.
                        If it contains any string it tries to parse it as a JSON string and append the
                        device_status to the JSON.
@return true in case of error
*/
bool cms_stat_print_device_status(const cms_stat_device_status_t* device_status,
                                  cms_mutable_buffer_t* buffer);

/**
Pretty print device status
@ingroup stat
@experimentalapi
Prints device and miscellaneous data in JSON format with 2 spaces indentation
@param device_status    Device and miscellaneous data to be printed
@param[inout] buffer    Buffer containing the device and miscellaneous data deparsed to JSON format.
                        If it contains any string it tries to parse it as a JSON string and append the
                        device_status to the JSON.
@return true in case of error
*/
bool cms_stat_pretty_print_device_status(const cms_stat_device_status_t* device_status,
                                         cms_mutable_buffer_t* buffer);

/**
Add MAC address to device status
@ingroup stat
@experimentalapi
Prints mac address into device status json
@param interface_id     interface ID to be appended to JSON key
@param mac_address      MAC address to be used as JSON value
@param[inout] buffer    Buffer containing JSON to be extended with MAC address.
@return true in case of error
*/
bool cms_stat_print_mac_addr_to_device_status(cms_interface_id_t interface_id,
                                              const cms_mac_addr_t mac_address,
                                              cms_mutable_buffer_t* buffer);

/**
Add pretty printed MAC address to device status
@ingroup stat
@experimentalapi
Prints mac address into device status json
@param interface_id     interface ID to be appended to JSON key
@param mac_address      MAC address to be used as JSON value
@param[inout] buffer    Buffer containing JSON to be extended with MAC address.
@return true in case of error
*/
bool cms_stat_pretty_print_mac_addr_to_device_status(cms_interface_id_t interface_id,
                                                     const cms_mac_addr_t mac_address,
                                                     cms_mutable_buffer_t* buffer);

/**
Add Station ID to device status
@ingroup stat
@experimentalapi
Prints StationId and its short form into device status json
@param station_id       stationId - full long format
@param[inout] buffer    Buffer containing JSON to be extended with StationID.
@return true in case of error
*/
bool cms_stat_print_station_id_to_device_status(const cms_sti_station_id_t* station_id,
                                                cms_mutable_buffer_t* buffer);

/**
Add pretty printed Station ID to device status
@ingroup stat
@experimentalapi
Prints StationId and its short form into device status json
@param station_id       stationId - full long format
@param[inout] buffer    Buffer containing JSON to be extended with StationID.
@return true in case of error
*/
bool cms_stat_pretty_print_station_id_to_device_status(const cms_sti_station_id_t* station_id,
                                                       cms_mutable_buffer_t* buffer);

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_STAT_PRINT_H_ */
