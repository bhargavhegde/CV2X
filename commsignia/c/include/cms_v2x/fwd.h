/** @file
@copyright
(C) Commsignia Ltd. - All Rights Reserved.
Unauthorised copying of this file, via any medium is strictly prohibited.
Proprietary and confidential.
@date 2020-2023
*/

#ifndef CMS_V2X_FWD_H_
#define CMS_V2X_FWD_H_

/** @file
@brief Forward declaration of types directly used by API.
*/

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
Shorthand for 'packed' annotation.
@ingroup common
*/
#define CMS_PACKED __attribute__((__packed__))


typedef struct cms_session_t cms_session_t;

typedef struct cms_buffer_view_t cms_buffer_view_t;
typedef struct cms_mutable_buffer_t cms_mutable_buffer_t;

typedef struct cms_licence_data_t cms_licence_data_t;

typedef struct cms_radio_rx_params_t cms_radio_rx_params_t;
typedef struct cms_radio_tx_params_t cms_radio_tx_params_t;

typedef struct cms_sec_dot2_rx_info_t cms_sec_dot2_rx_info_t;
typedef struct cms_sec_dot2_tx_sign_info_t cms_sec_dot2_tx_sign_info_t;
typedef struct cms_sec_dot2_tx_info_t cms_sec_dot2_tx_info_t;

typedef struct cms_fac_rx_sec_info_t cms_fac_rx_sec_info_t;
typedef struct cms_fac_notif_data_t cms_fac_notif_data_t;

typedef struct cms_wsmp_rx_hdr_info_t cms_wsmp_rx_hdr_info_t;
typedef struct cms_wsmp_tx_hdr_info_t cms_wsmp_tx_hdr_info_t;
typedef struct cms_wsmp_rx_notif_data_t cms_wsmp_rx_notif_data_t;
typedef struct cms_wsmp_send_data_t cms_wsmp_send_data_t;

typedef struct cms_validity_region_t cms_validity_region_t;

typedef struct cms_gn_rx_params_t cms_gn_rx_params_t;
typedef struct cms_gn_tx_params_t cms_gn_tx_params_t;
typedef struct cms_gn_rx_notif_data_t cms_gn_rx_notif_data_t;

typedef struct cms_btp_params_t cms_btp_params_t;
typedef struct cms_gn_traffic_class_t cms_gn_traffic_class_t;
typedef struct cms_gn_full_pos_t cms_gn_full_pos_t;
typedef struct cms_gn_dest_area_t cms_gn_dest_area_t;
typedef struct cms_gn_hdr_info_t cms_gn_hdr_info_t;
typedef struct cms_gn_send_data_t cms_gn_send_data_t;

typedef struct cms_dsmp_hdr_info_t cms_dsmp_hdr_info_t;
typedef struct cms_dsmp_ext_t cms_dsmp_ext_t;
typedef struct cms_dsmp_rx_notif_data_t cms_dsmp_rx_notif_data_t;
typedef struct cms_dsmp_send_data_t cms_dsmp_send_data_t;

typedef struct cms_ssp_t cms_ssp_t;
typedef struct cms_sec_hash_t cms_sec_hash_t;
typedef struct cms_sec_signature_t cms_sec_signature_t;
typedef struct cms_sec_pubkey_t cms_sec_pubkey_t;
typedef struct cms_sec_ecdsa_verify_input_t cms_sec_ecdsa_verify_input_t;
typedef struct cms_sec_hsm_slot_info_t cms_sec_hsm_slot_info_t;
typedef struct cms_sec_hsm_key_info_t cms_sec_hsm_key_info_t;

typedef struct cms_sti_station_id_t cms_sti_station_id_t;

typedef struct cms_cpm_position_t cms_cpm_position_t;
typedef struct cms_cpm_altitude_t cms_cpm_altitude_t;
typedef struct cms_cpm_ground_speed_t cms_cpm_ground_speed_t;
typedef struct cms_cpm_vertical_speed_t cms_cpm_vertical_speed_t;
typedef struct cms_cpm_ground_acceleration_t cms_cpm_ground_acceleration_t;
typedef struct cms_cpm_vertical_acceleration_t cms_cpm_vertical_acceleration_t;
typedef struct cms_cpm_yaw_angle_t cms_cpm_yaw_angle_t;
typedef struct cms_cpm_object_size_t cms_cpm_object_size_t;
typedef struct cms_cpm_detection_t cms_cpm_detection_t;
typedef struct cms_cpm_matched_position_t cms_cpm_matched_position_t;
typedef struct cms_cpm_perceived_object_t cms_cpm_perceived_object_t;
typedef struct cms_cpm_free_space_point_t cms_cpm_free_space_point_t;
typedef struct cms_cpm_free_space_polygon_t cms_cpm_free_space_polygon_t;
typedef struct cms_cpm_free_space_circle_t cms_cpm_free_space_circle_t;
typedef struct cms_cpm_free_space_ellipse_t cms_cpm_free_space_ellipse_t;
typedef struct cms_cpm_free_space_rectangle_t cms_cpm_free_space_rectangle_t;
typedef struct cms_cpm_free_space_area_data_t cms_cpm_free_space_area_data_t;
typedef struct cms_cpm_free_space_area_t cms_cpm_free_space_area_t;
typedef struct cms_cpm_free_space_t cms_cpm_free_space_t;
typedef struct cms_cpm_sensor_info_t cms_cpm_sensor_info_t;
typedef struct cms_cpm_element_ref_t cms_cpm_element_ref_t;


typedef struct cms_sec_hsm_get_slots_in_t cms_sec_hsm_get_slots_in_t;
typedef struct cms_sec_hsm_get_slots_result_t cms_sec_hsm_get_slots_result_t;

typedef struct cms_vam_path_prediction_t cms_vam_path_prediction_t;
typedef struct cms_vam_path_prediction_point_t cms_vam_path_prediction_point_t;

#ifdef __cplusplus
}
#endif

#endif /* CMS_V2X_FWD_H_ */
