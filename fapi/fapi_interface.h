#ifndef _FAPI_INTERFACE_H_
#define _FAPI_INTERFACE_H_

#include "stddef.h"
#include "stdint.h"

#define VAR_SIZE	1

#define FAPI_GET_MSG_PTR_VAR_SIZE(x, list_var_name, len) \
	((len> 0) ? \
			(sizeof(typeof(*x)) + sizeof(typeof(x->list_var_name)) * (len - VAR_SIZE)) : \
			(sizeof(typeof(*x))))

#define FAPI_GET_MSG_VAR_SIZE(x, list_var_name, len) \
	((len > 0) ? \
			(sizeof(typeof(x)) + sizeof(typeof(x.list_var_name)) * (x.len - VAR_SIZE)) : \
			(sizeof(typeof(x))))

// Constants - update based on implementation
#define FAPI_MAX_PHY_RF_INSTANCES 2
#define FAPI_PNF_PARAM_GENERAL_LOCATION_LENGTH 16
#define FAPI_PNF_PARAM_GENERAL_OUI_LENGTH 3
#define FAPI_MAX_NUM_RF_BANDS 16

// The following definition control the size of arrays used in the interface.
// These may be changed if desired. They are used in the encoder to make sure 
// that the user has not specified a 'count' larger than the max array, and also
// used by the decoder when decode an array. If the 'count' received is larger
// than the array it is to be stored in the decode fails. 
#define FAPI_MAX_NUM_ANTENNAS 8
#define FAPI_MAX_NUM_SUBBANDS 13
#define FAPI_MAX_BF_VECTORS 8
#define FAPI_MAX_CC 1
#define FAPI_MAX_NUM_PHYSICAL_ANTENNAS 8
#define FAPI_MAX_RSSI 8
#define FAPI_MAX_PSC_LIST 32
#define FAPI_MAX_PCI_LIST 32
#define FAPI_MAX_CARRIER_LIST 32
#define FAPI_MAX_ARFCN_LIST 128
#define FAPI_MAX_LTE_CELLS_FOUND 8
#define FAPI_MAX_UTRAN_CELLS_FOUND 8
#define FAPI_MAX_GSM_CELLS_FOUND 8
#define FAPI_MAX_NB_IOT_CELLS_FOUND 8
#define FAPI_MAX_SI_PERIODICITY 8
#define FAPI_MAX_SI_INDEX 8
#define FAPI_MAX_MIB_LENGTH 32
#define FAPI_MAX_SIB_LENGTH 256
#define FAPI_MAX_SI_LENGTH 256
#define FAPI_MAX_OPAQUE_DATA 64
#define FAPI_MAX_NUM_SCHEDULED_UES 8 // Used in the TPM structure
#define FAPI_MAX_PNF_PHY 5
#define FAPI_MAX_PNF_PHY_RF_CONFIG 5
#define FAPI_MAX_PNF_RF  5
#define FAPI_MAX_NMM_FREQUENCY_BANDS 32
#define FAPI_MAX_RECEIVED_INTERFERENCE_POWER_RESULTS 100
#define FAPI_MAX_UL_DL_CONFIGURATIONS 5
#define FAPI_MAX_CSI_RS_RESOURCE_CONFIG 4
#define FAPI_MAX_ANTENNA_PORT_COUNT 8
#define FAPI_MAX_EPDCCH_PRB 8
#define FAPI_MAX_TX_PHYSICAL_ANTENNA_PORTS 8
#define FAPI_MAX_NUMBER_ACK_NACK_TDD 8
#define FAPI_MAX_RO_DL 8

#define FAPI_HEADER_LENGTH 8
#define FAPI_P7_HEADER_LENGTH 16

#define FAPI_VENDOR_EXTENSION_MIN_TAG_VALUE 0xF000
#define FAPI_VENDOR_EXTENSION_MAX_TAG_VALUE 0xFFFF

#define FAPI_VERSION_3_0_11	0x000
#define FAPI_VERSION_3_0_12    0x001

// The IANA agreed port definition of the P5 SCTP VNF enpoint 
// http://www.iana.org/assignments/service-names-port-numbers/service-names-port-numbers.xhtml?search=7701
#define FAPI_P5_SCTP_PORT		7701
#if 0
typedef unsigned int	uint32_t;
typedef unsigned short	uint32_t;
typedef unsigned char	uint32_t;
typedef signed int		int32_t;
typedef signed short	int16_t;
typedef signed char		int8_t;
#endif

typedef struct __attribute__((packed)) {
	uint32_t message_id;
	uint32_t message_length;
} fapi_l1_message_header_t;

#define FAPI_PHY_ID_NA 0

//#define FAPI_P7_GET_MORE(_mss) ( ((_mss) & 0x80) >> 7 )
//#define FAPI_P7_GET_SEGMENT(_mss) ( ((_mss) & 0x70) >> 4 )
#define FAPI_P7_GET_MORE(_mss) ( ((_mss) & 0x8000) >> 15 )
#define FAPI_P7_GET_SEGMENT(_mss) ( ((_mss) & 0x7F00) >> 8 )
#define FAPI_P7_GET_SEQUENCE(_mss) ( (_mss) & 0x00FF )
#define FAPI_P7_SET_MSS(_more, _segm, _sequ) ( (((_more) & 0x1) << 7) | (((_segm) & 0x7) << 4) | ((_sequ) & 0xF) )

typedef struct __attribute__((packed)) {
	uint32_t tag;
	uint32_t length;
} fapi_tl_t;
#define FAPI_TAG_LENGTH_PACKED_LEN 4

// Convenience methods to convert between SFN/SFN formats
#define FAPI_SFNSF2DEC(_sfnsf) ((((_sfnsf) >> 4) * 10) + ((_sfnsf) & 0xF))
#define FAPI_SFNSFDEC2SFNSF(_sfnsf_dec) ((((_sfnsf_dec) / 10) << 4) | (((_sfnsf_dec) - (((_sfnsf_dec) / 10) * 10)) & 0xF))

#define FAPI_SFNSF2SFN(_sfnsf) ((_sfnsf) >> 4)
#define FAPI_SFNSF2SF(_sfnsf) ((_sfnsf) & 0xF)

#define FAPI_MAX_SFNSFDEC 10240

typedef fapi_tl_t fapi_vendor_extension_tlv_t[VAR_SIZE];


// nFAPI Message IDs
typedef enum {
	FAPI_PARAM_REQUEST = 0x00,
	FAPI_PARAM_RESPONSE,
	FAPI_CONFIG_REQUEST,
	FAPI_CONFIG_RESPONSE,
	FAPI_START_REQUEST,
	FAPI_STOP_REQUEST,
	FAPI_STOP_INDICATION,
	FAPI_UE_CONFIG_REQUEST,
	FAPI_UE_CONFIG_RESPONSE,
	FAPI_ERROR_INDICATION,
	FAPI_UE_RELEASE_REQUEST,
	FAPI_UE_RELEASE_RESPONSE,

	FAPI_DL_CONFIG_REQUEST = 0x80,
	FAPI_UL_CONFIG_REQUEST,
	FAPI_SUBFRAME_INDICATION,
	FAPI_HI_DCI0_REQUEST,
	FAPI_TX_REQUEST,
	FAPI_HARQ_INDICATION,
	FAPI_CRC_INDICATION,
	FAPI_RX_ULSCH_INDICATION,
	FAPI_RACH_INDICATION,
	FAPI_SRS_INDICATION,
	FAPI_RX_SR_INDICATION,
	FAPI_RX_CQI_INDICATION,
	FAPI_LBT_DL_CONFIG_REQUEST,
	FAPI_LBT_DL_INDICATION,
	FAPI_NB_HARQ_INDICATION,
	FAPI_NRACH_INDICATION,

	/* Сообщение дял передачи дополнительных флагов и параметров */
	FAPI_P8_REQUEST = 0xA0,
	FAPI_P8_INDICATION,

	FAPI_MAX_MESSAGE_ID,
} fapi_message_id_e;

// nFAPI Error Codes
typedef enum {
	FAPI_MSG_OK = 0,
	FAPI_MSG_INVALID_STATE,
	FAPI_MSG_INVALID_CONFIG,
	FAPI_SFN_OUT_OF_SYNC,
	FAPI_MSG_SUBFRAME_ERR,
	FAPI_MSG_BCH_MISSING,
	FAPI_MSG_INVALID_SFN,
	FAPI_MSG_HI_ERR,
	FAPI_MSG_TX_ERR,
	
	FAPI_LBT_NO_PDU_IN_DL_REQ,
	FAPI_LBT_NO_VALID_CONFIG_REQ_RECEIVED,
	FAPI_FAPI_E_LBT_SF_SFN_PASSED_END_SF_SFN,
	FAPI_FAPI_E_LBT_OVERLAP,
	FAPI_MSG_BCH_PRESENT,
	
	FAPI_NBIOT_UNEXPECTED_REQ,

	// This is special return code that indicates that a response has
	// been send via P9
	FAPI_MSG_P9_RESPONSE = 0xAA
} fapi_error_code_e;


typedef enum {
	FAPI_P4_MSG_OK = 100,
	FAPI_P4_MSG_INVALID_STATE = 101,
	FAPI_P4_MSG_INVALID_CONFIG = 102,
	FAPI_P4_MSG_RAT_NOT_SUPPORTED = 103,
	FAPI_P4_MSG_NMM_STOP_OK = 200,
	FAPI_P4_MSG_NMM_STOP_IGNOREDED = 201,
	FAPI_P4_MSG_NMM_STOP_INVALID_STATE = 202,
	FAPI_P4_MSG_PROCEDURE_COMPLETE = 300,
	FAPI_P4_MSG_PROCEDURE_STOPPED = 301,
	FAPI_P4_MSG_PARTIAL_RESULTS = 302,
	FAPI_P4_MSG_TIMEOUT = 303
} fapi_p4_error_code_e;

// nFAPI enums
typedef enum {
	FAPI_DL_CONFIG_DCI_DL_PDU_TYPE = 0,
	FAPI_DL_CONFIG_BCH_PDU_TYPE,
	FAPI_DL_CONFIG_MCH_PDU_TYPE,
	FAPI_DL_CONFIG_DLSCH_PDU_TYPE,
	FAPI_DL_CONFIG_PCH_PDU_TYPE,
	FAPI_DL_CONFIG_PRS_PDU_TYPE,
	FAPI_DL_CONFIG_CSI_RS_PDU_TYPE,
	FAPI_DL_CONFIG_EPDCCH_DL_PDU_TYPE,
	FAPI_DL_CONFIG_MPDCCH_PDU_TYPE,
	FAPI_DL_CONFIG_NBCH_PDU_TYPE,
	FAPI_DL_CONFIG_NPDCCH_PDU_TYPE,
	FAPI_DL_CONFIG_NDLSCH_PDU_TYPE
} fapi_dl_config_pdu_type_e;

typedef enum {
	FAPI_DL_DCI_FORMAT_1 = 0,
	FAPI_DL_DCI_FORMAT_1A,
	FAPI_DL_DCI_FORMAT_1B,
	FAPI_DL_DCI_FORMAT_1C,
	FAPI_DL_DCI_FORMAT_1D,
	FAPI_DL_DCI_FORMAT_2,
	FAPI_DL_DCI_FORMAT_2A,
	FAPI_DL_DCI_FORMAT_2B,
	FAPI_DL_DCI_FORMAT_2C
} fapi_dl_dci_format_e;

typedef enum {
	FAPI_UL_DCI_FORMAT_0 = 0,
	FAPI_UL_DCI_FORMAT_3,
	FAPI_UL_DCI_FORMAT_3A,
	FAPI_UL_DCI_FORMAT_4
} fapi_ul_dci_format_e;

typedef enum {
	FAPI_UL_CONFIG_ULSCH_PDU_TYPE = 0,
	FAPI_UL_CONFIG_ULSCH_CQI_RI_PDU_TYPE,
	FAPI_UL_CONFIG_ULSCH_HARQ_PDU_TYPE,
	FAPI_UL_CONFIG_ULSCH_CQI_HARQ_RI_PDU_TYPE,
	FAPI_UL_CONFIG_UCI_CQI_PDU_TYPE,
	FAPI_UL_CONFIG_UCI_SR_PDU_TYPE,
	FAPI_UL_CONFIG_UCI_HARQ_PDU_TYPE,
	FAPI_UL_CONFIG_UCI_SR_HARQ_PDU_TYPE,
	FAPI_UL_CONFIG_UCI_CQI_HARQ_PDU_TYPE,
	FAPI_UL_CONFIG_UCI_CQI_SR_PDU_TYPE,
	FAPI_UL_CONFIG_UCI_CQI_SR_HARQ_PDU_TYPE,
	FAPI_UL_CONFIG_SRS_PDU_TYPE,
	FAPI_UL_CONFIG_HARQ_BUFFER_PDU_TYPE,
	FAPI_UL_CONFIG_ULSCH_UCI_CSI_PDU_TYPE,
	FAPI_UL_CONFIG_ULSCH_UCI_HARQ_PDU_TYPE,
	FAPI_UL_CONFIG_ULSCH_CSI_UCI_HARQ_PDU_TYPE,
	FAPI_UL_CONFIG_NULSCH_PDU_TYPE,
	FAPI_UL_CONFIG_NRACH_PDU_TYPE,
} fapi_ul_config_pdu_type_e;

typedef enum {
	FAPI_HI_DCI0_HI_PDU_TYPE = 0,
	FAPI_HI_DCI0_DCI_PDU_TYPE,
	FAPI_HI_DCI0_EPDCCH_DCI_PDU_TYPE,
	FAPI_HI_DCI0_MPDCCH_DCI_PDU_TYPE,
	FAPI_HI_DCI0_NPDCCH_DCI_PDU_TYPE,
} fapi_hi_dci0_pdu_type_e;

typedef enum {
	FAPI_HARQ_ACK = 1,
	FAPI_HARQ_NACK,
	FAPI_HARQ_ACK_OR_NACK,
	FAPI_HARQ_DTX,
	FAPI_HARQ_ACK_OR_DTX,
	FAPI_HARQ_NACK_OR_DTX,
	FAPI_HARQ_ACK_OR_NACK_OR_DTX
} fapi_harq_type_e;

typedef enum {
	FAPI_CSI_REPORT_TYPE_PERIODIC = 0,
	FAPI_CSI_REPORT_TYPE_APERIODIC
} fapi_csi_report_type_e;

typedef enum {
	FAPI_DL_BW_SUPPORTED_6 = 1,
	FAPI_DL_BW_SUPPORTED_15 = 2,
	FAPI_DL_BW_SUPPORTED_25 = 4,
	FAPI_DL_BW_SUPPORTED_50 = 8,
	FAPI_DL_BW_SUPPORTED_75 = 16,
	FAPI_DL_BW_SUPPORTED_100 = 32
} fapi_dl_bandwith_supported_e;

typedef enum {
	FAPI_UL_BW_SUPPORTED_6 = 1,
	FAPI_UL_BW_SUPPORTED_15 = 2,
	FAPI_UL_BW_SUPPORTED_25 = 4,
	FAPI_UL_BW_SUPPORTED_50 = 8,
	FAPI_UL_BW_SUPPORTED_75 = 16,
	FAPI_UL_BW_SUPPORTED_100 = 32
} fapi_ul_bandwith_supported_e;

typedef enum {
	FAPI_3GPP_REL_SUPPORTED_8 = 0,
	FAPI_3GPP_REL_SUPPORTED_9 = 1,
	FAPI_3GPP_REL_SUPPORTED_10 = 2,
	FAPI_3GPP_REL_SUPPORTED_11 = 4,
	FAPI_3GPP_REL_SUPPORTED_12 = 8
} fapi_3gpp_release_supported_e;


typedef enum {
	FAPI_DUPLEXING_MODE_TDD = 0,
	FAPI_DUPLEXING_MODE_FDD = 1,
	FAPI_DUPLEXING_MODE_HD_FDD = 2,
} fapi_duplexing_mode_e;

typedef enum {
	FAPI_CP_NORMAL = 0,
	FAPI_CP_EXTENDED = 1
} fapi_cyclic_prefix_type_e;

typedef enum {
	FAPI_RAT_TYPE_LTE = 0,
	FAPI_RAT_TYPE_UTRAN = 1,
	FAPI_RAT_TYPE_GERAN = 2,
	FAPI_RAT_TYPE_NB_IOT = 3
} fapi_rat_type_e;

typedef enum {
	FAPI_HARQ_INDICATION_TDD_HARQ_ACK_NACK_FORMAT_BUNDLING,
	FAPI_HARQ_INDICATION_TDD_HARQ_ACK_NACK_FORMAT_MULIPLEXING,
	FAPI_HARQ_INDICATION_TDD_HARQ_ACK_NACK_FORMAT_SPECIAL_BUNDLING,
	FAPI_HARQ_INDICATION_TDD_HARQ_ACK_NACK_FORMAT_CHANNEL_SELECTION,
	FAPI_HARQ_INDICATION_TDD_HARQ_ACK_NACK_FORMAT_FORMAT_3,
	FAPI_HARQ_INDICATION_TDD_HARQ_ACK_NACK_FORMAT_FORMAT_4,
	FAPI_HARQ_INDICATION_TDD_HARQ_ACK_NACK_FORMAT_FORMAT_5
} fapi_harq_indication_tdd_ack_nackformat_e;


typedef enum {
	FAPI_LBT_DL_CONFIG_REQUEST_PDSCH_PDU_TYPE = 0,
	FAPI_LBT_DL_CONFIG_REQUEST_DRS_PDU_TYPE
} fapi_lbt_dl_config_pdu_type_e;

typedef enum {
	FAPI_LBT_DL_RSP_PDSCH_PDU_TYPE = 0,
	FAPI_LBT_DL_RSP_DRS_PDU_TYPE
} fapi_lbt_dl_rsp_pdu_type_e;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t length;
	uint32_t value[FAPI_MAX_OPAQUE_DATA];
} fapi_opaqaue_data_t;

// Utility functions to turn enums into char*
const char* fapi_error_code_to_str(fapi_error_code_e value);


// P5 Sub Structures
typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t fapi_sync_mode;
	uint32_t location_mode;
	uint32_t location_coordinates_length;
	uint32_t location_coordinates[FAPI_PNF_PARAM_GENERAL_LOCATION_LENGTH];
	uint32_t dl_config_timing;
	uint32_t tx_timing;
	uint32_t ul_config_timing;
	uint32_t hi_dci0_timing;
	uint32_t maximum_number_phys;
	uint32_t maximum_total_bandwidth;
	uint32_t maximum_total_number_dl_layers;
	uint32_t maximum_total_number_ul_layers;
	uint32_t shared_bands;
	uint32_t shared_pa;
	int32_t maximum_total_power;
	uint32_t oui[FAPI_PNF_PARAM_GENERAL_OUI_LENGTH];
} fapi_pnf_param_general_t;
#define FAPI_PNF_PARAM_GENERAL_TAG 0x1000





typedef struct __attribute__((packed)) {
	uint32_t rf_config_index;
} fapi_rf_config_info_t;

typedef struct __attribute__((packed)) {
	uint32_t phy_config_index;
	uint32_t number_of_rfs;
	fapi_rf_config_info_t rf_config[FAPI_MAX_PNF_PHY_RF_CONFIG];
	uint32_t number_of_rf_exclusions;
	fapi_rf_config_info_t excluded_rf_config[FAPI_MAX_PNF_PHY_RF_CONFIG];
	uint32_t downlink_channel_bandwidth_supported;
	uint32_t uplink_channel_bandwidth_supported;
	uint32_t number_of_dl_layers_supported;
	uint32_t number_of_ul_layers_supported;
	uint32_t maximum_3gpp_release_supported;
	uint32_t nmm_modes_supported;
} fapi_pnf_phy_info_t;


typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_phys;
	fapi_pnf_phy_info_t phy[FAPI_MAX_PNF_PHY];
} fapi_pnf_phy_t;
#define FAPI_PNF_PHY_TAG 0x1001

typedef struct __attribute__((packed)) {
	uint32_t phy_config_index;
	uint32_t transmission_mode_7_supported;
	uint32_t transmission_mode_8_supported;
	uint32_t two_antenna_ports_for_pucch;
	uint32_t transmission_mode_9_supported;
	uint32_t simultaneous_pucch_pusch;
	uint32_t four_layer_tx_with_tm3_and_tm4;
} fapi_pnf_phy_rel10_info_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_phys;
	fapi_pnf_phy_rel10_info_t phy[FAPI_MAX_PNF_PHY];
} fapi_pnf_phy_rel10_t;
#define FAPI_PNF_PHY_REL10_TAG 0x100A

typedef struct __attribute__((packed)) {
	uint32_t phy_config_index;
	uint32_t edpcch_supported;
	uint32_t multi_ack_csi_reporting;
	uint32_t pucch_tx_diversity;
	uint32_t ul_comp_supported;
	uint32_t transmission_mode_5_supported;
} fapi_pnf_phy_rel11_info_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_phys;
	fapi_pnf_phy_rel11_info_t phy[FAPI_MAX_PNF_PHY];
} fapi_pnf_phy_rel11_t;
#define FAPI_PNF_PHY_REL11_TAG 0x100B


typedef struct __attribute__((packed)) {
	uint32_t phy_config_index;
	uint32_t csi_subframe_set;
	uint32_t enhanced_4tx_codebook;
	uint32_t drs_supported;
	uint32_t ul_64qam_supported;
	uint32_t transmission_mode_10_supported;
	uint32_t alternative_bts_indices;
} fapi_pnf_phy_rel12_info_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_phys;
	fapi_pnf_phy_rel12_info_t phy[FAPI_MAX_PNF_PHY];
} fapi_pnf_phy_rel12_t;
#define FAPI_PNF_PHY_REL12_TAG 0x100C

typedef struct __attribute__((packed)) {
	uint32_t phy_config_index;
	uint32_t pucch_format4_supported;
	uint32_t pucch_format5_supported;
	uint32_t more_than_5_ca_support;
	uint32_t laa_supported;
	uint32_t laa_ending_in_dwpts_supported;
	uint32_t laa_starting_in_second_slot_supported;
	uint32_t beamforming_supported;
	uint32_t csi_rs_enhancement_supported;
	uint32_t drms_enhancement_supported;
	uint32_t srs_enhancement_supported;
} fapi_pnf_phy_rel13_info_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_phys;
	fapi_pnf_phy_rel13_info_t phy[FAPI_MAX_PNF_PHY];
} fapi_pnf_phy_rel13_t;
#define FAPI_PNF_PHY_REL13_TAG 0x100D

typedef struct __attribute__((packed)) {
	uint32_t phy_config_index;
	uint32_t number_of_rfs;
	fapi_rf_config_info_t rf_config[FAPI_MAX_PNF_PHY_RF_CONFIG];
	uint32_t number_of_rf_exclusions;
	fapi_rf_config_info_t excluded_rf_config[FAPI_MAX_PNF_PHY_RF_CONFIG];
	uint32_t number_of_dl_layers_supported;
	uint32_t number_of_ul_layers_supported;
	uint32_t maximum_3gpp_release_supported;
	uint32_t nmm_modes_supported;
} fapi_pnf_phy_rel13_nb_iot_info_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_phys;
	fapi_pnf_phy_rel13_nb_iot_info_t phy[FAPI_MAX_PNF_PHY];
} fapi_pnf_phy_rel13_nb_iot_t;
#define FAPI_PNF_PHY_REL13_NB_IOT_TAG 0x100E



typedef struct __attribute__((packed)) {
	uint32_t rf_config_index;
	uint32_t band;
	int32_t maximum_transmit_power; 
	int32_t minimum_transmit_power;
	uint32_t number_of_antennas_suppported;
	uint32_t minimum_downlink_frequency;
	uint32_t maximum_downlink_frequency;
	uint32_t minimum_uplink_frequency;
	uint32_t maximum_uplink_frequency;
} fapi_pnf_rf_info_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_rfs;
	fapi_pnf_rf_info_t rf[FAPI_MAX_PNF_RF];
} fapi_pnf_rf_t;
#define FAPI_PNF_RF_TAG 0x1002

typedef struct __attribute__((packed)) {
	uint32_t phy_id;
	uint32_t phy_config_index;
	uint32_t rf_config_index;
} fapi_phy_rf_config_info_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_phy_rf_config_info;
	fapi_phy_rf_config_info_t phy_rf_config[FAPI_MAX_PHY_RF_INSTANCES];
} fapi_pnf_phy_rf_config_t;
#define FAPI_PNF_PHY_RF_TAG 0x1003

// Generic strucutre for single tlv value.
typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t value;
} fapi_uint32_tlv_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	int32_t value;
} fapi_int16_tlv_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t value;
} fapi_uint8_tlv_t;

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t phy_state;
} fapi_l1_status;

#define FAPI_L1_STATUS_PHY_STATE_TAG 0x00FA

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t dl_bandwidth_support;
	fapi_uint32_tlv_t ul_bandwidth_support;
	fapi_uint32_tlv_t dl_modulation_support;
	fapi_uint32_tlv_t ul_modulation_support;
	fapi_uint32_tlv_t phy_antenna_capability;
	fapi_uint32_tlv_t release_capability;
	fapi_uint32_tlv_t mbsfn_capability;
} fapi_phy_capabilities_t;

#define FAPI_PHY_CAPABILITIES_DL_BANDWIDTH_SUPPORT_TAG 0x00C8
#define FAPI_PHY_CAPABILITIES_UL_BANDWIDTH_SUPPORT_TAG 0x00C9
#define FAPI_PHY_CAPABILITIES_DL_MODULATION_SUPPORT_TAG 0x00CA
#define FAPI_PHY_CAPABILITIES_UL_MODULATION_SUPPORT_TAG 0x00CB
#define FAPI_PHY_CAPABILITIES_PHY_ANTENNA_CAPABILITY_TAG 0x00CC
#define FAPI_PHY_CAPABILITIES_RELEASE_CAPABILITY_TAG 0x00CD
#define FAPI_PHY_CAPABILITIES_MBSFN_CAPABILITY_TAG 0x00CE


typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t data_report_mode;
	fapi_uint32_tlv_t sfnsf;
} fapi_l23_config_t;


#define FAPI_L23_CONFIG_DATA_REPORT_MODE_TAG 0x00F0
#define FAPI_L23_CONFIG_SFNSF_TAG 0x00F1

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t duplex_mode;
	fapi_uint32_tlv_t pcfich_power_offset;
	fapi_uint32_tlv_t pb;
	fapi_uint32_tlv_t dl_cyclic_prefix_type;
	fapi_uint32_tlv_t ul_cyclic_prefix_type;
} fapi_subframe_config_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t duplex_mode;
	uint32_t pcfich_power_offset;
	uint32_t pb;
	uint32_t dl_cyclic_prefix_type;
	uint32_t ul_cyclic_prefix_type;
} fapi_subframe_config_t;

#define FAPI_SUBFRAME_CONFIG_DUPLEX_MODE_TAG 0x0001
#define FAPI_SUBFRAME_CONFIG_PCFICH_POWER_OFFSET_TAG 0x0002
#define FAPI_SUBFRAME_CONFIG_PB_TAG 0x0003
#define FAPI_SUBFRAME_CONFIG_DL_CYCLIC_PREFIX_TYPE_TAG 0x0004
#define FAPI_SUBFRAME_CONFIG_UL_CYCLIC_PREFIX_TYPE_TAG 0x0005

typedef struct __attribute__((packed)) {
	uint32_t dl_channel_bandwidth;
	uint32_t ul_channel_bandwidth;
	uint32_t reference_signal_power;
	uint32_t tx_antenna_ports;
	uint32_t rx_antenna_ports;
} fapi_rf_config_t;

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t dl_channel_bandwidth;
	fapi_uint32_tlv_t ul_channel_bandwidth;
	fapi_uint32_tlv_t reference_signal_power;
	fapi_uint32_tlv_t tx_antenna_ports;
	fapi_uint32_tlv_t rx_antenna_ports;
} fapi_rf_config_tlv_t;

#define FAPI_RF_CONFIG_DL_CHANNEL_BANDWIDTH_TAG 0x000A
#define FAPI_RF_CONFIG_UL_CHANNEL_BANDWIDTH_TAG 0x000B
#define FAPI_RF_CONFIG_REFERENCE_SIGNAL_POWER_TAG 0x000C
#define FAPI_RF_CONFIG_TX_ANTENNA_PORTS_TAG 0x000D
#define FAPI_RF_CONFIG_RX_ANTENNA_PORTS_TAG 0x000E

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t phich_resource;
	fapi_uint32_tlv_t phich_duration;
	fapi_uint32_tlv_t phich_power_offset;
} fapi_phich_config_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t phich_resource;
	uint32_t phich_duration;
	uint32_t phich_power_offset;
} fapi_phich_config_t;

#define FAPI_PHICH_CONFIG_PHICH_RESOURCE_TAG 0x0014
#define FAPI_PHICH_CONFIG_PHICH_DURATION_TAG 0x0015
#define FAPI_PHICH_CONFIG_PHICH_POWER_OFFSET_TAG 0x0016

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t primary_synchronization_signal_epre_eprers;
	fapi_uint32_tlv_t secondary_synchronization_signal_epre_eprers;
	fapi_uint32_tlv_t physical_cell_id;
} fapi_sch_config_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t primary_synchronization_signal_epre_eprers;
	uint32_t secondary_synchronization_signal_epre_eprers;
	uint32_t physical_cell_id;
} fapi_sch_config_t;

#define FAPI_SCH_CONFIG_PRIMARY_SYNCHRONIZATION_SIGNAL_EPRE_EPRERS_TAG 0x001E
#define FAPI_SCH_CONFIG_SECONDARY_SYNCHRONIZATION_SIGNAL_EPRE_EPRERS_TAG 0x001F
#define FAPI_SCH_CONFIG_PHYSICAL_CELL_ID_TAG 0x0020

typedef struct __attribute__((packed)) {
	uint32_t configuration_index;
	uint32_t root_sequence_index;
	uint32_t zero_correlation_zone_configuration;
	uint32_t high_speed_flag;
	uint32_t frequency_offset;
} fapi_prach_config_t;

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t configuration_index;
	fapi_uint32_tlv_t root_sequence_index;
	fapi_uint32_tlv_t zero_correlation_zone_configuration;
	fapi_uint32_tlv_t high_speed_flag;
	fapi_uint32_tlv_t frequency_offset;
} fapi_prach_config_tlv_t;

#define FAPI_PRACH_CONFIG_CONFIGURATION_INDEX_TAG 0x0028
#define FAPI_PRACH_CONFIG_ROOT_SEQUENCE_INDEX_TAG 0x0029
#define FAPI_PRACH_CONFIG_ZERO_CORRELATION_ZONE_CONFIGURATION_TAG 0x002A
#define FAPI_PRACH_CONFIG_HIGH_SPEED_FLAG_TAG 0x002B
#define FAPI_PRACH_CONFIG_FREQUENCY_OFFSET_TAG 0x002C

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t hopping_mode;
	fapi_uint32_tlv_t hopping_offset;
	fapi_uint32_tlv_t number_of_subbands;
} fapi_pusch_config_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t hopping_mode;
	uint32_t hopping_offset;
	uint32_t number_of_subbands;
} fapi_pusch_config_t;

#define FAPI_PUSCH_CONFIG_HOPPING_MODE_TAG 0x0032
#define FAPI_PUSCH_CONFIG_HOPPING_OFFSET_TAG 0x0033
#define FAPI_PUSCH_CONFIG_NUMBER_OF_SUBBANDS_TAG 0x0034

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t delta_pucch_shift;
	fapi_uint32_tlv_t n_cqi_rb;
	fapi_uint32_tlv_t n_an_cs;
	fapi_uint32_tlv_t n1_pucch_an;
} fapi_pucch_config_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t delta_pucch_shift;
	uint32_t n_cqi_rb;
	uint32_t n_an_cs;
	uint32_t n1_pucch_an;
} fapi_pucch_config_t;

#define FAPI_PUCCH_CONFIG_DELTA_PUCCH_SHIFT_TAG 0x003C
#define FAPI_PUCCH_CONFIG_N_CQI_RB_TAG 0x003D
#define FAPI_PUCCH_CONFIG_N_AN_CS_TAG 0x003E
#define FAPI_PUCCH_CONFIG_N1_PUCCH_AN_TAG 0x003F

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t bandwidth_configuration;
	fapi_uint32_tlv_t max_up_pts;
	fapi_uint32_tlv_t srs_subframe_configuration;
	fapi_uint32_tlv_t srs_acknack_srs_simultaneous_transmission;
} fapi_srs_config_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t bandwidth_configuration;
	uint32_t max_up_pts;
	uint32_t srs_subframe_configuration;
	uint32_t srs_acknack_srs_simultaneous_transmission;
} fapi_srs_config_t;

#define FAPI_SRS_CONFIG_BANDWIDTH_CONFIGURATION_TAG 0x0046
#define FAPI_SRS_CONFIG_MAX_UP_PTS_TAG 0x0047
#define FAPI_SRS_CONFIG_SRS_SUBFRAME_CONFIGURATION_TAG 0x0048
#define FAPI_SRS_CONFIG_SRS_ACKNACK_SRS_SIMULTANEOUS_TRANSMISSION_TAG 0x0049

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t uplink_rs_hopping;
	fapi_uint32_tlv_t group_assignment;
	fapi_uint32_tlv_t cyclic_shift_1_for_drms;
} fapi_uplink_reference_signal_config_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t uplink_rs_hopping;
	uint32_t group_assignment;
	uint32_t cyclic_shift_1_for_drms;
} fapi_uplink_reference_signal_config_t;

#define FAPI_UPLINK_REFERENCE_SIGNAL_CONFIG_UPLINK_RS_HOPPING_TAG 0x0050
#define FAPI_UPLINK_REFERENCE_SIGNAL_CONFIG_GROUP_ASSIGNMENT_TAG 0x0051
#define FAPI_UPLINK_REFERENCE_SIGNAL_CONFIG_CYCLIC_SHIFT_1_FOR_DRMS_TAG 0x0052


typedef struct __attribute__((packed)) {
	uint32_t ed_threshold_lbt_pdsch;
	uint32_t ed_threshold_lbt_drs;
	uint32_t pd_threshold;
	uint32_t multi_carrier_type;
	uint32_t multi_carrier_tx;
	uint32_t multi_carrier_freeze;
	uint32_t tx_antenna_ports_drs;
	uint32_t tx_power_drs;
} fapi_laa_config_t;

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t ed_threshold_lbt_pdsch;
	fapi_uint32_tlv_t ed_threshold_lbt_drs;
	fapi_uint32_tlv_t pd_threshold;
	fapi_uint32_tlv_t multi_carrier_type;
	fapi_uint32_tlv_t multi_carrier_tx;
	fapi_uint32_tlv_t multi_carrier_freeze;
	fapi_uint32_tlv_t tx_antenna_ports_drs;
	fapi_uint32_tlv_t tx_power_drs;
} fapi_laa_config_tlv_t;

#define FAPI_LAA_CONFIG_ED_THRESHOLD_FOR_LBT_FOR_PDSCH_TAG 0x0064
#define FAPI_LAA_CONFIG_ED_THRESHOLD_FOR_LBT_FOR_DRS_TAG 0x0065
#define FAPI_LAA_CONFIG_PD_THRESHOLD_TAG 0x0066
#define FAPI_LAA_CONFIG_MULTI_CARRIER_TYPE_TAG 0x0067
#define FAPI_LAA_CONFIG_MULTI_CARRIER_TX_TAG 0x0068
#define FAPI_LAA_CONFIG_MULTI_CARRIER_FREEZE_TAG 0x0069
#define FAPI_LAA_CONFIG_TX_ANTENNA_PORTS_FOR_DRS_TAG 0x006A
#define FAPI_LAA_CONFIG_TRANSMISSION_POWER_FOR_DRS_TAG 0x006B

typedef struct __attribute__((packed)) {

	fapi_uint32_tlv_t pbch_repetitions_enable_r13;
	fapi_uint32_tlv_t prach_catm_root_sequence_index;
	fapi_uint32_tlv_t prach_catm_zero_correlation_zone_configuration;
	fapi_uint32_tlv_t prach_catm_high_speed_flag;
	fapi_uint32_tlv_t prach_ce_level_0_enable;
	fapi_uint32_tlv_t prach_ce_level_0_configuration_index;
	fapi_uint32_tlv_t prach_ce_level_0_frequency_offset;
	fapi_uint32_tlv_t prach_ce_level_0_number_of_repetitions_per_attempt;
	fapi_uint32_tlv_t prach_ce_level_0_starting_subframe_periodicity;
	fapi_uint32_tlv_t prach_ce_level_0_hopping_enable;
	fapi_uint32_tlv_t prach_ce_level_0_hopping_offset;
	fapi_uint32_tlv_t prach_ce_level_1_enable;
	fapi_uint32_tlv_t prach_ce_level_1_configuration_index;
	fapi_uint32_tlv_t prach_ce_level_1_frequency_offset;
	fapi_uint32_tlv_t prach_ce_level_1_number_of_repetitions_per_attempt;
	fapi_uint32_tlv_t prach_ce_level_1_starting_subframe_periodicity;
	fapi_uint32_tlv_t prach_ce_level_1_hopping_enable;
	fapi_uint32_tlv_t prach_ce_level_1_hopping_offset;
	fapi_uint32_tlv_t prach_ce_level_2_enable;
	fapi_uint32_tlv_t prach_ce_level_2_configuration_index;
	fapi_uint32_tlv_t prach_ce_level_2_frequency_offset;
	fapi_uint32_tlv_t prach_ce_level_2_number_of_repetitions_per_attempt;
	fapi_uint32_tlv_t prach_ce_level_2_starting_subframe_periodicity;
	fapi_uint32_tlv_t prach_ce_level_2_hopping_enable;
	fapi_uint32_tlv_t prach_ce_level_2_hopping_offset;
	fapi_uint32_tlv_t prach_ce_level_3_enable;
	fapi_uint32_tlv_t prach_ce_level_3_configuration_index;
	fapi_uint32_tlv_t prach_ce_level_3_frequency_offset;
	fapi_uint32_tlv_t prach_ce_level_3_number_of_repetitions_per_attempt;
	fapi_uint32_tlv_t prach_ce_level_3_starting_subframe_periodicity;
	fapi_uint32_tlv_t prach_ce_level_3_hopping_enable;
	fapi_uint32_tlv_t prach_ce_level_3_hopping_offset;
	fapi_uint32_tlv_t pucch_interval_ulhoppingconfigcommonmodea;
	fapi_uint32_tlv_t pucch_interval_ulhoppingconfigcommonmodeb;
} fapi_emtc_config_tlv_t;

typedef struct __attribute__((packed)) {

	uint32_t pbch_repetitions_enable_r13;
	uint32_t prach_catm_root_sequence_index;
	uint32_t prach_catm_zero_correlation_zone_configuration;
	uint32_t prach_catm_high_speed_flag;
	uint32_t prach_ce_level_0_enable;
	uint32_t prach_ce_level_0_configuration_index;
	uint32_t prach_ce_level_0_frequency_offset;
	uint32_t prach_ce_level_0_number_of_repetitions_per_attempt;
	uint32_t prach_ce_level_0_starting_subframe_periodicity;
	uint32_t prach_ce_level_0_hopping_enable;
	uint32_t prach_ce_level_0_hopping_offset;
	uint32_t prach_ce_level_1_enable;
	uint32_t prach_ce_level_1_configuration_index;
	uint32_t prach_ce_level_1_frequency_offset;
	uint32_t prach_ce_level_1_number_of_repetitions_per_attempt;
	uint32_t prach_ce_level_1_starting_subframe_periodicity;
	uint32_t prach_ce_level_1_hopping_enable;
	uint32_t prach_ce_level_1_hopping_offset;
	uint32_t prach_ce_level_2_enable;
	uint32_t prach_ce_level_2_configuration_index;
	uint32_t prach_ce_level_2_frequency_offset;
	uint32_t prach_ce_level_2_number_of_repetitions_per_attempt;
	uint32_t prach_ce_level_2_starting_subframe_periodicity;
	uint32_t prach_ce_level_2_hopping_enable;
	uint32_t prach_ce_level_2_hopping_offset;
	uint32_t prach_ce_level_3_enable;
	uint32_t prach_ce_level_3_configuration_index;
	uint32_t prach_ce_level_3_frequency_offset;
	uint32_t prach_ce_level_3_number_of_repetitions_per_attempt;
	uint32_t prach_ce_level_3_starting_subframe_periodicity;
	uint32_t prach_ce_level_3_hopping_enable;
	uint32_t prach_ce_level_3_hopping_offset;
	uint32_t pucch_interval_ulhoppingconfigcommonmodea;
	uint32_t pucch_interval_ulhoppingconfigcommonmodeb;
} fapi_emtc_config_t;

#define FAPI_EMTC_CONFIG_PBCH_REPETITIONS_ENABLE_R13_TAG 0x0078
#define FAPI_EMTC_CONFIG_PRACH_CATM_ROOT_SEQUENCE_INDEX_TAG 0x0079
#define FAPI_EMTC_CONFIG_PRACH_CATM_ZERO_CORRELATION_ZONE_CONFIGURATION_TAG 0x007A
#define FAPI_EMTC_CONFIG_PRACH_CATM_HIGH_SPEED_FLAG 0x007B
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_0_ENABLE_TAG 0x007C
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_0_CONFIGURATION_INDEX_TAG 0x007D
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_0_FREQUENCY_OFFSET_TAG 0x007E
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_0_NUMBER_OF_REPETITIONS_PER_ATTEMPT_TAG 0x007F
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_0_STARTING_SUBFRAME_PERIODICITY_TAG 0x0080
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_0_HOPPING_ENABLE_TAG 0x0081
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_0_HOPPING_OFFSET_TAG 0x0082
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_1_ENABLE_TAG 0x0083
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_1_CONFIGURATION_INDEX_TAG 0x0084
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_1_FREQUENCY_OFFSET_TAG 0x0085
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_1_NUMBER_OF_REPETITIONS_PER_ATTEMPT_TAG 0x0086
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_1_STARTING_SUBFRAME_PERIODICITY_TAG 0x0087
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_1_HOPPING_ENABLE_TAG 0x0088
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_1_HOPPING_OFFSET_TAG 0x0089
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_2_ENABLE_TAG 0x008A
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_2_CONFIGURATION_INDEX_TAG 0x008B
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_2_FREQUENCY_OFFSET_TAG 0x008C
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_2_NUMBER_OF_REPETITIONS_PER_ATTEMPT_TAG 0x008D
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_2_STARTING_SUBFRAME_PERIODICITY_TAG 0x008E
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_2_HOPPING_ENABLE_TAG 0x008F
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_2_HOPPING_OFFSET_TAG 0x0090
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_3_ENABLE_TAG 0x0091
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_3_CONFIGURATION_INDEX_TAG 0x0092
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_3_FREQUENCY_OFFSET_TAG 0x0093
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_3_NUMBER_OF_REPETITIONS_PER_ATTEMPT_TAG 0x0094
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_3_STARTING_SUBFRAME_PERIODICITY_TAG 0x0095
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_3_HOPPING_ENABLE_TAG 0x0096
#define FAPI_EMTC_CONFIG_PRACH_CE_LEVEL_3_HOPPING_OFFSET_TAG 0x0097
#define FAPI_EMTC_CONFIG_PUCCH_INTERVAL_ULHOPPINGCONFIGCOMMONMODEA_TAG 0x0098
#define FAPI_EMTC_CONFIG_PUCCH_INTERVAL_ULHOPPINGCONFIGCOMMONMODEB_TAG 0x0099

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t operating_mode;
	fapi_uint32_tlv_t anchor;
	fapi_uint32_tlv_t prb_index;
	fapi_uint32_tlv_t control_region_size;
	fapi_uint32_tlv_t assumed_crs_aps;
	fapi_uint32_tlv_t nprach_config_0_enabled;
	fapi_uint32_tlv_t nprach_config_0_sf_periodicity;
	fapi_uint32_tlv_t nprach_config_0_start_time;
	fapi_uint32_tlv_t nprach_config_0_subcarrier_offset;
	fapi_uint32_tlv_t nprach_config_0_number_of_subcarriers;
	fapi_uint32_tlv_t nprach_config_0_cp_length;
	fapi_uint32_tlv_t nprach_config_0_number_of_repetitions_per_attempt;
	fapi_uint32_tlv_t nprach_config_1_enabled;
	fapi_uint32_tlv_t nprach_config_1_sf_periodicity;
	fapi_uint32_tlv_t nprach_config_1_start_time;
	fapi_uint32_tlv_t nprach_config_1_subcarrier_offset;
	fapi_uint32_tlv_t nprach_config_1_number_of_subcarriers;
	fapi_uint32_tlv_t nprach_config_1_cp_length;
	fapi_uint32_tlv_t nprach_config_1_number_of_repetitions_per_attempt;
	fapi_uint32_tlv_t nprach_config_2_enabled;
	fapi_uint32_tlv_t nprach_config_2_sf_periodicity;
	fapi_uint32_tlv_t nprach_config_2_start_time;
	fapi_uint32_tlv_t nprach_config_2_subcarrier_offset;
	fapi_uint32_tlv_t nprach_config_2_number_of_subcarriers;
	fapi_uint32_tlv_t nprach_config_2_cp_length;
	fapi_uint32_tlv_t nprach_config_2_number_of_repetitions_per_attempt;
	fapi_uint32_tlv_t three_tone_base_sequence;
	fapi_uint32_tlv_t six_tone_base_sequence;
	fapi_uint32_tlv_t twelve_tone_base_sequence;
	fapi_uint32_tlv_t three_tone_cyclic_shift;
	fapi_uint32_tlv_t six_tone_cyclic_shift;
	fapi_uint32_tlv_t dl_gap_config_enable;
	fapi_uint32_tlv_t dl_gap_threshold;
	fapi_uint32_tlv_t dl_gap_periodicity;
	fapi_uint32_tlv_t dl_gap_duration_coefficient;
} fapi_nb_iot_config_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t operating_mode;
	uint32_t anchor;
	uint32_t prb_index;
	uint32_t control_region_size;
	uint32_t assumed_crs_aps;
	uint32_t nprach_config_0_enabled;
	uint32_t nprach_config_0_sf_periodicity;
	uint32_t nprach_config_0_start_time;
	uint32_t nprach_config_0_subcarrier_offset;
	uint32_t nprach_config_0_number_of_subcarriers;
	uint32_t nprach_config_0_cp_length;
	uint32_t nprach_config_0_number_of_repetitions_per_attempt;
	uint32_t nprach_config_0_npdcch_num_repetitions_ra;
	uint32_t nprach_config_0_npdcch_start_sf_css_ra;
	uint32_t nprach_config_0_npdcch_offset_ra;
	uint32_t nprach_config_1_enabled;
	uint32_t nprach_config_1_sf_periodicity;
	uint32_t nprach_config_1_start_time;
	uint32_t nprach_config_1_subcarrier_offset;
	uint32_t nprach_config_1_number_of_subcarriers;
	uint32_t nprach_config_1_cp_length;
	uint32_t nprach_config_1_number_of_repetitions_per_attempt;
	uint32_t nprach_config_1_npdcch_num_repetitions_ra;
	uint32_t nprach_config_1_npdcch_start_sf_css_ra;
	uint32_t nprach_config_1_npdcch_offset_ra;
	uint32_t nprach_config_2_enabled;
	uint32_t nprach_config_2_sf_periodicity;
	uint32_t nprach_config_2_start_time;
	uint32_t nprach_config_2_subcarrier_offset;
	uint32_t nprach_config_2_number_of_subcarriers;
	uint32_t nprach_config_2_cp_length;
	uint32_t nprach_config_2_number_of_repetitions_per_attempt;
	uint32_t nprach_config_2_npdcch_num_repetitions_ra;
	uint32_t nprach_config_2_npdcch_start_sf_css_ra;
	uint32_t nprach_config_2_npdcch_offset_ra;
	uint32_t three_tone_base_sequence;
	uint32_t six_tone_base_sequence;
	uint32_t twelve_tone_base_sequence;
	uint32_t three_tone_cyclic_shift;
	uint32_t six_tone_cyclic_shift;
	uint32_t dl_gap_config_enable;
	uint32_t dl_gap_threshold;
	uint32_t dl_gap_periodicity;
	uint32_t dl_gap_duration_coefficient;
} fapi_nb_iot_config_t;

#define FAPI_NB_IOT_CONFIG_OPERATING_MODE_TAG 0x00A5
#define FAPI_NB_IOT_CONFIG_ANCHOR_TAG 0x00A6
#define FAPI_NB_IOT_CONFIG_PRB_INDEX_TAG 0x00A7
#define FAPI_NB_IOT_CONFIG_CONTROL_REGION_SIZE_TAG 0x00A8
#define FAPI_NB_IOT_CONFIG_ASSUMED_CRS_APS_TAG 0x00A9
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_0_ENABLED_TAG 0x00AA
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_0_SF_PERIODICITY_TAG 0x00AB
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_0_START_TIME_TAG 0x00AC
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_0_SUBCARRIER_OFFSET_TAG 0x00AD
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_0_NUMBER_OF_SUBCARRIERS_TAG 0x00AE
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_0_CP_LENGTH_TAG 0x00AF
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_0_NUMBER_OF_REPETITIONS_PER_ATTEMPT_TAG 0x00B0
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_1_ENABLED_TAG 0x00B1
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_1_SF_PERIODICITY_TAG 0x00B2
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_1_START_TIME_TAG 0x00B3
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_1_SUBCARRIER_OFFSET_TAG 0x00B4
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_1_NUMBER_OF_SUBCARRIERS_TAG 0x00B5
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_1_CP_LENGTH_TAG 0x00B6
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_1_NUMBER_OF_REPETITIONS_PER_ATTEMPT_TAG 0x00B7
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_2_ENABLED_TAG 0x00B8
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_2_SF_PERIODICITY_TAG 0x00B9
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_2_START_TIME_TAG 0x00BA
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_2_SUBCARRIER_OFFSET_TAG 0x00BB
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_2_NUMBER_OF_SUBCARRIERS_TAG 0x00BC
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_2_CP_LENGTH_TAG 0x00BD
#define FAPI_NB_IOT_CONFIG_NPRACH_CONFIG_2_NUMBER_OF_REPETITIONS_PER_ATTEMPT_TAG 0x00BE
#define FAPI_NB_IOT_CONFIG_THREE_TONE_BASE_SEQUENCE_TAG 0x00BF
#define FAPI_NB_IOT_CONFIG_SIX_TONE_BASE_SEQUENCE_TAG 0x00C0
#define FAPI_NB_IOT_CONFIG_TWELVE_TONE_BASE_SEQUENCE_TAG 0x00C1
#define FAPI_NB_IOT_CONFIG_THREE_TONE_CYCLIC_SHIFT_TAG 0x00C2
#define FAPI_NB_IOT_CONFIG_SIX_TONE_CYCLIC_SHIFT_TAG 0x00C3
#define FAPI_NB_IOT_CONFIG_DL_GAP_CONFIG_ENABLE_TAG 0x00C4
#define FAPI_NB_IOT_CONFIG_DL_GAP_THRESHOLD_TAG 0x00C5
#define FAPI_NB_IOT_CONFIG_DL_GAP_PERIODICITY_TAG 0x00C6
#define FAPI_NB_IOT_CONFIG_DL_GAP_DURATION_COEFFICIENT_TAG 0x00C7

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t laa_support;
	fapi_uint32_tlv_t pd_sensing_lbt_support;
	fapi_uint32_tlv_t multi_carrier_lbt_support;
	fapi_uint32_tlv_t partial_sf_support;
} fapi_laa_capability_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t laa_support;
	uint32_t pd_sensing_lbt_support;
	uint32_t multi_carrier_lbt_support;
	uint32_t partial_sf_support;
} fapi_laa_capability_t;

#define FAPI_LAA_CAPABILITY_LAA_SUPPORT_TAG 0x00D1
#define FAPI_LAA_CAPABILITY_PD_SENSING_LBT_SUPPORT_TAG 0x00D2
#define FAPI_LAA_CAPABILITY_MULTI_CARRIER_LBT_SUPPORT_TAG 0x00D3
#define FAPI_LAA_CAPABILITY_PARTIAL_SF_SUPPORT_TAG 0x00D4

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t nb_iot_support;
	fapi_uint32_tlv_t nb_iot_operating_mode_capability;
} fapi_nb_iot_capability_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t nb_iot_support;
	uint32_t nb_iot_operating_mode_capability;
} fapi_nb_iot_capability_t;

#define FAPI_LAA_CAPABILITY_NB_IOT_SUPPORT_TAG 0x00D5
#define FAPI_LAA_CAPABILITY_NB_IOT_OPERATING_MODE_CAPABILITY_TAG 0x00D6

typedef struct __attribute__((packed)) {
	fapi_uint32_tlv_t subframe_assignment;
	fapi_uint32_tlv_t special_subframe_patterns;
} fapi_tdd_frame_structure_tlv_t;

typedef struct __attribute__((packed)) {
	uint32_t subframe_assignment;
	uint32_t special_subframe_patterns;
} fapi_tdd_frame_structure_t;

#define FAPI_TDD_FRAME_STRUCTURE_SUBFRAME_ASSIGNMENT_TAG 0x005A
#define FAPI_TDD_FRAME_STRUCTURE_SPECIAL_SUBFRAME_PATTERNS_TAG 0x005B

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_rf_bands;
	uint32_t rf_band[FAPI_MAX_NUM_RF_BANDS];
} fapi_rf_bands_t;
#define FAPI_PHY_RF_BANDS_TAG 0x0114

#define FAPI_IPV4_ADDRESS_LENGTH 4
#define FAPI_IPV6_ADDRESS_LENGTH 16

// Convience enum to allow the ip addres type to be distinguished
typedef enum {
	FAPI_IP_ADDRESS_IPV4 = 0,
	FAPI_IP_ADDRESS_IPV6
} fapi_ip_address_type_e;

// The type could be infered from the length, but it is clearer in 
// code to have a type variable set
typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t type;
	union {
		uint32_t ipv4_address[FAPI_IPV4_ADDRESS_LENGTH];
		uint32_t ipv6_address[FAPI_IPV6_ADDRESS_LENGTH];
	} u;
} fapi_ip_address_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t address[FAPI_IPV4_ADDRESS_LENGTH];
} fapi_ipv4_address_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t address[FAPI_IPV6_ADDRESS_LENGTH];
} fapi_ipv6_address_t;



typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_rf_bands;
	uint32_t bands[FAPI_MAX_NMM_FREQUENCY_BANDS];
} fapi_nmm_frequency_bands_t;

// P5 Message Structures
#if 0
typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_pnf_phy_rf_config_t pnf_phy_rf_config;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_pnf_config_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_pnf_config_response_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_pnf_start_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_pnf_start_response_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_pnf_stop_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_pnf_stop_response_t;
#endif

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_param_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	uint32_t num_tlv;
	// fdd or tdd in idle or configured tlvs
	fapi_l1_status l1_status;
	fapi_phy_capabilities_t phy_capabilities;
	fapi_laa_capability_t laa_capability;
	fapi_nb_iot_capability_t nb_iot_capability;
	
	fapi_subframe_config_t subframe_config;
	fapi_rf_config_t rf_config;
	fapi_phich_config_t phich_config;
	fapi_sch_config_t sch_config;
	fapi_prach_config_t prach_config;
	fapi_pusch_config_t pusch_config;
	fapi_pucch_config_t pucch_config;
	fapi_srs_config_t srs_config;
	fapi_uplink_reference_signal_config_t uplink_reference_signal_config;
	fapi_tdd_frame_structure_t tdd_frame_structure_config;
	fapi_l23_config_t l23_config;
	fapi_nb_iot_config_t nb_iot_config;
#if 0
	// addition fapi tlvs as per table 2-16 in idle or configure
	fapi_fapi_t fapi_config;
#endif
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_param_response_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_subframe_config_t subframe_config;
	fapi_rf_config_t rf_config;
	fapi_phich_config_t phich_config;
	fapi_sch_config_t sch_config;
	fapi_prach_config_t prach_config;
	fapi_pusch_config_t pusch_config;
	fapi_pucch_config_t pucch_config;
	fapi_srs_config_t srs_config;
	fapi_uplink_reference_signal_config_t uplink_reference_signal_config;
	fapi_laa_config_t laa_config;
	fapi_emtc_config_t emtc_config;
	fapi_tdd_frame_structure_t tdd_frame_structure_config;
	fapi_l23_config_t l23_config;
	fapi_nb_iot_config_t nb_iot_config;
#if 0
	// addition fapi tlvs as per table 2-16 in idle or configure
	fapi_fapi_t fapi_config;
#endif
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_config_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t num_tlv;
	fapi_uint32_tlv_t *tlv_buf;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_config_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	uint32_t wrong_tlv;
	uint32_t missing_tlv;
	fapi_uint32_tlv_t invalid_tlv_buf[VAR_SIZE];
	fapi_uint32_tlv_t missing_tlv_buf[VAR_SIZE];
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_config_response_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_start_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_stop_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_stop_indication_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_uint32_tlv_t dl_rs_tx_power;
	fapi_uint32_tlv_t received_interference_power;
	fapi_uint32_tlv_t thermal_noise_power;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_measurement_request_t;

#define FAPI_MEASUREMENT_REQUEST_DL_RS_XTX_POWER_TAG 0x1004
#define FAPI_MEASUREMENT_REQUEST_RECEIVED_INTERFERENCE_POWER_TAG 0x1005
#define FAPI_MEASUREMENT_REQUEST_THERMAL_NOISE_POWER_TAG 0x1006


typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_resource_blocks;
	int32_t received_interference_power[FAPI_MAX_RECEIVED_INTERFERENCE_POWER_RESULTS];
} fapi_received_interference_power_measurement_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_int16_tlv_t dl_rs_tx_power_measurement;
	fapi_received_interference_power_measurement_t received_interference_power_measurement;
	fapi_int16_tlv_t thermal_noise_power_measurement;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_measurement_response_t;

#define FAPI_MEASUREMENT_RESPONSE_DL_RS_POWER_MEASUREMENT_TAG 0x1007
#define FAPI_MEASUREMENT_RESPONSE_RECEIVED_INTERFERENCE_POWER_MEASUREMENT_TAG 0x1008
#define FAPI_MEASUREMENT_RESPONSE_THERMAL_NOISE_MEASUREMENT_TAG 0x1009

// P7 Sub Structures
typedef struct __attribute__((packed)) {
	uint32_t dci_format;
	uint32_t cce_idx;
	uint32_t aggregation_level;
	uint32_t rnti;
	uint32_t resource_allocation_type;
	uint32_t virtual_resource_block_assignment_flag;
	uint32_t resource_block_coding;
	uint32_t mcs_1;
	uint32_t redundancy_version_1;
	uint32_t new_data_indicator_1;
	uint32_t transport_block_to_codeword_swap_flag;
	uint32_t mcs_2;
	uint32_t redundancy_version_2;
	uint32_t new_data_indicator_2;
	uint32_t harq_process;
	uint32_t tpmi;
	uint32_t pmi;
	uint32_t precoding_information;
	uint32_t tpc;
	uint32_t downlink_assignment_index;
	uint32_t ngap;
	uint32_t transport_block_size_index;
	uint32_t downlink_power_offset;
	uint32_t allocate_prach_flag;
	uint32_t preamble_index;
	uint32_t prach_mask_index;
	uint32_t rnti_type;
	uint32_t transmission_power;
} fapi_dl_config_dci_dl_pdu_rel8_t;

typedef struct __attribute__((packed)) {
	uint32_t mcch_flag;
	uint32_t mcch_change_notification;
	uint32_t scrambling_identity;
} fapi_dl_config_dci_dl_pdu_rel9_t;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t cross_carrier_scheduling_flag;
	uint32_t carrier_indicator;
	uint32_t srs_flag;
	uint32_t srs_request;
	uint32_t antenna_ports_scrambling_and_layers;
	uint32_t total_dci_length_including_padding;
	uint32_t n_dl_rb;
} fapi_dl_config_dci_dl_pdu_rel10_t;

typedef struct __attribute__((packed)) {
	uint32_t harq_ack_resource_offset;
	uint32_t pdsch_re_mapping_quasi_co_location_indicator;
} fapi_dl_config_dci_dl_pdu_rel11_t;

typedef struct __attribute__((packed)) {
	uint32_t primary_cell_type;
	uint32_t ul_dl_configuration_flag;
	uint32_t number_ul_dl_configurations;
	uint32_t ul_dl_configuration_indication[FAPI_MAX_UL_DL_CONFIGURATIONS];
} fapi_dl_config_dci_dl_pdu_rel12_t;


typedef struct __attribute__((packed)) {
	uint32_t subband_index;
	uint32_t scheduled_ues;
	uint32_t precoding_value[FAPI_MAX_NUM_PHYSICAL_ANTENNAS][FAPI_MAX_NUM_SCHEDULED_UES];
} fapi_dl_config_dci_dl_tpm_subband_info_t;

typedef struct __attribute__((packed)) {
	uint32_t num_prb_per_subband;
	uint32_t number_of_subbands;
	uint32_t num_antennas;
	fapi_dl_config_dci_dl_tpm_subband_info_t subband_info[FAPI_MAX_NUM_SUBBANDS];
} fapi_dl_config_dci_dl_tpm_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t laa_end_partial_sf_flag;
	uint32_t laa_end_partial_sf_configuration;
	uint32_t initial_lbt_sf;
	uint32_t codebook_size_determination;
	uint32_t drms_table_flag;
	uint32_t tpm_struct_flag;
	fapi_dl_config_dci_dl_tpm_t tpm;
} fapi_dl_config_dci_dl_pdu_rel13_t;
#endif

typedef struct __attribute__((packed)) {
	fapi_dl_config_dci_dl_pdu_rel8_t dci_dl_pdu_rel8;
	fapi_dl_config_dci_dl_pdu_rel9_t dci_dl_pdu_rel9;
#if 0
	fapi_dl_config_dci_dl_pdu_rel10_t dci_dl_pdu_rel10;
	fapi_dl_config_dci_dl_pdu_rel11_t dci_dl_pdu_rel11;
	fapi_dl_config_dci_dl_pdu_rel12_t dci_dl_pdu_rel12;
	fapi_dl_config_dci_dl_pdu_rel13_t dci_dl_pdu_rel13;
#endif
} fapi_dl_config_dci_dl_pdu;

typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t pdu_index;
	uint32_t transmission_power;
} fapi_dl_config_bch_pdu_rel8_t;

typedef struct __attribute__((packed)) {
	fapi_dl_config_bch_pdu_rel8_t bch_pdu_rel8;
} fapi_dl_config_bch_pdu;

typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t pdu_index;
	uint32_t rnti;
	uint32_t resource_allocation_type;
	uint32_t resource_block_coding;
	uint32_t modulation;
	uint32_t transmission_power;
	uint32_t mbsfn_area_id;
} fapi_dl_config_mch_pdu_rel8_t;

typedef struct __attribute__((packed)) {
	fapi_dl_config_mch_pdu_rel8_t mch_pdu_rel8;
} fapi_dl_config_mch_pdu;

typedef struct __attribute__((packed)) {
	uint32_t subband_index;
	uint32_t num_antennas;
	uint32_t bf_value[FAPI_MAX_NUM_ANTENNAS];
} fapi_bf_vector_t;

typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t pdu_index;
	uint32_t rnti;
	uint32_t resource_allocation_type;
	uint32_t virtual_resource_block_assignment_flag;
	uint32_t resource_block_coding;
	uint32_t modulation;
	uint32_t redundancy_version;
	uint32_t transport_blocks;
	uint32_t transport_block_to_codeword_swap_flag;
	uint32_t transmission_scheme;
	uint32_t number_of_layers;
	uint32_t number_of_subbands;
	uint32_t codebook_index[FAPI_MAX_NUM_SUBBANDS];
	uint32_t ue_category_capacity;
	uint32_t pa;
	uint32_t delta_power_offset_index;
	uint32_t ngap;
	uint32_t nprb;
	uint32_t transmission_mode;
	uint32_t num_bf_prb_per_subband;
	uint32_t num_bf_vector;
	fapi_bf_vector_t bf_vector[FAPI_MAX_BF_VECTORS];
} fapi_dl_config_dlsch_pdu_rel8_t;

typedef struct __attribute__((packed)) {
	uint32_t nscid;
} fapi_dl_config_dlsch_pdu_rel9_t;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t csi_rs_flag;
	uint32_t csi_rs_resource_config_r10;
	uint32_t csi_rs_zero_tx_power_resource_config_bitmap_r10;
	uint32_t csi_rs_number_nzp_configuration;
	uint32_t csi_rs_resource_config[FAPI_MAX_CSI_RS_RESOURCE_CONFIG];
	uint32_t pdsch_start;
} fapi_dl_config_dlsch_pdu_rel10_t;

typedef struct __attribute__((packed)) {
	uint32_t drms_config_flag;
	uint32_t drms_scrambling;
	uint32_t csi_config_flag;
	uint32_t csi_scrambling;
	uint32_t pdsch_re_mapping_flag;
	uint32_t pdsch_re_mapping_atenna_ports;
	uint32_t pdsch_re_mapping_freq_shift;
} fapi_dl_config_dlsch_pdu_rel11_t;

typedef struct __attribute__((packed)) {
	uint32_t altcqi_table_r12;
	uint32_t maxlayers;
	uint32_t n_dl_harq;
} fapi_dl_config_dlsch_pdu_rel12_t;

typedef struct __attribute__((packed)) {
	uint32_t dwpts_symbols;
	uint32_t initial_lbt_sf;
	uint32_t ue_type;
	uint32_t pdsch_payload_type;
	uint32_t initial_transmission_sf_io;
	uint32_t drms_table_flag;
} fapi_dl_config_dlsch_pdu_rel13_t;
#endif

typedef struct __attribute__((packed)) {
	fapi_dl_config_dlsch_pdu_rel8_t dlsch_pdu_rel8;
	fapi_dl_config_dlsch_pdu_rel9_t dlsch_pdu_rel9;
#if 0
	fapi_dl_config_dlsch_pdu_rel10_t dlsch_pdu_rel10;
	fapi_dl_config_dlsch_pdu_rel11_t dlsch_pdu_rel11;
	fapi_dl_config_dlsch_pdu_rel12_t dlsch_pdu_rel12;
	fapi_dl_config_dlsch_pdu_rel13_t dlsch_pdu_rel13;
#endif
} fapi_dl_config_dlsch_pdu;

typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t pdu_index;
	uint32_t p_rnti;
	uint32_t resource_allocation_type;
	uint32_t virtual_resource_block_assignment_flag;
	uint32_t resource_block_coding;
	uint32_t mcs;
	uint32_t redundancy_version;
	uint32_t number_of_transport_blocks;
	uint32_t transport_block_to_codeword_swap_flag;
	uint32_t transmission_scheme;
	uint32_t number_of_layers;
	uint32_t codebook_index;
	uint32_t ue_category_capacity;
	uint32_t pa;
	uint32_t transmission_power;
	uint32_t nprb;
	uint32_t ngap;
} fapi_dl_config_pch_pdu_rel8_t;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t ue_mode;
	uint32_t initial_transmission_sf_io;
} fapi_dl_config_pch_pdu_rel13_t;
#endif

typedef struct __attribute__((packed)) {
	fapi_dl_config_pch_pdu_rel8_t pch_pdu_rel8;
#if 0
	fapi_dl_config_pch_pdu_rel13_t pch_pdu_rel13;
#endif
} fapi_dl_config_pch_pdu;

typedef struct __attribute__((packed)) {
	uint32_t transmission_power;
	uint32_t prs_bandwidth;
	uint32_t prs_cyclic_prefix_type;
	uint32_t prs_muting;
} fapi_dl_config_prs_pdu_rel9_t;

typedef struct __attribute__((packed)) {
	fapi_dl_config_prs_pdu_rel9_t prs_pdu_rel9;
} fapi_dl_config_prs_pdu;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t csi_rs_antenna_port_count_r10;
	uint32_t csi_rs_resource_config_r10;
	uint32_t transmission_power;
	uint32_t csi_rs_zero_tx_power_resource_config_bitmap_r10;
	uint32_t csi_rs_number_of_nzp_configuration;
	uint32_t csi_rs_resource_config[FAPI_MAX_CSI_RS_RESOURCE_CONFIG];
} fapi_dl_config_csi_rs_pdu_rel10_t;

typedef struct __attribute__((packed)) {
	uint32_t csi_rs_class;
	uint32_t cdm_type;
	uint32_t num_bf_vector;
	struct __attribute__((packed)) {
		uint32_t csi_rs_resource_index;
		uint32_t bf_value[FAPI_MAX_ANTENNA_PORT_COUNT];
	} bf_vector[FAPI_MAX_BF_VECTORS];

}fapi_dl_config_csi_rs_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_dl_config_csi_rs_pdu_rel10_t csi_rs_pdu_rel10;
	fapi_dl_config_csi_rs_pdu_rel13_t csi_rs_pdu_rel13;
} fapi_dl_config_csi_rs_pdu;
#endif


typedef struct __attribute__((packed)) {
	uint32_t epdcch_resource_assignment_flag;
	uint32_t epdcch_id;
	uint32_t epdcch_start_symbol;
	uint32_t epdcch_num_prb;
	uint32_t epdcch_prb_index[FAPI_MAX_EPDCCH_PRB];
	fapi_bf_vector_t bf_vector;
} fapi_dl_config_epdcch_parameters_rel11_t;

typedef struct __attribute__((packed)) {
	uint32_t dwpts_symbols;
	uint32_t initial_lbt_sf;
} fapi_dl_config_epdcch_parameters_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_dl_config_dci_dl_pdu_rel8_t			epdcch_pdu_rel8;
	fapi_dl_config_dci_dl_pdu_rel9_t			epdcch_pdu_rel9;
#if 0
	fapi_dl_config_dci_dl_pdu_rel10_t			epdcch_pdu_rel10;
	fapi_dl_config_dci_dl_pdu_rel11_t			epdcch_pdu_rel11;
	fapi_dl_config_dci_dl_pdu_rel12_t			epdcch_pdu_rel12;
	fapi_dl_config_dci_dl_pdu_rel13_t			epdcch_pdu_rel13;
	fapi_dl_config_epdcch_parameters_rel11_t	epdcch_params_rel11;
	fapi_dl_config_epdcch_parameters_rel13_t	epdcch_params_rel13;
#endif
} fapi_dl_config_epdcch_pdu;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t mpdcch_narrow_band;
	uint32_t number_of_prb_pairs;
	uint32_t resource_block_assignment;
	uint32_t mpdcch_tansmission_type;
	uint32_t start_symbol;
	uint32_t ecce_index;
	uint32_t aggregation_level;
	uint32_t rnti_type;
	uint32_t rnti;
	uint32_t ce_mode;
	uint32_t drms_scrambling_init;
	uint32_t initial_transmission_sf_io;
	uint32_t transmission_power;
	uint32_t dci_format;
	uint32_t resource_block_coding;
	uint32_t mcs;
	uint32_t pdsch_reptition_levels;
	uint32_t redundancy_version;
	uint32_t new_data_indicator;
	uint32_t harq_process;
	uint32_t tpmi_length;
	uint32_t tpmi;
	uint32_t pmi_flag;
	uint32_t pmi;
	uint32_t harq_resource_offset;
	uint32_t dci_subframe_repetition_number;
	uint32_t tpc;
	uint32_t downlink_assignment_index_length;
	uint32_t downlink_assignment_index;
	uint32_t allocate_prach_flag;
	uint32_t preamble_index;
	uint32_t prach_mask_index;
	uint32_t starting_ce_level;
	uint32_t srs_request;
	uint32_t antenna_ports_and_scrambling_identity_flag;
	uint32_t antenna_ports_and_scrambling_identity;
	uint32_t frequency_hopping_enabled_flag;
	uint32_t paging_direct_indication_differentiation_flag;
	uint32_t direct_indication;
	uint32_t total_dci_length_including_padding;
	uint32_t number_of_tx_antenna_ports;
	uint32_t precoding_value[FAPI_MAX_TX_PHYSICAL_ANTENNA_PORTS];
} fapi_dl_config_mpdcch_pdu_rel13_t;


typedef struct __attribute__((packed)) {
	fapi_dl_config_mpdcch_pdu_rel13_t mpdcch_pdu_rel13;
} fapi_dl_config_mpdcch_pdu;
#endif

typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t pdu_index;
	uint32_t transmission_power;
	uint32_t hyper_sfn_2_lsbs;
} fapi_dl_config_nbch_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_dl_config_nbch_pdu_rel13_t nbch_pdu_rel13;
} fapi_dl_config_nbch_pdu;

typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t pdu_index;
	uint32_t ncce_index;
	uint32_t aggregation_level;
	uint32_t start_symbol;
	uint32_t rnti_type;
	uint32_t rnti;
	uint32_t scrambling_reinitialization_batch_index;
	uint32_t nrs_antenna_ports_assumed_by_the_ue;
	uint32_t dci_format;
	uint32_t scheduling_delay;
	uint32_t resource_assignment;
	uint32_t repetition_number;
	uint32_t mcs;
	uint32_t new_data_indicator;
	uint32_t harq_ack_resource;
	uint32_t npdcch_order_indication;
	uint32_t starting_number_of_nprach_repetitions;
	uint32_t subcarrier_indication_of_nprach;
	uint32_t paging_direct_indication_differentation_flag;
	uint32_t direct_indication;
	uint32_t dci_subframe_repetition_number;
	uint32_t total_dci_length_including_padding;
} fapi_dl_config_npdcch_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_dl_config_npdcch_pdu_rel13_t npdcch_pdu_rel13;
} fapi_dl_config_npdcch_pdu;

typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t pdu_index;
	uint32_t start_symbol;
	uint32_t rnti_type;
	uint32_t rnti;
	uint32_t resource_assignment;
	uint32_t repetition_number;
	uint32_t modulation;
	uint32_t number_of_subframes_for_resource_assignment;
	uint32_t scrambling_sequence_initialization_cinit;
	uint32_t sf_idx;
	uint32_t nrs_antenna_ports_assumed_by_the_ue;
} fapi_dl_config_ndlsch_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_dl_config_ndlsch_pdu_rel13_t ndlsch_pdu_rel13;
} fapi_dl_config_ndlsch_pdu;

typedef struct __attribute__((packed)) {
	uint32_t pdu_type;
	uint32_t pdu_size;
	union {
		fapi_dl_config_dci_dl_pdu	dci_dl_pdu;
		fapi_dl_config_bch_pdu		bch_pdu;
		fapi_dl_config_mch_pdu		mch_pdu;
		fapi_dl_config_dlsch_pdu	dlsch_pdu;
		fapi_dl_config_pch_pdu		pch_pdu;
		fapi_dl_config_prs_pdu		prs_pdu;
#if 0
		fapi_dl_config_csi_rs_pdu	csi_rs_pdu;
#endif
		fapi_dl_config_epdcch_pdu	epdcch_pdu;
#if 0
		fapi_dl_config_mpdcch_pdu	mpdcch_pdu;
#endif
		fapi_dl_config_nbch_pdu	nbch_pdu;
		fapi_dl_config_npdcch_pdu	npdcch_pdu;
		fapi_dl_config_ndlsch_pdu	ndlsch_pdu;
	};
} fapi_dl_config_request_pdu_t;

typedef struct __attribute__((packed)) {
	uint32_t number_pdcch_ofdm_symbols;
	uint32_t number_dci;
	uint32_t number_pdu;
	uint32_t number_pdsch_rnti;
	uint32_t transmission_power_pcfich;
	fapi_dl_config_request_pdu_t dl_config_pdu_list[VAR_SIZE];
} fapi_dl_config_request_body_t;

typedef struct __attribute__((packed)) {
	uint32_t handle;
	uint32_t size;
	uint32_t rnti;
	uint32_t resource_block_start;
	uint32_t number_of_resource_blocks;
	uint32_t modulation_type;
	uint32_t cyclic_shift_2_for_drms;
	uint32_t frequency_hopping_enabled_flag;
	uint32_t frequency_hopping_bits;
	uint32_t new_data_indication;
	uint32_t redundancy_version;
	uint32_t harq_process_number;
	uint32_t ul_tx_mode;
	uint32_t current_tx_nb;
	uint32_t n_srs;
} fapi_ul_config_ulsch_pdu_rel8_t;
#if 0
typedef struct __attribute__((packed)) {
	uint32_t resource_allocation_type;
	uint32_t resource_block_coding;
	uint32_t transport_blocks;
	uint32_t transmission_scheme;
	uint32_t number_of_layers;
	uint32_t codebook_index;
	uint32_t disable_sequence_hopping_flag;
} fapi_ul_config_ulsch_pdu_rel10_t;

typedef struct __attribute__((packed)) {
	uint32_t virtual_cell_id_enabled_flag;
	uint32_t npusch_identity;
	uint32_t dmrs_config_flag;
	uint32_t ndmrs_csh_identity;
} fapi_ul_config_ulsch_pdu_rel11_t;

typedef struct __attribute__((packed)) {
	uint32_t  ue_type;
	uint32_t total_number_of_repetitions;
	uint32_t repetition_number;
	uint32_t initial_transmission_sf_io;
	uint32_t  empty_symbols_due_to_re_tunning;
} fapi_ul_config_ulsch_pdu_rel13_t;
#endif

typedef struct __attribute__((packed)) {
	fapi_ul_config_ulsch_pdu_rel8_t ulsch_pdu_rel8;
#if 0
	fapi_ul_config_ulsch_pdu_rel10_t ulsch_pdu_rel10;
	fapi_ul_config_ulsch_pdu_rel11_t ulsch_pdu_rel11;
	fapi_ul_config_ulsch_pdu_rel13_t ulsch_pdu_rel13;
#endif
} fapi_ul_config_ulsch_pdu;

typedef struct __attribute__((packed)) {
	uint32_t dl_cqi_pmi_size_rank_1;
	uint32_t dl_cqi_pmi_size_rank_greater_1;
	uint32_t ri_size;
	uint32_t delta_offset_cqi;
	uint32_t delta_offset_ri;
} fapi_ul_config_cqi_ri_information_rel8_t;

typedef struct __attribute__((packed)) {
	uint32_t dl_cqi_pmi_ri_size;
	uint32_t control_type;
} fapi_ul_config_periodic_cqi_pmi_ri_report_t;

typedef struct __attribute__((packed)) {
	uint32_t number_of_cc;
	struct __attribute__((packed)) {
		uint32_t ri_size;
		uint32_t dl_cqi_pmi_size;
	} cc[FAPI_MAX_CC];
} fapi_ul_config_aperiodic_cqi_pmi_ri_report_t;

typedef struct __attribute__((packed)) {
	uint32_t report_type;
	uint32_t delta_offset_cqi;
	uint32_t delta_offset_ri;
	union {
		fapi_ul_config_periodic_cqi_pmi_ri_report_t periodic_cqi_pmi_ri_report;
		fapi_ul_config_aperiodic_cqi_pmi_ri_report_t aperiodic_cqi_pmi_ri_report;
	};
} fapi_ul_config_cqi_ri_information_rel9_t;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t dl_cqi_pmi_ri_size_2;
} fapi_ul_config_periodic_cqi_pmi_ri_report_re13_t;

typedef struct __attribute__((packed)) {
} fapi_ul_config_aperiodic_cqi_pmi_ri_report_re13_t;

typedef struct __attribute__((packed)) {
	uint32_t report_type; // Convience parameter, not sent on the wire
	union {
		fapi_ul_config_periodic_cqi_pmi_ri_report_re13_t periodic_cqi_pmi_ri_report;
		fapi_ul_config_aperiodic_cqi_pmi_ri_report_re13_t aperiodic_cqi_pmi_ri_report;
	};
} fapi_ul_config_cqi_ri_information_rel13_t;
#endif

typedef struct __attribute__((packed)) {
	fapi_ul_config_cqi_ri_information_rel8_t cqi_ri_information_rel8;
#if 0
	fapi_ul_config_cqi_ri_information_rel9_t cqi_ri_information_rel9;
	fapi_ul_config_cqi_ri_information_rel13_t cqi_ri_information_rel13;
#endif
} fapi_ul_config_cqi_ri_information;

typedef struct __attribute__((packed)) {
	uint32_t harq_size;
	uint32_t delta_offset_harq;
	uint32_t ack_nack_mode;
} fapi_ul_config_ulsch_harq_information_rel10_t;

typedef struct __attribute__((packed)) {
	uint32_t harq_size_2;
	uint32_t delta_offset_harq_2;
} fapi_ul_config_ulsch_harq_information_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ulsch_harq_information_rel10_t harq_information_rel10;
	fapi_ul_config_ulsch_harq_information_rel13_t harq_information_rel13;
} fapi_ul_config_ulsch_harq_information;

typedef struct __attribute__((packed)) {
	uint32_t n_srs_initial;
	uint32_t initial_number_of_resource_blocks;
} fapi_ul_config_initial_transmission_parameters_rel8_t;

typedef struct __attribute__((packed)) {
	fapi_ul_config_initial_transmission_parameters_rel8_t initial_transmission_parameters_rel8;
} fapi_ul_config_initial_transmission_parameters;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ulsch_pdu ulsch_pdu;
	fapi_ul_config_cqi_ri_information cqi_ri_information;
	fapi_ul_config_initial_transmission_parameters initial_transmission_parameters;
} fapi_ul_config_ulsch_cqi_ri_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ulsch_pdu ulsch_pdu;
	fapi_ul_config_ulsch_harq_information harq_information;
	fapi_ul_config_initial_transmission_parameters initial_transmission_parameters;
} fapi_ul_config_ulsch_harq_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ulsch_pdu ulsch_pdu;
	fapi_ul_config_cqi_ri_information cqi_ri_information;
	fapi_ul_config_ulsch_harq_information harq_information;
	fapi_ul_config_initial_transmission_parameters initial_transmission_parameters;
} fapi_ul_config_ulsch_cqi_harq_ri_pdu;


typedef struct __attribute__((packed)) {
	uint32_t handle;
	uint32_t rnti;
} fapi_ul_config_ue_information_rel8_t;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t virtual_cell_id_enabled_flag;
	uint32_t npusch_identity;
} fapi_ul_config_ue_information_rel11_t;

typedef struct __attribute__((packed)) {
	uint32_t  ue_type;
	uint32_t  empty_symbols;
	uint32_t total_number_of_repetitions;
	uint32_t repetition_number;
} fapi_ul_config_ue_information_rel13_t;
#endif

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information_rel8_t ue_information_rel8;
#if 0
	fapi_ul_config_ue_information_rel11_t ue_information_rel11;
	fapi_ul_config_ue_information_rel13_t ue_information_rel13;
#endif
} fapi_ul_config_ue_information;

typedef struct __attribute__((packed)) {
	uint32_t pucch_index;
	uint32_t dl_cqi_pmi_size;
} fapi_ul_config_cqi_information_rel8_t;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t number_of_pucch_resource;
	uint32_t pucch_index_p1;
} fapi_ul_config_cqi_information_rel10_t;

typedef struct __attribute__((packed)) {
	uint32_t csi_mode;
	uint32_t dl_cqi_pmi_size_2;
	uint32_t starting_prb;
	uint32_t n_prb;
	uint32_t cdm_index;
	uint32_t n_srs;
} fapi_ul_config_cqi_information_rel13_t;
#endif

typedef struct __attribute__((packed)) {
	fapi_ul_config_cqi_information_rel8_t cqi_information_rel8;
#if 0
	fapi_ul_config_cqi_information_rel10_t cqi_information_rel10;
	fapi_ul_config_cqi_information_rel13_t cqi_information_rel13;
#endif
} fapi_ul_config_cqi_information;

typedef struct __attribute__((packed)) {
	uint32_t pucch_index;
} fapi_ul_config_sr_information_rel8_t;

#if 0
typedef struct __attribute__((packed)) {
	uint32_t number_of_pucch_resources;
	uint32_t pucch_index_p1;
} fapi_ul_config_sr_information_rel10_t;
#endif

typedef struct __attribute__((packed)) { 
	fapi_ul_config_sr_information_rel8_t sr_information_rel8;
#if 0
	fapi_ul_config_sr_information_rel10_t sr_information_rel10;
#endif
} fapi_ul_config_sr_information;

typedef struct __attribute__((packed)) { 
	uint32_t harq_size;
	uint32_t ack_nack_mode;
	uint32_t number_of_pucch_resources;
	uint32_t n_pucch_1_0;
	uint32_t n_pucch_1_1;
	uint32_t n_pucch_1_2;
	uint32_t n_pucch_1_3;
} fapi_ul_config_harq_information_rel10_tdd_t;


typedef struct __attribute__((packed)) { 
	uint32_t n_pucch_1_0;
	uint32_t harq_size;
} fapi_ul_config_harq_information_rel8_fdd_t;

typedef struct __attribute__((packed)) { 
	uint32_t harq_size;
	uint32_t ack_nack_mode;
	uint32_t number_of_pucch_resources;
	uint32_t n_pucch_1_0;
	uint32_t n_pucch_1_1;
	uint32_t n_pucch_1_2;
	uint32_t n_pucch_1_3;
} fapi_ul_config_harq_information_rel9_fdd_t;

typedef struct __attribute__((packed)) { 
	uint32_t  num_ant_ports;
	uint32_t n_pucch_2_0;
	uint32_t n_pucch_2_1;
	uint32_t n_pucch_2_2;
	uint32_t n_pucch_2_3;	
} fapi_ul_config_harq_information_rel11_t;

typedef struct __attribute__((packed)) { 
	uint32_t  harq_size_2;
	uint32_t starting_prb;
	uint32_t n_prb;
	uint32_t cdm_index;
	uint32_t n_srs;
} fapi_ul_config_harq_information_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_ul_config_harq_information_rel10_tdd_t harq_information_rel10_tdd;
	fapi_ul_config_harq_information_rel8_fdd_t harq_information_rel8_fdd;
	fapi_ul_config_harq_information_rel9_fdd_t harq_information_rel9_fdd;
	fapi_ul_config_harq_information_rel11_t harq_information_rel11;
	fapi_ul_config_harq_information_rel13_t harq_information_rel13;
} fapi_ul_config_harq_information;

typedef struct __attribute__((packed)) { 
	uint32_t handle;
	uint32_t size;
	uint32_t rnti;
	uint32_t srs_bandwidth;
	uint32_t frequency_domain_position;
	uint32_t srs_hopping_bandwidth;
	uint32_t transmission_comb;
	uint32_t i_srs;
	uint32_t sounding_reference_cyclic_shift;
} fapi_ul_config_srs_pdu_rel8_t;

typedef struct __attribute__((packed)) { 
	uint32_t antenna_port;
} fapi_ul_config_srs_pdu_rel10_t;

typedef struct __attribute__((packed)) { 
	uint32_t number_of_combs;
} fapi_ul_config_srs_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_ul_config_srs_pdu_rel8_t srs_pdu_rel8;
	fapi_ul_config_srs_pdu_rel10_t srs_pdu_rel10;
	fapi_ul_config_srs_pdu_rel13_t srs_pdu_rel13;
} fapi_ul_config_srs_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information ue_information;
	fapi_ul_config_cqi_information cqi_information;
} fapi_ul_config_uci_cqi_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information ue_information;
	fapi_ul_config_sr_information sr_information;
} fapi_ul_config_uci_sr_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information ue_information;
	fapi_ul_config_harq_information harq_information;
} fapi_ul_config_uci_harq_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information ue_information;
	fapi_ul_config_sr_information sr_information;
	fapi_ul_config_harq_information harq_information;
} fapi_ul_config_uci_sr_harq_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information ue_information;
	fapi_ul_config_cqi_information cqi_information;
	fapi_ul_config_harq_information harq_information;
} fapi_ul_config_uci_cqi_harq_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information ue_information;
	fapi_ul_config_cqi_information cqi_information;
	fapi_ul_config_sr_information sr_information;
} fapi_ul_config_uci_cqi_sr_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information ue_information;
	fapi_ul_config_cqi_information cqi_information;
	fapi_ul_config_sr_information sr_information;
	fapi_ul_config_harq_information harq_information;
} fapi_ul_config_uci_cqi_sr_harq_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ue_information ue_information;
} fapi_ul_config_harq_buffer_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ulsch_pdu ulsch_pdu;
	fapi_ul_config_cqi_information csi_information;
} fapi_ul_config_ulsch_uci_csi_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ulsch_pdu ulsch_pdu;
	fapi_ul_config_harq_information harq_information;
} fapi_ul_config_ulsch_uci_harq_pdu;

typedef struct __attribute__((packed)) {
	fapi_ul_config_ulsch_pdu ulsch_pdu;
	fapi_ul_config_cqi_information csi_information;
	fapi_ul_config_harq_information harq_information;
} fapi_ul_config_ulsch_csi_uci_harq_pdu;

typedef struct __attribute__((packed)) {
	uint32_t harq_ack_resource;
} fapi_ul_config_nb_harq_information_rel13_fdd_t;

typedef struct __attribute__((packed)) {
	fapi_ul_config_nb_harq_information_rel13_fdd_t nb_harq_information_rel13_fdd;
} fapi_ul_config_nb_harq_information;

typedef struct __attribute__((packed)) {
	uint32_t nulsch_format;
	uint32_t handle;
	uint32_t size;
	uint32_t rnti;
	uint32_t subcarrier_indication;
	uint32_t resource_assignment;
	uint32_t mcs;
	uint32_t redudancy_version;
	uint32_t repetition_number;
	uint32_t new_data_indication;
	uint32_t n_srs;
	uint32_t scrambling_sequence_initialization_cinit;
	uint32_t sf_idx;
	fapi_ul_config_ue_information ue_information;
	fapi_ul_config_nb_harq_information nb_harq_information;
} fapi_ul_config_nulsch_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_ul_config_nulsch_pdu_rel13_t nulsch_pdu_rel13;
} fapi_ul_config_nulsch_pdu;


typedef struct __attribute__((packed)) {
	uint32_t nprach_config_0;
	uint32_t nprach_config_1;
	uint32_t nprach_config_2;
} fapi_ul_config_nrach_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_ul_config_nrach_pdu_rel13_t nrach_pdu_rel13;
} fapi_ul_config_nrach_pdu;

typedef struct __attribute__((packed)) {
	uint32_t pdu_type;
	uint32_t pdu_size;
	union {
		fapi_ul_config_ulsch_pdu				ulsch_pdu;
		fapi_ul_config_ulsch_cqi_ri_pdu		ulsch_cqi_ri_pdu;
		fapi_ul_config_ulsch_harq_pdu			ulsch_harq_pdu;
		fapi_ul_config_ulsch_cqi_harq_ri_pdu	ulsch_cqi_harq_ri_pdu;
		fapi_ul_config_uci_cqi_pdu				uci_cqi_pdu;
		fapi_ul_config_uci_sr_pdu				uci_sr_pdu;
		fapi_ul_config_uci_harq_pdu			uci_harq_pdu;
		fapi_ul_config_uci_sr_harq_pdu			uci_sr_harq_pdu;
		fapi_ul_config_uci_cqi_harq_pdu		uci_cqi_harq_pdu;
		fapi_ul_config_uci_cqi_sr_pdu			uci_cqi_sr_pdu;
		fapi_ul_config_uci_cqi_sr_harq_pdu		uci_cqi_sr_harq_pdu;
		fapi_ul_config_srs_pdu					srs_pdu;
		fapi_ul_config_harq_buffer_pdu			harq_buffer_pdu;
		fapi_ul_config_ulsch_uci_csi_pdu		ulsch_uci_csi_pdu;
		fapi_ul_config_ulsch_uci_harq_pdu		ulsch_uci_harq_pdu;
		fapi_ul_config_ulsch_csi_uci_harq_pdu	ulsch_csi_uci_harq_pdu;
		fapi_ul_config_nulsch_pdu				nulsch_pdu;
		fapi_ul_config_nrach_pdu				nrach_pdu;
	};
} fapi_ul_config_request_pdu_t;

typedef struct __attribute__((packed)) {
	uint32_t number_of_pdus;
	uint32_t rach_prach_frequency_resources;
	uint32_t srs_present;
	fapi_ul_config_request_pdu_t ul_config_pdu_list[VAR_SIZE];
} fapi_ul_config_request_body_t;

/* 4.2.3 */

/* 4-111 */
typedef struct __attribute__((packed)) {
	uint32_t resource_block_start;
	uint32_t cyclic_shift_2_for_drms;
	uint32_t hi_value;
	uint32_t i_phich;
	uint32_t transmission_power;
} fapi_hi_dci0_hi_pdu_rel8_t;

/* 4-112 */
typedef struct __attribute__((packed)) {
	uint32_t flag_tb2;
	uint32_t hi_value_2;
} fapi_hi_dci0_hi_pdu_rel10_t;

/* 4-110 */
typedef struct __attribute__((packed)) {
	fapi_hi_dci0_hi_pdu_rel8_t		hi_pdu_rel8;
	fapi_hi_dci0_hi_pdu_rel10_t	hi_pdu_rel10;
} fapi_hi_dci0_hi_pdu;

/* 4-114 */
typedef struct __attribute__((packed)) {
	uint32_t dci_format;
	uint32_t cce_index;
	uint32_t aggregation_level;
	uint32_t rnti;
	uint32_t resource_block_start;
	uint32_t number_of_resource_block;
	uint32_t mcs_1;
	uint32_t cyclic_shift_2_for_drms;
	uint32_t frequency_hopping_enabled_flag;
	uint32_t frequency_hopping_bits;
	uint32_t new_data_indication_1;
	uint32_t ue_tx_antenna_seleciton;
	uint32_t tpc;
	uint32_t cqi_csi_request;
	uint32_t ul_index;
	uint32_t dl_assignment_index;
	uint32_t tpc_bitmap;
	uint32_t transmission_power;
} fapi_hi_dci0_dci_pdu_rel8_t;

/* 4-115 */
typedef struct __attribute__((packed)) {
	uint32_t cross_carrier_scheduling_flag;
	uint32_t carrier_indicator;
	uint32_t size_of_cqi_csi_feild;
	uint32_t srs_flag;
	uint32_t srs_request;
	uint32_t resource_allocation_flag;
	uint32_t resource_allocation_type;
	uint32_t resource_block_coding;
	uint32_t mcs_2;
	uint32_t new_data_indication_2;
	uint32_t number_of_antenna_ports;
	uint32_t tpmi;
	uint32_t total_dci_length_including_padding;
	uint32_t n_ul_rb;
} fapi_hi_dci0_dci_pdu_rel10_t;

/* 4-116 */
typedef struct __attribute__((packed)) {
	uint32_t pscch_resource;
	uint32_t time_resource_pattern;
} fapi_hi_dci0_dci_pdu_rel12_t;

/* 4.2.3.2 */

/* 4-113 */
typedef struct __attribute__((packed)) {
	fapi_hi_dci0_dci_pdu_rel8_t	dci_pdu_rel8;
	fapi_hi_dci0_dci_pdu_rel10_t	dci_pdu_rel10;
	fapi_hi_dci0_dci_pdu_rel12_t	dci_pdu_rel12;
} fapi_hi_dci0_dci_pdu;

/* 4.2.3.3 */

typedef fapi_hi_dci0_dci_pdu_rel8_t fapi_hi_dci0_epdcch_dci_pdu_rel8_t;

typedef fapi_hi_dci0_dci_pdu_rel10_t fapi_hi_dci0_epdcch_dci_pdu_rel10_t;

typedef fapi_dl_config_epdcch_parameters_rel11_t fapi_hi_dci0_epdcch_parameters_rel11_t;

/* 4-117 */
typedef struct __attribute__((packed)) {
	fapi_hi_dci0_epdcch_dci_pdu_rel8_t		epdcch_dci_pdu_rel8;
	fapi_hi_dci0_epdcch_dci_pdu_rel10_t	epdcch_dci_pdu_rel10;
	fapi_hi_dci0_epdcch_parameters_rel11_t	epdcch_parameters_rel11;
} fapi_hi_dci0_epdcch_dci_pdu;

/* 4.2.3.4 */
/* 4-119 */
typedef struct __attribute__((packed)) {
	uint32_t mpdcch_narrowband;
	uint32_t number_of_prb_pairs;
	uint32_t resource_block_assignment;
	uint32_t mpdcch_transmission_type;
	uint32_t start_symbol;
	uint32_t ecce_index;
	uint32_t aggreagation_level;
	uint32_t rnti_type;
	uint32_t rnti;
	uint32_t ce_mode;
	uint32_t drms_scrambling_init;
	uint32_t initial_transmission_sf_io;
	uint32_t transmission_power;
	uint32_t dci_format;
	uint32_t resource_block_start;
	uint32_t number_of_resource_blocks;
	uint32_t mcs;
	uint32_t pusch_repetition_levels;
	uint32_t frequency_hopping_flag;
	uint32_t new_data_indication;
	uint32_t harq_process;
	uint32_t redudency_version;
	uint32_t tpc;
	uint32_t csi_request;
	uint32_t ul_inex;
	uint32_t dai_presence_flag;
	uint32_t dl_assignment_index;
	uint32_t srs_request;
	uint32_t dci_subframe_repetition_number;
	uint32_t tcp_bitmap;
	uint32_t total_dci_length_include_padding;
	uint32_t number_of_tx_antenna_ports;
	uint32_t precoding_value[FAPI_MAX_ANTENNA_PORT_COUNT];
} fapi_hi_dci0_mpdcch_dci_pdu_rel13_t;

/* 4-118 */
typedef struct __attribute__((packed)) {
	fapi_hi_dci0_mpdcch_dci_pdu_rel13_t	mpdcch_dci_pdu_rel13;
} fapi_hi_dci0_mpdcch_dci_pdu;

/* 4.2.3.5 */

/* 4-121 */
typedef struct __attribute__((packed)) {
	uint32_t ncce_index;
	uint32_t aggregation_level;
	uint32_t start_symbol;
	uint32_t rnti;
	uint32_t scrambling_reinitialization_batch_index;
	uint32_t nrs_antenna_ports_assumed_by_the_ue;
	uint32_t subcarrier_indication;
	uint32_t resource_assignment;
	uint32_t scheduling_delay;
	uint32_t mcs;
	uint32_t redudancy_version;
	uint32_t repetition_number;
	uint32_t new_data_indicator;
	uint32_t dci_subframe_repetition_number;
} fapi_hi_dci0_npdcch_dci_pdu_rel13_t;

/* 4-120 */
typedef struct __attribute__((packed)) {
	fapi_hi_dci0_npdcch_dci_pdu_rel13_t	npdcch_dci_pdu_rel13;
} fapi_hi_dci0_npdcch_dci_pdu;

/* 4-109 */
typedef struct __attribute__((packed)) {
	uint32_t pdu_type;
	uint32_t pdu_size;
	union {
		fapi_hi_dci0_hi_pdu			hi_pdu;
		fapi_hi_dci0_dci_pdu			dci_pdu;
		fapi_hi_dci0_epdcch_dci_pdu	epdcch_dci_pdu;
		fapi_hi_dci0_mpdcch_dci_pdu	mpdcch_dci_pdu;
		fapi_hi_dci0_npdcch_dci_pdu	npdcch_dci_pdu;
	};
} fapi_hi_dci0_request_pdu_t;

#define FAPI_HI_DCI0_MAX_PDU 100

/* 4-108 */
typedef struct __attribute__((packed)) {
	uint32_t sfnsf;
	uint32_t number_of_dci;
	uint32_t number_of_hi;
	fapi_hi_dci0_request_pdu_t hi_dci0_pdu_list[VAR_SIZE];
} fapi_hi_dci0_request_body_t;

#define FAPI_TX_MAX_SEGMENTS 32
typedef struct __attribute__((packed)) {
	uint32_t pdu_length;
	uint32_t pdu_index;
	uint32_t num_segments;
	struct __attribute__((packed)) {
		uint32_t tb_tag;
		uint32_t tb_length;
		void* tb_data;
	} tbs[FAPI_TX_MAX_SEGMENTS];
} fapi_tx_request_pdu_t;

#define FAPI_TX_MAX_PDU 100
typedef struct __attribute__((packed)) {
	uint32_t number_of_pdus;
	fapi_tx_request_pdu_t tx_pdu_list[VAR_SIZE];
} fapi_tx_request_body_t;

// P7 Message Structures
typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t t1;
	int32_t delta_sfn_sf;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_dl_node_sync_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t t1;
	uint32_t t2;
	uint32_t t3;	
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_ul_node_sync_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t last_sfn_sf;
	uint32_t time_since_last_timing_info;
	uint32_t dl_config_jitter;
	uint32_t tx_request_jitter;
	uint32_t ul_config_jitter;
	uint32_t hi_dci0_jitter;
	int32_t dl_config_latest_delay;
	int32_t tx_request_latest_delay;
	int32_t ul_config_latest_delay;
	int32_t hi_dci0_latest_delay;
	int32_t dl_config_earliest_arrival;
	int32_t tx_request_earliest_arrival;
	int32_t ul_config_earliest_arrival;
	int32_t hi_dci0_earliest_arrival;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_timing_info_t;

typedef struct __attribute__((packed)) {
	uint32_t handle;
	uint32_t rnti;
} fapi_rx_ue_information;

typedef struct __attribute__((packed)) { 
	uint32_t value_0;
	uint32_t value_1;
} fapi_harq_indication_tdd_harq_data_bundling_t;

typedef struct __attribute__((packed)) { 
	uint32_t value_0;
	uint32_t value_1;
	uint32_t value_2;
	uint32_t value_3;
} fapi_harq_indication_tdd_harq_data_multiplexing_t;

typedef struct __attribute__((packed)) { 
	uint32_t value_0;
} fapi_harq_indication_tdd_harq_data_special_bundling_t;

typedef struct __attribute__((packed)) { 
	uint32_t value_0;
} fapi_harq_indication_tdd_harq_data_t;

typedef struct __attribute__((packed)) { 
	uint32_t mode;
	uint32_t number_of_ack_nack;
	union{
		fapi_harq_indication_tdd_harq_data_bundling_t			bundling;
		fapi_harq_indication_tdd_harq_data_multiplexing_t		multiplex;
		fapi_harq_indication_tdd_harq_data_special_bundling_t	special_bundling;
	} harq_data;
} fapi_harq_indication_tdd_rel8_t;

typedef struct __attribute__((packed)) {
	uint32_t mode;
	uint32_t number_of_ack_nack;
	union{
		fapi_harq_indication_tdd_harq_data_t	bundling;
		fapi_harq_indication_tdd_harq_data_t	multiplex;
		fapi_harq_indication_tdd_harq_data_special_bundling_t	special_bundling;
		fapi_harq_indication_tdd_harq_data_t	channel_selection;
		fapi_harq_indication_tdd_harq_data_t	format_3;
	} harq_data[FAPI_MAX_NUMBER_ACK_NACK_TDD];
} fapi_harq_indication_tdd_rel9_t;

typedef struct __attribute__((packed)) {
	uint32_t mode;
	uint32_t number_of_ack_nack;
	union{
		fapi_harq_indication_tdd_harq_data_t					bundling;
		fapi_harq_indication_tdd_harq_data_t					multiplex;
		fapi_harq_indication_tdd_harq_data_special_bundling_t	special_bundling;
		fapi_harq_indication_tdd_harq_data_t					channel_selection;
		fapi_harq_indication_tdd_harq_data_t			format_3;
		fapi_harq_indication_tdd_harq_data_t			format_4;
		fapi_harq_indication_tdd_harq_data_t			format_5;
	} harq_data[FAPI_MAX_NUMBER_ACK_NACK_TDD];
} fapi_harq_indication_tdd_rel13_t;

typedef struct __attribute__((packed)) { 
	uint32_t harq_tb1;
	uint32_t harq_tb2;
} fapi_harq_indication_fdd_rel8_t;

#define FAPI_HARQ_ACK_NACK_REL9_MAX 10
typedef struct __attribute__((packed)) {
	uint32_t mode;
	uint32_t number_of_ack_nack;
	uint32_t harq_tb_n[FAPI_HARQ_ACK_NACK_REL9_MAX];
} fapi_harq_indication_fdd_rel9_t;

#define FAPI_HARQ_ACK_NACK_REL13_MAX 22 // Need to check this max?
typedef struct __attribute__((packed)) {
	uint32_t mode;
	uint32_t number_of_ack_nack;
	uint32_t harq_tb_n[FAPI_HARQ_ACK_NACK_REL13_MAX];
} fapi_harq_indication_fdd_rel13_t;

typedef struct __attribute__((packed)) {
	uint32_t ul_cqi;
	uint32_t channel;
} fapi_ul_cqi_information_t;

// Only expect 1 harq_indication TLV.tag to be set
// Would this be a better a an union, but not clear which combinations
// are valid
typedef struct __attribute__((packed)) {
	fapi_rx_ue_information				rx_ue_information;
	fapi_harq_indication_tdd_rel8_t	harq_indication_tdd_rel8;
	fapi_harq_indication_tdd_rel9_t	harq_indication_tdd_rel9;
	fapi_harq_indication_tdd_rel13_t	harq_indication_tdd_rel13;
	fapi_harq_indication_fdd_rel8_t	harq_indication_fdd_rel8;
	fapi_harq_indication_fdd_rel9_t	harq_indication_fdd_rel9;
	fapi_harq_indication_fdd_rel13_t	harq_indication_fdd_rel13;
	fapi_ul_cqi_information_t			ul_cqi_information;
} fapi_harq_indication_pdu_t;

#define FAPI_HARQ_IND_MAX_PDU 100
typedef struct __attribute__((packed)) {
	uint32_t number_of_harqs;
	fapi_harq_indication_pdu_t harq_pdu_list[VAR_SIZE];
} fapi_harq_indication_body_t;

typedef struct __attribute__((packed)) {
	uint32_t crc_flag;
} fapi_crc_indication_rel8_t;

typedef struct __attribute__((packed)) {
	fapi_rx_ue_information		rx_ue_information;
	fapi_crc_indication_rel8_t	crc_indication_rel8;
} fapi_crc_indication_pdu_t;

#define FAPI_CRC_IND_MAX_PDU 100
typedef struct __attribute__((packed)) {
	uint32_t number_of_crcs;
	fapi_crc_indication_pdu_t crc_pdu_list[VAR_SIZE];
} fapi_crc_indication_body_t;

typedef struct __attribute__((packed)) {
	fapi_rx_ue_information		rx_ue_information;
	fapi_ul_cqi_information_t	ul_cqi_information;
} fapi_sr_indication_pdu_t;

#define FAPI_SR_IND_MAX_PDU 100
typedef struct __attribute__((packed)) {
	uint32_t number_of_srs;				// Question : should this be srs
	fapi_sr_indication_pdu_t sr_pdu_list[VAR_SIZE];
} fapi_sr_indication_body_t;

// The data offset should be set to 0 or 1 before encoding
// If it is set to 1 the fapi library will detemine the correct offset

typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t data_offset;
	uint32_t ul_cqi;
	uint32_t ri;
	uint32_t timing_advance;
} fapi_cqi_indication_rel8_t;

#define FAPI_CC_MAX 4
typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t data_offset;
	uint32_t ul_cqi;
	uint32_t number_of_cc_reported;
	uint32_t ri[FAPI_CC_MAX];
	uint32_t timing_advance;
	uint32_t timing_advance_r9;
} fapi_cqi_indication_rel9_t;

typedef struct __attribute__((packed)) {
	fapi_rx_ue_information		rx_ue_information;
	fapi_cqi_indication_rel8_t cqi_indication_rel8;
	fapi_cqi_indication_rel9_t cqi_indication_rel9;
	fapi_ul_cqi_information_t	ul_cqi_information;
} fapi_cqi_indication_pdu_t;

#define FAPI_CQI_RAW_MAX_LEN 12
typedef struct __attribute__((packed)) {
	uint32_t pdu[FAPI_CQI_RAW_MAX_LEN];
} fapi_cqi_indication_raw_pdu_t;

#define FAPI_CQI_IND_MAX_PDU 100
typedef struct __attribute__((packed)) {
	uint32_t number_of_cqis;
	fapi_cqi_indication_pdu_t			cqi_pdu_list[VAR_SIZE];
	//fapi_cqi_indication_raw_pdu_t		cqi_raw_pdu_list[VAR_SIZE];
} fapi_cqi_indication_body_t;

typedef struct __attribute__((packed)) { 
	uint32_t rnti;
	uint32_t preamble;
	uint32_t timing_advance;
	uint32_t power;
} fapi_preamble_pdu_rel8_t;

typedef struct __attribute__((packed)) {
	fapi_preamble_pdu_rel8_t r8;
	uint32_t timing_advance_r9;
} fapi_preamble_pdu_rel9_t;

typedef struct __attribute__((packed)) {
	uint32_t rach_resource_type;
} fapi_preamble_pdu_rel13_t;

typedef union __attribute__((packed)) { 
	fapi_preamble_pdu_rel8_t	preamble_rel8;
	fapi_preamble_pdu_rel9_t	preamble_rel9;
	fapi_preamble_pdu_rel13_t	preamble_rel13;
} fapi_preamble_pdu_t;

#define FAPI_PREAMBLE_MAX_PDU 100
typedef struct __attribute__((packed)) {
	uint32_t number_of_preambles;
	fapi_preamble_pdu_rel9_t	preamble_list[VAR_SIZE];
} fapi_rach_indication_body_t;

#define FAPI_NUM_RB_MAX 100
typedef struct __attribute__((packed)) {
	uint32_t doppler_estimation;
	uint32_t timing_advance;
	uint32_t number_of_resource_blocks;
	uint32_t rb_start;
	uint32_t snr[FAPI_NUM_RB_MAX];
} fapi_srs_indication_fdd_rel8_t;

typedef struct __attribute__((packed)) { 
	uint32_t timing_advance_r9;
} fapi_srs_indication_fdd_rel9_t;

typedef struct __attribute__((packed)) { 
	uint32_t uppts_symbol;
} fapi_srs_indication_ttd_rel10_t;

typedef struct __attribute__((packed)) { 
	uint32_t ul_rtoa;
} fapi_srs_indication_fdd_rel11_t;

typedef struct __attribute__((packed)) {
	uint32_t num_prb_per_subband;
	uint32_t number_of_subbands;
	uint32_t num_atennas;
	struct __attribute__((packed)) {
		uint32_t subband_index;
		uint32_t channel[FAPI_MAX_NUM_PHYSICAL_ANTENNAS];
	} subands[FAPI_MAX_NUM_SUBBANDS];
} fapi_tdd_channel_measurement_t;

typedef struct __attribute__((packed)) {
	fapi_rx_ue_information				rx_ue_information;
	fapi_srs_indication_fdd_rel8_t		srs_indication_fdd_rel8;
	fapi_srs_indication_fdd_rel9_t		srs_indication_fdd_rel9;
	fapi_srs_indication_ttd_rel10_t	srs_indication_tdd_rel10;
	fapi_srs_indication_fdd_rel11_t	srs_indication_fdd_rel11;
	fapi_tdd_channel_measurement_t		tdd_channel_measurement;
} fapi_srs_indication_pdu_t;

#define FAPI_SRS_IND_MAX_PDU 16
typedef struct __attribute__((packed)) {
	uint32_t number_of_ues;
	fapi_srs_indication_pdu_t* srs_pdu_list;
} fapi_srs_indication_body_t;

/* 4-132 */
typedef struct __attribute__((packed)) {
	uint32_t length;
	uint32_t offset;
	uint32_t ul_cqi;
	uint32_t timing_advance;
} fapi_rx_indication_rel8_t;

/* 4-133 */
typedef struct __attribute__((packed)) {
	uint32_t timing_advance_r9;
 } fapi_rx_indication_rel9_t;

 /* 4-131 */
typedef struct __attribute__((packed)) {
	fapi_rx_ue_information rx_ue_information;
	fapi_rx_indication_rel8_t rx_indication_rel8;
	fapi_rx_indication_rel9_t rx_indication_rel9;
	void* data_ptr;
} fapi_rx_indication_pdu_t;

/* 4-131 */
#define FAPI_RX_IND_MAX_PDU 100
typedef struct __attribute__((packed)) {
	uint32_t number_of_pdus;
	fapi_rx_indication_pdu_t rx_pdu_list[VAR_SIZE];
} fapi_rx_indication_body_t;

typedef struct __attribute__((packed)) {
	uint32_t harq_tb1;
} fapi_nb_harq_indication_fdd_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_rx_ue_information					rx_ue_information;
	fapi_nb_harq_indication_fdd_rel13_t	nb_harq_indication_fdd_rel13;
	fapi_ul_cqi_information_t				ul_cqi_information;
} fapi_nb_harq_indication_pdu_t;

typedef struct __attribute__((packed)) {
	uint32_t number_of_harqs;
	fapi_nb_harq_indication_pdu_t* nb_harq_pdu_list;
} fapi_nb_harq_indication_body_t;

typedef struct __attribute__((packed)) {
	uint32_t rnti;
	uint32_t initial_sc;
	uint32_t timing_advance;
	uint32_t nrach_ce_level;
} fapi_nrach_indication_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_nrach_indication_pdu_rel13_t		nrach_indication_rel13;
} fapi_nrach_indication_pdu_t;

typedef struct __attribute__((packed)) {
	uint32_t number_of_initial_scs_detected;
	fapi_nrach_indication_pdu_t* nrach_pdu_list;
} fapi_nrach_indication_body_t;

typedef struct __attribute__((packed)) {
	uint32_t handle;
	uint32_t mp_cca;
	uint32_t n_cca;
	uint32_t offset;
	uint32_t lte_txop_sf;
	uint32_t txop_sfn_sf_end;
	uint32_t lbt_mode;
} fapi_lbt_pdsch_req_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_lbt_pdsch_req_pdu_rel13_t lbt_pdsch_req_pdu_rel13;
} fapi_lbt_pdsch_req_pdu;

typedef struct __attribute__((packed)) {
	uint32_t handle;
	uint32_t offset;
	uint32_t sfn_sf_end;
	uint32_t lbt_mode;
} fapi_lbt_drs_req_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_lbt_drs_req_pdu_rel13_t lbt_drs_req_pdu_rel13;
} fapi_lbt_drs_req_pdu;

typedef struct __attribute__((packed)) {
	uint32_t pdu_type;
	uint32_t pdu_size;
	union {
		fapi_lbt_pdsch_req_pdu		lbt_pdsch_req_pdu;
		fapi_lbt_drs_req_pdu		lbt_drs_req_pdu;
	};
} fapi_lbt_dl_config_request_pdu_t;

#define FAPI_LBT_DL_CONFIG_REQ_MAX_PDU 16
typedef struct __attribute__((packed)) {
	uint32_t number_of_pdus;
	fapi_lbt_dl_config_request_pdu_t*		lbt_dl_config_req_pdu_list;
} fapi_lbt_dl_config_request_body_t;


typedef struct __attribute__((packed)) {
	uint32_t handle;
	uint32_t result;
	uint32_t lte_txop_symbols;
	uint32_t initial_partial_sf;
} fapi_lbt_pdsch_rsp_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_lbt_pdsch_rsp_pdu_rel13_t lbt_pdsch_rsp_pdu_rel13;
} fapi_lbt_pdsch_rsp_pdu;

typedef struct __attribute__((packed)) {
	uint32_t handle;
	uint32_t result;
} fapi_lbt_drs_rsp_pdu_rel13_t;

typedef struct __attribute__((packed)) {
	fapi_lbt_drs_rsp_pdu_rel13_t lbt_drs_rsp_pdu_rel13;
} fapi_lbt_drs_rsp_pdu;


typedef struct __attribute__((packed)) {
	uint32_t pdu_type;
	uint32_t pdu_size;
	union {
		fapi_lbt_pdsch_rsp_pdu		lbt_pdsch_rsp_pdu;
		fapi_lbt_drs_rsp_pdu		lbt_drs_rsp_pdu;
	};
} fapi_lbt_dl_indication_pdu_t;

#define FAPI_LBT_IND_MAX_PDU 16
typedef struct __attribute__((packed)) {
	uint32_t number_of_pdus;
	fapi_lbt_dl_indication_pdu_t* lbt_indication_pdu_list;
} fapi_lbt_dl_indication_body_t;

typedef struct __attribute__((packed)) {
} fapi_error_indication_msg_invalid_state;

typedef struct __attribute__((packed)) {
} fapi_error_indication_msg_bch_missing;

typedef struct __attribute__((packed)) {
	uint32_t recieved_sfn_sf;
	uint32_t expected_sfn_sf;
} fapi_error_indication_sfn_out_of_sync;

typedef struct __attribute__((packed)) {
	uint32_t sub_error_code;
	uint32_t direction;
	uint32_t rnti;
	uint32_t pdu_type;
} fapi_error_indication_msg_pdu_err;

typedef struct __attribute__((packed)) {
	uint32_t recieved_sfn_sf;
	uint32_t expected_sfn_sf;
} fapi_error_indication_msg_invalid_sfn;

typedef struct __attribute__((packed)) {
	uint32_t sub_error_code;
	uint32_t phich_lowest_ul_rb_index;
} fapi_error_indication_msg_hi_err;

typedef struct __attribute__((packed)) {
	uint32_t sub_error_code;
	uint32_t pdu_index;
} fapi_error_indication_msg_tx_err;

#if 0
// 
// P4 Message Structures
//

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t frequency_band_indicator;
	uint32_t measurement_period;
	uint32_t bandwidth;
	uint32_t timeout;
	uint32_t number_of_earfcns;
	uint32_t earfcn[FAPI_MAX_CARRIER_LIST];
} fapi_lte_rssi_request_t;

#define FAPI_LTE_RSSI_REQUEST_TAG 0x3000

#define FAPI_P4_START_TAG FAPI_LTE_RSSI_REQUEST_TAG

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t frequency_band_indicator;
	uint32_t measurement_period;
	uint32_t timeout;
	uint32_t number_of_uarfcns;
	uint32_t uarfcn[FAPI_MAX_CARRIER_LIST];
} fapi_utran_rssi_request_t;

#define FAPI_UTRAN_RSSI_REQUEST_TAG 0x3001

typedef struct __attribute__((packed)) {
	uint32_t arfcn;
	uint32_t direction;
} fapi_arfcn_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t frequency_band_indicator;
	uint32_t measurement_period;
	uint32_t timeout;
	uint32_t number_of_arfcns;
	fapi_arfcn_t arfcn[FAPI_MAX_CARRIER_LIST];
} fapi_geran_rssi_request_t;

#define FAPI_GERAN_RSSI_REQUEST_TAG 0x3002



typedef struct __attribute__((packed)) {
	uint32_t earfcn;
	uint32_t number_of_ro_dl;
	uint32_t ro_dl[FAPI_MAX_RO_DL];
} fapi_earfcn_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t frequency_band_indicator;
	uint32_t measurement_period;
	uint32_t timeout;
	uint32_t number_of_earfcns;
	fapi_earfcn_t earfcn[FAPI_MAX_CARRIER_LIST];
} fapi_nb_iot_rssi_request_t;

#define FAPI_NB_IOT_RSSI_REQUEST_TAG 0x3020

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_rssi;
	int16_t rssi[FAPI_MAX_RSSI];
} fapi_rssi_indication_body_t;

#define FAPI_RSSI_INDICATION_TAG 0x3003

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t earfcn;
	uint32_t measurement_bandwidth;
	uint32_t exhaustive_search;
	uint32_t timeout;
	uint32_t number_of_pci;
	uint32_t pci[FAPI_MAX_PCI_LIST];
} fapi_lte_cell_search_request_t;

#define FAPI_LTE_CELL_SEARCH_REQUEST_TAG 0x3004

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t uarfcn;
	uint32_t exhaustive_search;
	uint32_t timeout;
	uint32_t number_of_psc;
	uint32_t psc[FAPI_MAX_PSC_LIST];
} fapi_utran_cell_search_request_t;

#define FAPI_UTRAN_CELL_SEARCH_REQUEST_TAG 0x3005

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t timeout;
	uint32_t number_of_arfcn;
	uint32_t arfcn[FAPI_MAX_ARFCN_LIST];
} fapi_geran_cell_search_request_t;

#define FAPI_GERAN_CELL_SEARCH_REQUEST_TAG 0x3006

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t earfcn;
	uint32_t ro_dl;
	uint32_t exhaustive_search;
	uint32_t timeout;
	uint32_t number_of_pci;
	uint32_t pci[FAPI_MAX_PCI_LIST];
} fapi_nb_iot_cell_search_request_t;

#define FAPI_NB_IOT_CELL_SEARCH_REQUEST_TAG 0x3021

typedef struct __attribute__((packed)) {
	uint32_t pci;
	uint32_t rsrp;
	uint32_t rsrq;
	int16_t frequency_offset;
} fapi_lte_found_cell_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_lte_cells_found;
	fapi_lte_found_cell_t lte_found_cells[FAPI_MAX_LTE_CELLS_FOUND];
} fapi_lte_cell_search_indication_t;

#define FAPI_LTE_CELL_SEARCH_INDICATION_TAG 0x3007

typedef struct __attribute__((packed)) {
	uint32_t psc;
	uint32_t rscp;
	uint32_t ecno;
	int16_t frequency_offset;
} fapi_utran_found_cell_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_utran_cells_found;
	fapi_utran_found_cell_t utran_found_cells[FAPI_MAX_UTRAN_CELLS_FOUND];
} fapi_utran_cell_search_indication_t;

#define FAPI_UTRAN_CELL_SEARCH_INDICATION_TAG 0x3008

typedef struct __attribute__((packed)) {
	uint32_t arfcn;
	uint32_t bsic;
	uint32_t rxlev;
	uint32_t rxqual;
	int16_t frequency_offset;
	uint32_t sfn_offset;
} fapi_gsm_found_cell_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_gsm_cells_found;
	fapi_gsm_found_cell_t gsm_found_cells[FAPI_MAX_GSM_CELLS_FOUND];
} fapi_geran_cell_search_indication_t;

#define FAPI_GERAN_CELL_SEARCH_INDICATION_TAG 0x3009

typedef struct __attribute__((packed)) {
	uint32_t pci;
	uint32_t rsrp;
	uint32_t rsrq;
	int16_t frequency_offset;
} fapi_nb_iot_found_cell_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_nb_iot_cells_found;
	fapi_nb_iot_found_cell_t nb_iot_found_cells[FAPI_MAX_NB_IOT_CELLS_FOUND];
} fapi_nb_iot_cell_search_indication_t;

#define FAPI_NB_IOT_CELL_SEARCH_INDICATION_TAG 0x3022

typedef fapi_opaqaue_data_t fapi_pnf_cell_search_state_t;

#define FAPI_PNF_CELL_SEARCH_STATE_TAG 0x300A

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t earfcn;
	uint32_t pci;
	uint32_t timeout;
} fapi_lte_broadcast_detect_request_t;

#define FAPI_LTE_BROADCAST_DETECT_REQUEST_TAG 0x300B

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t uarfcn;
	uint32_t psc;
	uint32_t timeout;
} fapi_utran_broadcast_detect_request_t;

#define FAPI_UTRAN_BROADCAST_DETECT_REQUEST_TAG 0x300C

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t earfcn;
	uint32_t ro_dl;
	uint32_t pci;
	uint32_t timeout;
} fapi_nb_iot_broadcast_detect_request_t;

#define FAPI_NB_IOT_BROADCAST_DETECT_REQUEST_TAG 0x3023

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_tx_antenna;
	uint32_t mib_length;
	uint32_t mib[FAPI_MAX_MIB_LENGTH];
	uint32_t sfn_offset;
} fapi_lte_broadcast_detect_indication_t;

#define FAPI_LTE_BROADCAST_DETECT_INDICATION_TAG 0x300E

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t mib_length;
	uint32_t mib[FAPI_MAX_MIB_LENGTH];
	uint32_t sfn_offset;
} fapi_utran_broadcast_detect_indication_t;

#define FAPI_UTRAN_BROADCAST_DETECT_INDICATION_TAG 0x300F


typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t number_of_tx_antenna;
	uint32_t mib_length;
	uint32_t mib[FAPI_MAX_MIB_LENGTH];
	uint32_t sfn_offset;
} fapi_nb_iot_broadcast_detect_indication_t;

#define FAPI_NB_IOT_BROADCAST_DETECT_INDICATION_TAG 0x3024

#define FAPI_PNF_CELL_BROADCAST_STATE_TAG 0x3010

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t earfcn;
	uint32_t pci;
	uint32_t downlink_channel_bandwidth;
	uint32_t phich_configuration;
	uint32_t number_of_tx_antenna;
	uint32_t retry_count;
	uint32_t timeout;
} fapi_lte_system_information_schedule_request_t;

#define FAPI_LTE_SYSTEM_INFORMATION_SCHEDULE_REQUEST_TAG 0x3011


typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t earfcn;
	uint32_t ro_dl;
	uint32_t pci;
	uint32_t scheduling_info_sib1_nb;
	uint32_t timeout;
} fapi_nb_iot_system_information_schedule_request_t;

#define FAPI_NB_IOT_SYSTEM_INFORMATION_SCHEDULE_REQUEST_TAG 0x3025

typedef fapi_opaqaue_data_t fapi_pnf_cell_broadcast_state_t;

typedef struct __attribute__((packed)) {
	uint32_t si_periodicity;
	uint32_t si_index;
} fapi_lte_system_information_si_periodicity_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t earfcn;
	uint32_t pci;
	uint32_t downlink_channel_bandwidth;
	uint32_t phich_configuration;
	uint32_t number_of_tx_antenna;
	uint32_t number_of_si_periodicity;
	fapi_lte_system_information_si_periodicity_t si_periodicity[FAPI_MAX_SI_PERIODICITY];
	uint32_t si_window_length;
	uint32_t timeout;
} fapi_lte_system_information_request_t;

#define FAPI_LTE_SYSTEM_INFORMATION_REQUEST_TAG 0x3014

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t uarfcn;
	uint32_t psc;
	uint32_t timeout;
} fapi_utran_system_information_request_t;

#define FAPI_UTRAN_SYSTEM_INFORMATION_REQUEST_TAG 0x3015

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t arfcn;
	uint32_t bsic;
	uint32_t timeout;
} fapi_geran_system_information_request_t;

#define FAPI_GERAN_SYSTEM_INFORMATION_REQUEST_TAG 0x3016

typedef struct __attribute__((packed)) {
	uint32_t si_periodicity;
	uint32_t si_repetition_pattern;
	uint32_t si_tb_size;
	uint32_t number_of_si_index;
	uint32_t si_index[FAPI_MAX_SI_INDEX];
} fapi_nb_iot_system_information_si_periodicity_t;

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t earfcn;
	uint32_t ro_dl;
	uint32_t pci;
	uint32_t number_of_si_periodicity;
	fapi_nb_iot_system_information_si_periodicity_t si_periodicity[FAPI_MAX_SI_PERIODICITY];
	uint32_t si_window_length;
	uint32_t timeout;
} fapi_nb_iot_system_information_request_t;

#define FAPI_NB_IOT_SYSTEM_INFORMATION_REQUEST_TAG 0x3027

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t sib_type;
	uint32_t sib_length;
	uint32_t sib[FAPI_MAX_SIB_LENGTH];
} fapi_lte_system_information_indication_t;

#define FAPI_LTE_SYSTEM_INFORMATION_INDICATION_TAG 0x3018

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t sib_length;
	uint32_t sib[FAPI_MAX_SIB_LENGTH];
} fapi_utran_system_information_indication_t;

#define FAPI_UTRAN_SYSTEM_INFORMATION_INDICATION_TAG 0x3019

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t si_length;
	uint32_t si[FAPI_MAX_SI_LENGTH];
} fapi_geran_system_information_indication_t;

#define FAPI_GERAN_SYSTEM_INFORMATION_INDICATION_TAG 0x301a

typedef struct __attribute__((packed)) {
	fapi_tl_t tl;
	uint32_t sib_type;
	uint32_t sib_length;
	uint32_t sib[FAPI_MAX_SIB_LENGTH];
} fapi_nb_iot_system_information_indication_t;

#define FAPI_NB_IOT_SYSTEM_INFORMATION_INDICATION_TAG 0x3026
#endif

//
// P7
//

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t sfn_sf;
	uint32_t length;
	fapi_dl_config_request_body_t dl_config_request_body;
} fapi_dl_config_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t sfn_sf;
	uint32_t length;
	fapi_ul_config_request_body_t ul_config_request_body;
} fapi_ul_config_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t sfn_sf;
	fapi_hi_dci0_request_body_t hi_dci0_request_body;
} fapi_hi_dci0_request_t;

/* 4.2.5.1 */
/* 4-125 */
typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t sfn_sf;
	fapi_tx_request_body_t tx_request_body;
} fapi_tx_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	//uint32_t sfn_sf;
	uint32_t frame;
	uint32_t subframe;
} fapi_subframe_indication_t;

/* 4.2.6.2 4-134 */
typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t frame;
	uint32_t subframe;
	fapi_harq_indication_body_t harq_indication_body;
} fapi_harq_indication_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t frame;
	uint32_t subframe;
	fapi_crc_indication_body_t crc_indication_body;
} fapi_crc_indication_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t frame;
	uint32_t subframe;
	fapi_sr_indication_body_t sr_indication_body;
} fapi_sr_indication_t;

typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t frame;
	uint32_t subframe;
	fapi_cqi_indication_body_t cqi_indication_body;
} fapi_cqi_indication_t;

typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t frame;
	uint32_t subframe;
	fapi_rach_indication_body_t rach_indication_body;
} fapi_rach_indication_t;

typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t sfn_sf;
	fapi_srs_indication_body_t srs_indication_body;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_srs_indication_t;

/* 4.2.6.1 4-130 */
typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	//uint32_t sfn_sf;
	uint32_t frame;
	uint32_t subframe;
	fapi_rx_indication_body_t rx_indication_body;
	//fapi_vendor_extension_tlv_t vendor_extension;
} fapi_rx_indication_t;

/* 4.2.6.2 4-134 */
typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t frame;
	uint32_t subframe;
	fapi_nb_harq_indication_body_t nb_harq_indication_body;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_nb_harq_indication_t;

typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t frame;
	uint32_t subframe;
	fapi_nrach_indication_body_t nrach_indication_body;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_nrach_indication_t;

typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t sfn_sf;
	fapi_lbt_dl_config_request_body_t lbt_dl_config_request_body;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_lbt_dl_config_request_t;

typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t sfn_sf;
	fapi_lbt_dl_indication_body_t lbt_dl_indication_body;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_lbt_dl_indication_t;


typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t message_id;
	uint32_t error_code;
	union {
		fapi_error_indication_msg_invalid_state	msg_invalid_state;
		fapi_error_indication_msg_bch_missing		msg_bch_missing;
		fapi_error_indication_sfn_out_of_sync		sfn_out_of_sync;
		fapi_error_indication_msg_pdu_err			msg_pdu_err;
		fapi_error_indication_msg_invalid_sfn		msg_invalid_sfn;
		fapi_error_indication_msg_hi_err			msg_hi_err;
		fapi_error_indication_msg_tx_err			msg_tx_err;
	};
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_error_indication_t;

/* 
 * 
 * P8 PHY DIAGNOSTICS INTERFACE
 * 
 */

/* Дополнительные флаги и параметры */
#define FAPI_P8_DUMP_FLAG_UL               0x00000001
#define FAPI_P8_DUMP_FLAG_UL_ON_BAD_CRC    0x00000002
#define FAPI_P8_DUMP_FLAG_DL               0x00000004
#define FAPI_P8_DUMP_FLAG_RACH             0x00000008
#define FAPI_P8_DUMP_FLAG_UL_PUCCH         0x00000010
#define FAPI_P8_DUMP_FLAG_UL_PUCCH_ACK     0x00000020
#define FAPI_P8_DUMP_FLAG_UL_PUCCH_NACK    0x00000040
#define FAPI_P8_DUMP_FLAG_UL_PUCCH_SR      0x00000080
#define FAPI_P8_DUMP_FLAG_UL_PUCCH_NOT     0x00000100
#define FAPI_P8_DUMP_FLAG_UL_PUCCH_CQI     0x00000200
#define FAPI_P8_DUMP_FLAG_UL_PUCCH_SR_ALL  0x00000400
#define FAPI_P8_DUMP_FLAG_UL_PUSCH_ACK     0x00001000
#define FAPI_P8_DUMP_FLAG_UL_PUSCH_NACK    0x00002000
#define FAPI_P8_DUMP_FLAG_UL_PUSCH_CQI     0x00004000
#define FAPI_P8_DUMP_FLAG_UL_LLR_CHECK     0x80000000

typedef struct __attribute__((packed)){
	uint32_t handle;
	uint32_t type;
	uint32_t offset;
	uint32_t length;
} fapi_p8_ind_pdu_t;

#define MAX_P8_PDU_IND_NUMBER	16
typedef struct __attribute__((packed)){
	fapi_l1_message_header_t header;
	uint32_t frame;
	uint32_t subframe;
	uint32_t number_of_pdus;

	fapi_p8_ind_pdu_t pdus[MAX_P8_PDU_IND_NUMBER];
	uint8_t pdu_data[VAR_SIZE];
} fapi_p8_indication_t;

#if 0
// 
// P4 Messages
// 

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t rat_type;
	union {
		fapi_lte_rssi_request_t					lte_rssi_request;
		fapi_utran_rssi_request_t					utran_rssi_request;
		fapi_geran_rssi_request_t					geran_rssi_request;
		fapi_nb_iot_rssi_request_t					nb_iot_rssi_request;
	};
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_rssi_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_rssi_response_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_rssi_indication_body_t rssi_indication_body;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_rssi_indication_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t rat_type;
	union {
		fapi_lte_cell_search_request_t				lte_cell_search_request;
		fapi_utran_cell_search_request_t			utran_cell_search_request;
		fapi_geran_cell_search_request_t			geran_cell_search_request;
		fapi_nb_iot_cell_search_request_t			nb_iot_cell_search_request;
	};
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_cell_search_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_cell_search_response_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_lte_cell_search_indication_t lte_cell_search_indication;
	fapi_utran_cell_search_indication_t utran_cell_search_indication;
	fapi_geran_cell_search_indication_t geran_cell_search_indication;
	fapi_pnf_cell_search_state_t pnf_cell_search_state;
	fapi_nb_iot_cell_search_indication_t nb_iot_cell_search_indication;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_cell_search_indication_t;


typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t rat_type;
	union {
		fapi_lte_broadcast_detect_request_t		lte_broadcast_detect_request;
		fapi_utran_broadcast_detect_request_t		utran_broadcast_detect_request;
		fapi_nb_iot_broadcast_detect_request_t		nb_iot_broadcast_detect_request;
	};
	fapi_pnf_cell_search_state_t pnf_cell_search_state;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_broadcast_detect_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_broadcast_detect_response_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_lte_broadcast_detect_indication_t lte_broadcast_detect_indication;
	fapi_utran_broadcast_detect_indication_t utran_broadcast_detect_indication;
	fapi_nb_iot_broadcast_detect_indication_t nb_iot_broadcast_detect_indication;
	fapi_pnf_cell_broadcast_state_t pnf_cell_broadcast_state;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_broadcast_detect_indication_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t rat_type;
	union {
		fapi_lte_system_information_schedule_request_t lte_system_information_schedule_request;
		fapi_nb_iot_system_information_schedule_request_t nb_iot_system_information_schedule_request;
	};
	fapi_pnf_cell_broadcast_state_t pnf_cell_broadcast_state;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_system_information_schedule_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_system_information_schedule_response_t;

typedef struct __attribute__((packed)) { 
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_lte_system_information_indication_t lte_system_information_indication;
	fapi_nb_iot_system_information_indication_t nb_iot_system_information_indication;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_system_information_schedule_indication_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t rat_type;
	union {
		fapi_lte_system_information_request_t lte_system_information_request;
		fapi_utran_system_information_request_t utran_system_information_request;
		fapi_geran_system_information_request_t geran_system_information_request;
		fapi_nb_iot_system_information_request_t nb_iot_system_information_request;
	};
	fapi_pnf_cell_broadcast_state_t pnf_cell_broadcast_state;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_system_information_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_system_information_response_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_lte_system_information_indication_t lte_system_information_indication;
	fapi_utran_system_information_indication_t utran_system_information_indication;
	fapi_geran_system_information_indication_t geran_system_information_indication;
	fapi_nb_iot_system_information_indication_t nb_iot_system_information_indication;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_system_information_indication_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_nmm_stop_request_t;

typedef struct __attribute__((packed)) {
	fapi_l1_message_header_t header;
	uint32_t error_code;
	fapi_vendor_extension_tlv_t vendor_extension;
} fapi_nmm_stop_response_t;
#endif

#endif /* _FAPI_INTERFACE_H_ */
