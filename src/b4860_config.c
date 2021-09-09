#include "smartdsp_os.h"
#include "os_init.h"

#include "os_config.h"
#include "smartdsp_os_device.h"
#include "b486x_init.h"
#include "sc39xx_cache.h"
#include "hw_timers32_init.h"
#include "cpri_init.h"
#include "app_config.h"
#include "cpri_b4860.h"

#include "b486x_l1_defense.h"

#ifndef L2_LOCK_ENABLE
#define L2_LOCK_ENABLE 1
#endif

#ifndef L2_PARTITION_ENABLE
#define L2_PARTITION_ENABLE 1
#endif

cpc_init_params_t cpc_params =
{
     /** L3 enable -
      TRUE/FALSE enables/disables L3 cache */
     L3CACHE_ENABLE,
     /** M3 enable -
      TRUE/FALSE enables/disables M3 mode */
     M3_ENABLE,
};

l2_init_params_t l2_init_params =
{
        L2CACHE_ENABLE,   /**< TRUE/FALSE enables/disables L2 cache */
        L2_PARTITION_ENABLE,   /**< TRUE/FALSE enables/disables L2 partition mode */
        L2_LOCK_ENABLE     /**< TRUE/FALSE enables/disables L2 line lock mode */
};


platform_init_params_t platform_params =
{
    /** Gather Enable -
     TRUE/FALSE enables/disables SGB gathering function */
     OS_SGB_ENABLE,
 
    /** Instruction Cache Enable -
     When TRUE, Instruction Cache is enabled. Instruction accesses cache policy
         determined by relevant MMU descriptor.
     When FALSE, Instruction Cache is disabled overriding ICache and L2 cache policy
         to non-cacheable for all instruction accesses */
     ICACHE_ENABLE,
 
    /** Data Cache Enable -
     When TRUE,  Data Cache is enabled. Data accesses cache policy determined
          by relevant MMU descriptor.
     When FALSE, Data Cache is disabled overriding DCache and L2 cache policy
         to non-cacheable for all data accesses */
     DCACHE_ENABLE,
 
    /** Stack Overrun Error Enabled -
     TRUE/FALSE enables/disables precise exceptions as a result of stack related accesses
     that match non-stack descriptors  */
     TRUE,
 
    /** Voluntary Cache Commands Cancel -
     - When TRUE, the MMU and CME cancel the following cache performance commands: DMALLOC, DFETCHx
     - When false, cache performance commands are enabled */
     FALSE,
 
     /** Voulantary Cache Commands Error Enabled -
         When TRUE,
          - generates precise exception on error on granular DFETCHx commands, unless VCCC bit is enabled:
            DFLUSH, DSYNC, DINVALIDATE, DTUNLOCK, DMALLOC, DFETCHx
          When FALSE, the MMU closes the access inside the platform, but does not inform the core of the error */
     FALSE,
 
    /** Memory Protection Enable -
     TRUE/FALSE enables/disables the protection-checking function in all enabled segment descriptors.
     It also enables/disables the miss interrupt support on a miss access */
     TRUE,
 
    /** Address Translation Enable -
     TRUE/FALSE enables/disables the address translation mechanism */
     TRUE,
 
    /** Error Detection Code Exception Enable -
     TRUE/FALSE enables/disables the the ECC exception */
     TRUE
 
};

#if (OS_MIMIC_PA_ARCH == ON)

#include "b486x_heterogeneous.h"
static os_het_smartdsp_log_t smartsdp_log[OS_SOC_MAX_NUM_OF_CORES];

os_het_control_t het_ctrl_params =
{
		HET_START_VALID_VALUE,
        {
                OS_HET_INITIALIZED,     // pa_initialized
                OS_HET_UNINITIALIZED    // sc_initialized
        },
        /*  pa_shared_mem */
        {
                0x50000000,    // start_addr
                0x0f000000     // size
        },
        /*  sc_shared_mem */
        {
        		0x5F000000,     // start_addr
        		0x1000000       // size
        },
        NULL,                           // *ipc
        NULL,                           // L1 defense
        NULL,                           // smartdsp_debug
        {{0,0},{0,0},0,{0,0,0,{0,0},{0},0,{0}}}, //het_debug_print initialization
        0,                              // shared_ctrl_size - SET BY DSP
        0,            // num_ipc_regions
        HET_END_VALID_VALUE
};

#endif // #if (OS_MIMIC_PA_ARCH == ON)


#if ((MAPLE_0 == ON) || (MAPLE_1 == ON) || (MAPLE_2 == ON))
#include "maple_init.h"
maple_init_params_t maple_init_params =
{
    (MAPLE_0 + MAPLE_1 + MAPLE_2),  /**< Num MAPLE devices */
    {
#if (MAPLE_0 == ON)
        {
            MAPLE_0_NAME,      /**< Name of device */
            MAPLE_LW_ID_0,     /**< MAPLE device ID */
            MAPLE_FLG_DEFAULT
        },
#endif
#if (MAPLE_1 == ON)
        {
            MAPLE_1_NAME,      /**< Name of device */
            MAPLE_LW_ID_1,     /**< MAPLE device ID */
            MAPLE_FLG_DEFAULT
        },
#endif
#if (MAPLE_2 == ON)
        {
            MAPLE_2_NAME,      /**< Name of device */
            MAPLE_W_ID_2,     /**< MAPLE device ID */
            MAPLE_FLG_DEFAULT
        }
#endif
    }
};
#endif  // (MAPLE == ON)

#if ((MAPLE_0_FTPE_0 == ON) || (MAPLE_1_FTPE_0 == ON) || (MAPLE_2_FTPE_0 == ON))
#include "maple_ftpe_init.h"

maple_ftpe_init_params_t maple_ftpe_init_params =
{
    (MAPLE_0 * 3) + (MAPLE_1 * 3) + (MAPLE_2 * 2),   /**< Num FTPE devices */
    {
#if (MAPLE_0 == ON)
        {
            MAPLE_0_FTPE_0_NAME,   /**< Name of device */
            FTPE_DEV_ID_0,         /**< FTPE number in MAPLE */
            MAPLE_FTPE_TYPE        /**< Should be with right Maple id */
        },
        {
            MAPLE_0_FTPE_1_NAME,   /**< Name of device */
            FTPE_DEV_ID_1,         /**< FTPE number in MAPLE */
            MAPLE_FTPE_TYPE        /**< Should be with right Maple id */
        },
        {
            MAPLE_0_FTPE_2_NAME,   /**< Name of device */
            FTPE_DEV_ID_2,         /**< FTPE number in MAPLE */
            MAPLE_FTPE_TYPE        /**< Should be with right Maple id */
        },
#endif
#if (MAPLE_1 == ON)
        {
            MAPLE_1_FTPE_0_NAME,   /**< Name of device */
            FTPE_DEV_ID_0,         /**< FTPE number in MAPLE */
            MAPLE_FTPE_TYPE        /**< Should be with right Maple id */
        },
        {
            MAPLE_1_FTPE_1_NAME,   /**< Name of device */
            FTPE_DEV_ID_1,         /**< FTPE number in MAPLE */
            MAPLE_FTPE_TYPE        /**< Should be with right Maple id */
        },
        {
            MAPLE_1_FTPE_2_NAME,   /**< Name of device */
            FTPE_DEV_ID_2,         /**< FTPE number in MAPLE */
            MAPLE_FTPE_TYPE        /**< Should be with right Maple id */
        },
#endif
#if (MAPLE_2 == ON)
        {
            MAPLE_2_FTPE_0_NAME,   /**< Name of device */
            FTPE_DEV_ID_0,         /**< FTPE number in MAPLE */
            MAPLE_FTPE_TYPE        /**< Should be with right Maple id */
        },
        {
            MAPLE_2_FTPE_1_NAME,   /**< Name of device */
            FTPE_DEV_ID_1,         /**< FTPE number in MAPLE */
            MAPLE_FTPE_TYPE        /**< Should be with right Maple id */
        }
#endif
    }
};

#endif  // ((MAPLE_FTPE_0 == ON) || (MAPLE_FTPE_1 == ON) || (MAPLE_FTPE_2 == ON))

#if ((MAPLE_0_PDSCH == ON) || (MAPLE_1_PDSCH == ON))
#include "maple_pdsch_init.h"

maple_pdsch_init_params_t maple_pdsch_init_params =
{
	(MAPLE_0_PDSCH + MAPLE_1_PDSCH),               /**< Num PDSCH devices */
    {
#if (MAPLE_0_PDSCH == ON)
        {
            MAPLE_0_PDSCH_NAME,      /**< Name of device */
            PDSCH_DEV_ID_0,          /**< PDSCH number in MAPLE */
            MAPLE_PDSCH_TYPE         /**< Should be with right Maple id */
        },
#endif
#if (MAPLE_1_PDSCH == ON)
        {
            MAPLE_1_PDSCH_NAME,      /**< Name of device */
            PDSCH_DEV_ID_0,          /**< PDSCH number in MAPLE */
            MAPLE_PDSCH_TYPE         /**< Should be with right Maple id */
        }
#endif
    }
};

#endif  // (MAPLE_0_PDSCH == ON) || (MAPLE_1_PDSCH == ON)

#if ((MAPLE_0_PUSCH == ON) || (MAPLE_1_PUSCH == ON))
#include "maple_pusch_init.h"

maple_pusch_init_params_t maple_pusch_init_params =
{
    1,                      /**< Num PUSCH devices */
    {
        {
            PUSCH_DEV_NAME_0,   /**< Name of device */
            PUSCH_DEV_ID_0,          /**< PUSCH number in MAPLE */
            MAPLE_PUSCH_TYPE
        }
    }
};
#endif

#if ((MAPLE_0_EQPE == ON) || (MAPLE_1_EQPE == ON))
#include "maple_eqpe_init.h"

maple_eqpe_init_params_t maple_eqpe_init_params =
{
    (MAPLE_0_EQPE + MAPLE_1_EQPE),                      /**< Num EQPE devices */
    {
        {
            EQPE_DEV_NAME_0,   /**< Name of device */
            EQPE_DEV_ID_0,     /**< EQPE number in MAPLE */
            MAPLE_EQPE_TYPE
        }
#if MAPLE_1_EQPE == ON
        ,
        {
            EQPE_DEV_NAME_1,   /**< Name of device */
            EQPE_DEV_ID_0,     /**< EQPE number in MAPLE */
            MAPLE_EQPE_TYPE
        }
#endif
    }
};
#endif

#if ((MAPLE_0_PUFFT == ON) || (MAPLE_1_PUFFT == ON) || (MAPLE_2_PUFFT == ON))
#include "maple_pufft_init.h"

maple_pufft_init_params_t maple_pufft_init_params =
{
    (MAPLE_0_PUFFT + MAPLE_1_PUFFT + MAPLE_2_PUFFT),               /**< Num PUFFT devices */
    {
#if (MAPLE_0_PUFFT == ON)
        {
            MAPLE_0_PUFFT_NAME,      /**< Name of device */
            PUFFT_DEV_ID_0,          /**< PUFFT number in MAPLE */
            MAPLE_PUFFT_TYPE         /**< Should be with right Maple id */
        },
#endif
#if (MAPLE_1_PUFFT == ON)
        {
            MAPLE_1_PUFFT_NAME,     /**< Name of device */
            PUFFT_DEV_ID_0,         /**< PUFFT number in MAPLE */
            MAPLE_PUFFT_TYPE        /**< Should be with right Maple id */
        },
#endif
#if (MAPLE_2_PUFFT == ON)
        {
            MAPLE_2_PUFFT_NAME,     /**< Name of device */
            PUFFT_DEV_ID_0,         /**< PUFFT number in MAPLE */
            MAPLE_PUFFT_TYPE        /**< Should be with right Maple id */
        }
#endif
 
    }
};
#endif  // (MAPLE_0_PUFFT == ON) || (MAPLE_1_PUFFT == ON)

#if ((MAPLE_0_TVPE == ON) || (MAPLE_1_TVPE == ON))
#include "maple_tvpe_init.h"

maple_tvpe_init_params_t maple_tvpe_init_params =
{
    NUM_TVPE_USED,               /**< Num eTVPE devices */
    {
#if (MAPLE_0_TVPE == ON)
        {
            MAPLE_0_TVPE_NAME,      /**< Name of device */
            TVPE_DEV_ID_0,          /**< eTVPE number in MAPLE */
            MAPLE_TVPE_TYPE         /**< Should be with right Maple id */
        },
#endif
#if (MAPLE_1_TVPE == ON)
        {
            MAPLE_1_TVPE_NAME,     /**< Name of device */
            TVPE_DEV_ID_0,          /**< eTVPE number in MAPLE */
            MAPLE_TVPE_TYPE         /**< Should be with right Maple id */
        }
#endif
    }
};
#endif  // (MAPLE_0_TVPE == ON) || (MAPLE_1_TVPE == ON)

timer32_group_init_params_t soc_timer32_group0_params =
{
   {
    {
    	SOC_MUX_IN_CPRI0_RX_TRANSACTION,
        TIN_MUX0
    },
    {
    	SOC_MUX_IN_CPRI1_RX_TRANSACTION,
        TIN_MUX1
    },
    {
    	SOC_MUX_IN_CPRI2_RX_TRANSACTION,
        TIN_MUX2
    },
    {
    	SOC_MUX_IN_CPRI3_RX_TRANSACTION,
        TIN_MUX3
    }
    },
    SOC_CLOCK_IN_CPRI_PLL1,
    SOC_TIMER32_GROUP0   /* MODULE NUMBER */
};

timer32_group_init_params_t soc_timer32_group1_params =
{
 {
      SOC_MUX_IN_TMR0_OUT,
      TIN_MUX0
 },
 SOC_CLOCK_IN_PLAFORM2,
 SOC_TIMER32_GROUP1 /* GROUP NUMBER */
};

timer32_group_init_params_t soc_timer32_group2_params =
{
 {
      SOC_MUX_IN_TMR0_OUT,
      TIN_MUX0
 },
 SOC_CLOCK_IN_PLAFORM2,
 SOC_TIMER32_GROUP2 /* GROUP NUMBER */
};

timer32_group_init_params_t soc_timer32_group3_params =
{
 {
      SOC_MUX_IN_TMR0_OUT,
      TIN_MUX0
 },
 SOC_CLOCK_IN_PLAFORM2,
 SOC_TIMER32_GROUP3 /* GROUP NUMBER */
};


#if (B4860_CPRI0 == ON || B4860_CPRI1 == ON || B4860_CPRI2 == ON || B4860_CPRI3 == ON || B4860_CPRI4 == ON || B4860_CPRI5 == ON)

cpri_interrupt_t cpri_interrupts1[] =
{
		/*
    {CPRI_INT_RER,              CPRI_IQ_THRESHOLD_EVENT , OS_HWI_PRIORITY1},
    {CPRI_INT_TER,              CPRI_IQ_THRESHOLD_EVENT , OS_HWI_PRIORITY1},
    */
	    {CPRI_INT_RER,              /*CPRI_IQ_THRESHOLD_EVENT | */CPRI_IQ_FIRST_THRESHOLD_EVENT | CPRI_IQ_SECOND_THRESHOLD_EVENT , OS_HWI_PRIORITY1},
	    {CPRI_INT_TER,              CPRI_IQ_FIRST_THRESHOLD_EVENT | CPRI_IQ_SECOND_THRESHOLD_EVENT , OS_HWI_PRIORITY1},

//#ifdef TEST_RESET_REQUEST
//    {CPRI_INT_ERROR,            CPRI_REMOTE_RESET_ACKNOWLEDGE , OS_HWI_PRIORITY1},
//#endif //TEST_RESET_REQUEST
    
    {CPRI_INT_ERROR,            CPRI_REMOTE_RESET_ACKNOWLEDGE , OS_HWI_PRIORITY1},
    //{CPRI_INT_ERROR,            CPRI_ALL_ERRORS & ~CPRI_LOCAL_LOST_OF_FRAME_ERROR , OS_HWI_PRIORITY1},
    {CPRI_INT_TRANSMIT_CONTROL, CPRI_ETHERNET_EVENT , OS_HWI_PRIORITY1},
    {CPRI_INT_RECEIVE_CONTROL,  CPRI_ETHERNET_EVENT | CPRI_VSS_EVENT, OS_HWI_PRIORITY1},
    {CPRI_INT_LAST, 0}
};

// Hytera 5MHz mapping
mapping_params_t cpri0_mapping_params =
{
    CPRI_BASIC_MODE,                 /**< mapping mode used on the link*/
    CPRI_OVERSAMPLING_RATIO_2,       /**< only for basic mapping - oversampling factor for AxC's on the link*/
    0,                               /**< Only for advanced mapping - Number of Basic Frames in AxC Container Block*/
    NULL,                            /**< Only for advanced mapping - pointer to beginning of the rx mapping table array - length up to 1920 */
    NULL,                            /**< Only for advanced mapping - pointer to beginning of the tx mapping table array - length up to 1920 */
    0,                               /**< Only for advanced mapping - length of tx table up to 1920 */
    0                                /**< Only for advanced mapping - length of rx table up to 1920 */
};

cpri_multicast_params_t cpri0_multicast_params =
{
    CPRI_NORMAL_MODE,           /**< multicast mode  */
    CPRI_IQ_NOT_USED,           /**< uni_multicast mode transaction size */
    0,                          /**< size of rx buffer for second destination*/
};


#if 0
cpri_iq_init_params_t cpri0_iq_init_params =
{
    NUM_OF_CHANNELS_IN_TEST,        /**< number of tx AxC's passing on the link - 1 to 24*/
    NUM_OF_CHANNELS_IN_TEST,        /**< number of rx AxC's passing on the link - 1 to 24*/
    CPRI_CORE0,                     /**< cores that will use IQ data type*/
    CPRI_CORE0,                     /**< core that will initialize the CPRI block*/
    CPRI_AXC_0_ACTIVE,              /**< active tx AxC channels*/
    CPRI_AXC_0_ACTIVE,              /**< active rx AxC channels*/
    CPRI_NOT_ACTIVE,                /**< AxC channels for summing */
    CPRI_NOT_ACTIVE,                /**< Forwarded AxC channels */
    CPRI_NOT_ACTIVE,                /**< active rx AxC DMA channels*/
    CPRI_NOT_ACTIVE,                /**< active rx AxC DMA channels*/
    CPRI_NOT_ACTIVE,                /**< auxiliary mode active */
    CPRI_NOT_ACTIVE,                /**< determines if the transfer of transmit IQ data should be synchronized to other cpri modules. */
    CPRI_NOT_ACTIVE,                /**< determines if the received IQ data should be synchronized to the other cpri. */
    CPRI_NOT_ACTIVE,                /**< Selects if the CPRI pair operates in tx shared mode*/
    CPRI_NOT_ACTIVE,                /**< Selects if the CPRI pair operates in rx shared mode*/
    CPRI_NOT_ACTIVE,                /**< Double Bandwidth Mode */
    CPRI_NOT_ACTIVE,                /**< dual 8 bit summing is disabled */
    &cpri0_mapping_params,          /**< parameters for the mapping used by cpri*/
    &cpri0_multicast_params,        /**< parameters multicast modes cpri*/
    CPRI_SAMPLE_16_BIT,             /**< 15, 16 or 8 bits of i/q sample width */
    TX_TRANSACTION_SIZE,            /**< size of iq bus tx transaction - must be 128 or 64 if TSM is not set*/
    RX_TRANSACTION_SIZE,            /**< size of iq bus rx transaction - must be 128 or 64 if TSM is not set*/
    IQ_RX_BUFFER_SIZE,              /**< size of rx buffer*/
    IQ_TX_BUFFER_SIZE,              /**< size of tx buffer*/
    IQ_TX_BUFFER_SIZE/2,            /**< the location of the first threshold in the AxC tx data buffers*/
    IQ_TX_BUFFER_SIZE,              /**< the location of the second threshold in the AxC tx data buffers*/
    IQ_RX_BUFFER_SIZE/2,            /**< the location of the first threshold in the AxC rx data buffers*/
    IQ_RX_BUFFER_SIZE,              /**< the location of the second threshold in the AxC rx data buffers*/
    IQ_TX_THRESHOLD_SIZE,           /**< the amount of transmitted bytes for a IQ tx event*/
    IQ_RX_THRESHOLD_SIZE,           /**< the amount of received bytes for a IQ rx event*/
    0x0,                            /**< hyper frame number for the start of TX side AxC Container Block*/
    0x0,                            /**< basic frame number for the start of TX side AxC Container Block*/
    0,                              /**< hyper frame number for the start of RX side AxC Container Block*/
    0x0,                            /**< basic frame number for the start of RX side AxC Container Block*/
    0x95,                           /**< Stores the hyper frame number for start of CPRI_TX_START synchronization output*/
    0xBF,                           /**< Stores the basic frame number for start of CPRI_TX_START synchronization output*/
    NULL,                           /**< No Auxiliary mode */
    0,                              /**< Auxiliary table size is irrelevant */
    CPRI_NOT_ACTIVE,                    /**< Generate Receive Symbol Awareness interrupt (expected usage: toward DSP timer) */
    CPRI_NOT_ACTIVE,                    /**< Generate 2nd destination receive Symbol Awareness interrupt (expected usage: toward DSP timer) */
    CPRI_NOT_ACTIVE                     /**< Generate transmit Symbol Awareness interrupt (expected usage: toward DSP timer) */
};
#endif

cpri_iq_init_params_t cpri0_iq_init_params =
{
    1,        /**< number of tx AxC's passing on the link - 1 to 24*/
    1,        /**< number of rx AxC's passing on the link - 1 to 24*/
    CPRI_CORE0,                     /**< cores that will use IQ data type*/
    CPRI_CORE0,                     /**< core that will initialize the CPRI block*/
    CPRI_AXC_0_ACTIVE,              /**< active tx AxC channels*/
    CPRI_AXC_0_ACTIVE,              /**< active rx AxC channels*/
    CPRI_NOT_ACTIVE,                /**< AxC channels for summing */
    CPRI_NOT_ACTIVE,                /**< Forwarded AxC channels */
    CPRI_NOT_ACTIVE,                /**< active rx AxC DMA channels*/
    CPRI_NOT_ACTIVE,                /**< active rx AxC DMA channels*/
    CPRI_NOT_ACTIVE,                /**< auxiliary mode active */
    CPRI_NOT_ACTIVE,                /**< determines if the transfer of transmit IQ data should be synchronized to other cpri modules. */
    CPRI_NOT_ACTIVE,                /**< determines if the received IQ data should be synchronized to the other cpri. */
    CPRI_NOT_ACTIVE,                /**< Selects if the CPRI pair operates in tx shared mode*/
    CPRI_NOT_ACTIVE,                /**< Selects if the CPRI pair operates in rx shared mode*/
    CPRI_NOT_ACTIVE,                /**< Double Bandwidth Mode */
    CPRI_NOT_ACTIVE,                /**< dual 8 bit summing is disabled */
    &cpri0_mapping_params,          /**< parameters for the mapping used by cpri*/
    &cpri0_multicast_params,        /**< parameters multicast modes cpri*/
    CPRI_SAMPLE_15_BIT,             /**< 15, 16 or 8 bits of i/q sample width */
    TX_TRANSACTION_SIZE,            /**< size of iq bus tx transaction - must be 128 or 64 if TSM is not set*/
    RX_TRANSACTION_SIZE,            /**< size of iq bus rx transaction - must be 128 or 64 if TSM is not set*/
    IQ_RX_BUFFER_SIZE_DEFAULT * NUM_CPRI_BUFFERS,              /**< size of rx buffer*/
    IQ_TX_BUFFER_SIZE_DEFAULT * NUM_CPRI_BUFFERS,              /**< size of tx buffer*/
    0,//IQ_TX_BUFFER_SIZE - (TX_TRANSACTION_SIZE * 64),            /**< the location of the first threshold in the AxC tx data buffers*/
    IQ_TX_BUFFER_SIZE_DEFAULT, //IQ_TX_BUFFER_SIZE * 2 - (TX_TRANSACTION_SIZE * 64),              /**< the location of the second threshold in the AxC tx data buffers*/
    0,            /**< the location of the first threshold in the AxC rx data buffers*/
    IQ_RX_BUFFER_SIZE_DEFAULT,              /**< the location of the second threshold in the AxC rx data buffers*/
    256,//IQ_TX_THRESHOLD_SIZE,           /**< the amount of transmitted bytes for a IQ tx event*/
    256,//IQ_RX_THRESHOLD_SIZE,           /**< the amount of received bytes for a IQ rx event*/
    0x0,                            /**< hyper frame number for the start of TX side AxC Container Block*/
    0x0,                            /**< basic frame number for the start of TX side AxC Container Block*/
    0,                              /**< hyper frame number for the start of RX side AxC Container Block*/
    0,                            /**< basic frame number for the start of RX side AxC Container Block*/
    0x95,                           /**< Stores the hyper frame number for start of CPRI_TX_START synchronization output*/
    0xf8,//0xBF,                           /**< Stores the basic frame number for start of CPRI_TX_START synchronization output*/
    NULL,                           /**< No Auxiliary mode */
    0,                              /**< Auxiliary table size is irrelevant */
    CPRI_ACTIVE,                    /**< Generate Receive Symbol Awareness interrupt (expected usage: toward DSP timer) */
    CPRI_ACTIVE,                    /**< Generate 2nd destination receive Symbol Awareness interrupt (expected usage: toward DSP timer) */
    CPRI_ACTIVE                     /**< Generate transmit Symbol Awareness interrupt (expected usage: toward DSP timer) */
};

#if 0
cpri_iq_init_params_t cpri1_iq_init_params =
{
    NUM_OF_CHANNELS_IN_TEST,        /**< number of tx AxC's passing on the link - 1 to 24*/
    NUM_OF_CHANNELS_IN_TEST,        /**< number of rx AxC's passing on the link - 1 to 24*/
    CPRI_CORE1,        /**< cores that will use IQ data type*/
    CPRI_CORE1,                     /**< core that will initialize the CPRI block*/
    CPRI_AXC_0_ACTIVE,              /**< active tx AxC channels*/
    CPRI_AXC_0_ACTIVE,              /**< active rx AxC channels*/
    CPRI_NOT_ACTIVE,                /**< AxC channels for summing */
    CPRI_NOT_ACTIVE,                /**< Forwarded AxC channels */
    CPRI_NOT_ACTIVE,                /**< active rx AxC DMA channels*/
    CPRI_NOT_ACTIVE,                /**< active rx AxC DMA channels*/
    CPRI_NOT_ACTIVE,                /**< auxiliary mode active */
    CPRI_NOT_ACTIVE,                /**< determines if the transfer of transmit IQ data should be synchronized to other cpri modules. */
    CPRI_NOT_ACTIVE,                   /**< determines if the received IQ data should be synchronized to the other cpri. */
    CPRI_NOT_ACTIVE,                /**< Selects if the CPRI pair operates in tx shared mode*/
    CPRI_NOT_ACTIVE,                /**< Selects if the CPRI pair operates in rx shared mode*/
    CPRI_NOT_ACTIVE,                /**< Double Bandwidth Mode */
    CPRI_NOT_ACTIVE,                /**< dual 8 bit summing is disabled */
    &cpri0_mapping_params,          /**< parameters for the mapping used by cpri*/
    &cpri0_multicast_params,        /**< parameters multicast modes cpri*/
    CPRI_SAMPLE_16_BIT,             /**< 15, 16 or 8 bits of i/q sample width*/
    TX_TRANSACTION_SIZE,            /**< size of iq bus tx transaction - must be 128 or 64 if TSM is not set*/
    RX_TRANSACTION_SIZE,            /**< size of iq bus rx transaction - must be 128 or 64 if TSM is not set*/
    IQ_RX_BUFFER_SIZE,              /**< size of rx buffer*/
    IQ_TX_BUFFER_SIZE,              /**< size of tx buffer*/
    IQ_TX_BUFFER_SIZE/2,            /**< the location of the first threshold in the AxC tx data buffers*/
    IQ_TX_BUFFER_SIZE,              /**< the location of the second threshold in the AxC tx data buffers*/
    IQ_RX_BUFFER_SIZE/2,            /**< the location of the first threshold in the AxC rx data buffers*/
    IQ_RX_BUFFER_SIZE,              /**< the location of the second threshold in the AxC rx data buffers*/
    IQ_TX_THRESHOLD_SIZE,                              /**< the amount of transmitted bytes for a IQ tx event*/
    IQ_RX_THRESHOLD_SIZE,                              /**< the amount of received bytes for a IQ rx event*/
    0x0,                           /**< hyper frame number for the start of TX side AxC Container Block*/
    0x0,                           /**< basic frame number for the start of TX side AxC Container Block*/
    0,                              /**< hyper frame number for the start of RX side AxC Container Block*/
    0x0,                            /**< basic frame number for the start of RX side AxC Container Block*/
    0x95,                              /**< Stores the hyper frame number for start of CPRI_TX_START synchronization output*/
    0xBF,                              /**< Stores the basic frame number for start of CPRI_TX_START synchronization output*/
    NULL,                           /**< No Auxiliary mode */
    0,                              /**< Auxiliary table size is irrelevant */
    CPRI_NOT_ACTIVE,                    /**< Generate Receive Symbol Awareness interrupt (expected usage: toward DSP timer) */
    CPRI_NOT_ACTIVE,                    /**< Generate 2nd destination receive Symbol Awareness interrupt (expected usage: toward DSP timer) */
    CPRI_NOT_ACTIVE                     /**< Generate transmit Symbol Awareness interrupt (expected usage: toward DSP timer) */
};
#endif
cpri_ethernet_init_params_t cpri0_ethernet_init_params =
{
     CPRI_CORE0,                     /**< core that will initialize and use the CPRI block (one only)*/
     CPRI_ACTIVE,                    /**<Enable insertion of the Ethernet FCS at the end of the frame. */
     CPRI_NOT_ACTIVE,                    /**<Enable checking of received frames CRC. */
     CPRI_ACTIVE,                    /**<Enables receipt of RX frames >1536 bytes. */
     CPRI_NOT_ACTIVE,                /**<Enables discard of RX frames with illegal preamble nibble before receipt of SFD.. */
     CPRI_ACTIVE,                /**<Enables RX of broadcast packets. */
     CPRI_NOT_ACTIVE,                /**<Enables RX of the subset of multicast Ethernet packets enabled by the hash function. */
     CPRI_NOT_ACTIVE,                    /**<Enables RX MAC address checking. */
     CPRI_NOT_ACTIVE,                /**<Used to select TX store and forward mode */
     CPRI_NOT_ACTIVE,                /**<Enables RX packet length checking */
     0,                              /**<Select the endian mode to use for Ethernet RX and TX data (0 - big, 1- little)  */
     CPRI_ACTIVE,                    /**< If TRUE, enable transmitted frames interrupt for Tx confirmation for this channel. */
     CPRI_ACTIVE,                    /**< If TRUE, flushes frame from cache before transmit (buffers must be in system area only) */
     CPRI_ACTIVE,                    /**< If TRUE, enable transmitted frames interrupt for Rx confirmation for this channel. */
     CPRI_ACTIVE,                    /**< If TRUE, flushes frame from cache before transmit (buffers must be in system area only) */
     CPRI_NOT_ACTIVE,                /**< Ethernet WA mode (A_007968) is disabled.*/
     0,                              /**< On ly in WA mode; steering bits of the VSS TX buffer, which contains both VSS and Ethernet data.*/
     0x56789abc,                     /**<only if rx_mac_address_check enabled - Ethernet address least significant bits*/
     0x1234,                         /**<only if rx_mac_address_check enabled - Ethernet address most significant bits*/
     ETHERNET_RX_BUFFER_SIZE,        /**<size of rx buffer*/
     0,                              /**<relevant only for multicast - hash table to filter multicast traffic */
     ETHERNET_BD_RING_SIZE,          /**<receive ethernet buffer descriptor ring size */
     ETHERNET_BD_RING_SIZE,          /**<Transmit ethernet buffer descriptor ring size */
     0,                              /**< the number of transmitted packets for a ethernet tx event - 1*/
     0,                              /**< the number of received packets for a ethernet tx event - 1*/
     OS_MEM_DDR0_LOCAL_CACHEABLE,             /**< Heap to use for a channel's BDs */
     OS_MEM_DDR0_LOCAL_CACHEABLE              /**< Heap to use for a channel's BDs */
};

cpri_vss_init_params_t cpri0_vss_init_params =
{
    CPRI_CORE0,                      /**< core that will initialize and use the CPRI block (one only)*/
    CPRI_VSS_TRANSACTION_16_BYTES,   /**<size of vss bus tx transaction*/
    CPRI_VSS_TRANSACTION_16_BYTES,   /**<size of vss bus rx transaction*/
    VSS_TX_BUFFER_SIZE,              /**<size of tx buffer*/
    VSS_RX_BUFFER_SIZE,              /**<size of rx buffer*/
    VSS_TX_THRESHOLD_SIZE,           /**< the amount of transmitted bytes for a VSS tx event*/
    VSS_RX_THRESHOLD_SIZE            /**< the amount of received bytes for a VSS rx event*/
};

cpri_init_params_t cpri_init_params_array[NUM_OF_USED_CPRI_UNITS] =
{
    {
#if (defined(USE_CPRI) && (USE_CPRI >= 0 && USE_CPRI <= 7))
        USE_CPRI,                     /**< number of CPRI block to initialize */

#else
        CPRI_0,                     /**< number of CPRI block to initialize */
#endif
        CPRI_NO_LOOPBACK,      /**< loop mode to run CPRI with */
        CPRI_MASTER_MODE,           /**< CPRI master or slave */
        //CPRI_SLAVE_MODE,
        SYNC_GENERATED_LOCALLY,     /**< sync pulse source */
        //SYNC_GENERATED_FROM_OWN_FRAMER,
        //SYNC_GENERATED_EXTERNALLY,
        //SYNC_GENERATED_FROM_PAIR,
        CPRI_SHARED_SYNC_MODE_CPRI1,/**< Dummy value; this field used only in shared mode */
        CPRI_10_HFS_RESET_ACK_ASSERT,/**< Assert reset request ack for 5 HFs */
        CPRI_6_HFS_OUT_OF_10,        /**< Detect reset ack or req when 3 out of 5 HFs are asserted */
        CPRI_NOT_ACTIVE,            /**< rx transparent mode active */
        CPRI_NOT_ACTIVE,            /**< tx transparent mode active */
        CPRI_NOT_ACTIVE,            /**< transmit sync should be an output or not*/
        CPRI_NOT_ACTIVE,//CPRI_ACTIVE,            /**< Sync state machine enable*/
        CPRI_NOT_ACTIVE,            /**< independent(0) / shared(1) sync mode*/
        CPRI_NOT_ACTIVE,            /**< When #external_sync_active is set to #SYNC_GENERATED_EXTERNALLY, define sync direction to positive (active) or negative (not active) */
        CPRI_NOT_ACTIVE,            /**< if active the 10ms sync pulse comes from block 1588. */
        CPRI_NOT_ACTIVE,            /**< whether this CPRI unit participates in external remote-reset indication */
        0,                          /**< number of sync errors to hunt state*/
        0x10,                       /**< Transmit CPRI Framer Buffer size. 0 -> default size*/
        0x14,                       /**< Used to select Ethernet/VSS portion in cpri frame, 0x14 is maximal Ethernet, 0x3f is maximal VSS*/
        0,                          /**< additional transmit delay (for delay measurement)*/
        0x0,                        /**< Seed for scrambling. 0 if scrambling is not desired */
        0x3,                        /**< Timer tolerance value */
        0,                          /**< TX delay to configure on the link*/
        0,                          /**< TX delay to configure on the link*/
        &cpri_interrupts1[0],        /**< pointer to beginning of cpri_interrupts array */
        NULL,                       /**< init extended features */
        CPRI_IQ_ENABLED,            /**< enable/disable IQ path - must be enabled!*/
        CPRI_VSS_ENABLED,          /**< enable/disable VSS path*/
        CPRI_ETHERNET_ENABLED,      /**< enable/disable Ethernet path*/
        CPRI_HDLC_DISABLED,         /**< enable/disable HDLC path*/
        &cpri0_iq_init_params,      /**< pointer to iq_init_params - must not be NULL!*/
        &cpri0_vss_init_params,                       /**< pointer to vss_init_params*/
        &cpri0_ethernet_init_params, /**< pointer to ethernet_init_params*/
        NULL                        /**< pointer to iq_init_params*/
    },
#if (NUM_OF_USED_CPRI_UNITS > 1)
    {
#if (defined(USE_CPRI) && (USE_CPRI >= 0 && USE_CPRI <= 7))
        //USE_CPRI,                     /**< number of CPRI block to initialize */
    	CPRI_1,
#else
#error "Invalid CPRI number, use USE_CPRI=<0..7>"
#endif
        CPRI_NO_LOOPBACK,      /**< loop mode to run CPRI with */
        CPRI_MASTER_MODE,           /**< CPRI master or slave */
        //CPRI_SLAVE_MODE,
        SYNC_GENERATED_LOCALLY,     /**< sync pulse source */
        //SYNC_GENERATED_FROM_OWN_FRAMER,
        //SYNC_GENERATED_EXTERNALLY,
        //SYNC_GENERATED_FROM_PAIR,
        CPRI_SHARED_SYNC_MODE_CPRI1,/**< Dummy value; this field used only in shared mode */
        CPRI_10_HFS_RESET_ACK_ASSERT,/**< Assert reset request ack for 5 HFs */
        CPRI_6_HFS_OUT_OF_10,        /**< Detect reset ack or req when 3 out of 5 HFs are asserted */
        CPRI_NOT_ACTIVE,            /**< rx transparent mode active */
        CPRI_NOT_ACTIVE,            /**< tx transparent mode active */
        CPRI_NOT_ACTIVE,            /**< transmit sync should be an output or not*/
        CPRI_NOT_ACTIVE,//CPRI_ACTIVE,            /**< Sync state machine enable*/
        CPRI_NOT_ACTIVE,            /**< independent(0) / shared(1) sync mode*/
        CPRI_NOT_ACTIVE,            /**< When #external_sync_active is set to #SYNC_GENERATED_EXTERNALLY, define sync direction to positive (active) or negative (not active) */
        CPRI_NOT_ACTIVE,            /**< if active the 10ms sync pulse comes from block 1588. */
        CPRI_NOT_ACTIVE,            /**< whether this CPRI unit participates in external remote-reset indication */
        0,                          /**< number of sync errors to hunt state*/
        0x10,                       /**< Transmit CPRI Framer Buffer size. 0 -> default size*/
        0x14,                       /**< Used to select Ethernet/VSS portion in cpri frame, 0x14 is maximal Ethernet, 0x3f is maximal VSS*/
        0,                          /**< additional transmit delay (for delay measurement)*/
        0x0,                        /**< Seed for scrambling. 0 if scrambling is not desired */
        0x3,                        /**< Timer tolerance value */
        0,                          /**< TX delay to configure on the link*/
        0,                          /**< TX delay to configure on the link*/
        &cpri_interrupts1[0],        /**< pointer to beginning of cpri_interrupts array */
        NULL,                       /**< init extended features */
        CPRI_IQ_ENABLED,            /**< enable/disable IQ path - must be enabled!*/
        CPRI_VSS_ENABLED,          /**< enable/disable VSS path*/
        CPRI_ETHERNET_ENABLED,      /**< enable/disable Ethernet path*/
        CPRI_HDLC_DISABLED,         /**< enable/disable HDLC path*/
        &cpri0_iq_init_params,      /**< pointer to iq_init_params - must not be NULL!*/
        &cpri0_vss_init_params,                       /**< pointer to vss_init_params*/
        &cpri0_ethernet_init_params, /**< pointer to ethernet_init_params*/
        NULL                        /**< pointer to iq_init_params*/
    },
#if (NUM_OF_USED_CPRI_UNITS > 2)
    {
#if (defined(USE_CPRI) && (USE_CPRI >= 0 && USE_CPRI <= 7))
        //USE_CPRI,                     /**< number of CPRI block to initialize */
    	CPRI_2,
#else
#error "Invalid CPRI number, use USE_CPRI=<0..7>"
#endif
        CPRI_NO_LOOPBACK,      /**< loop mode to run CPRI with */
        CPRI_MASTER_MODE,           /**< CPRI master or slave */
        //CPRI_SLAVE_MODE,
        SYNC_GENERATED_LOCALLY,     /**< sync pulse source */
        //SYNC_GENERATED_FROM_OWN_FRAMER,
        //SYNC_GENERATED_EXTERNALLY,
        //SYNC_GENERATED_FROM_PAIR,
        CPRI_SHARED_SYNC_MODE_CPRI1,/**< Dummy value; this field used only in shared mode */
        CPRI_10_HFS_RESET_ACK_ASSERT,/**< Assert reset request ack for 5 HFs */
        CPRI_6_HFS_OUT_OF_10,        /**< Detect reset ack or req when 3 out of 5 HFs are asserted */
        CPRI_NOT_ACTIVE,            /**< rx transparent mode active */
        CPRI_NOT_ACTIVE,            /**< tx transparent mode active */
        CPRI_NOT_ACTIVE,            /**< transmit sync should be an output or not*/
        CPRI_NOT_ACTIVE,//CPRI_ACTIVE,            /**< Sync state machine enable*/
        CPRI_NOT_ACTIVE,            /**< independent(0) / shared(1) sync mode*/
        CPRI_NOT_ACTIVE,            /**< When #external_sync_active is set to #SYNC_GENERATED_EXTERNALLY, define sync direction to positive (active) or negative (not active) */
        CPRI_NOT_ACTIVE,            /**< if active the 10ms sync pulse comes from block 1588. */
        CPRI_NOT_ACTIVE,            /**< whether this CPRI unit participates in external remote-reset indication */
        0,                          /**< number of sync errors to hunt state*/
        0x10,                       /**< Transmit CPRI Framer Buffer size. 0 -> default size*/
        0x14,                       /**< Used to select Ethernet/VSS portion in cpri frame, 0x14 is maximal Ethernet, 0x3f is maximal VSS*/
        0,                          /**< additional transmit delay (for delay measurement)*/
        0x0,                        /**< Seed for scrambling. 0 if scrambling is not desired */
        0x3,                        /**< Timer tolerance value */
        0,                          /**< TX delay to configure on the link*/
        0,                          /**< TX delay to configure on the link*/
        &cpri_interrupts1[0],        /**< pointer to beginning of cpri_interrupts array */
        NULL,                       /**< init extended features */
        CPRI_IQ_ENABLED,            /**< enable/disable IQ path - must be enabled!*/
        CPRI_VSS_ENABLED,          /**< enable/disable VSS path*/
        CPRI_ETHERNET_ENABLED,      /**< enable/disable Ethernet path*/
        CPRI_HDLC_DISABLED,         /**< enable/disable HDLC path*/
        &cpri0_iq_init_params,      /**< pointer to iq_init_params - must not be NULL!*/
        &cpri0_vss_init_params,                       /**< pointer to vss_init_params*/
        &cpri0_ethernet_init_params, /**< pointer to ethernet_init_params*/
        NULL                        /**< pointer to iq_init_params*/
    },
#if (NUM_OF_USED_CPRI_UNITS > 3)
    {
#if (defined(USE_CPRI) && (USE_CPRI >= 0 && USE_CPRI <= 7))
        //USE_CPRI,                     /**< number of CPRI block to initialize */
    	CPRI_3,
#else
#error "Invalid CPRI number, use USE_CPRI=<0..7>"
#endif
        CPRI_NO_LOOPBACK,      /**< loop mode to run CPRI with */
        CPRI_MASTER_MODE,           /**< CPRI master or slave */
        //CPRI_SLAVE_MODE,
        SYNC_GENERATED_LOCALLY,     /**< sync pulse source */
        //SYNC_GENERATED_FROM_OWN_FRAMER,
        //SYNC_GENERATED_EXTERNALLY,
        //SYNC_GENERATED_FROM_PAIR,
        CPRI_SHARED_SYNC_MODE_CPRI1,/**< Dummy value; this field used only in shared mode */
        CPRI_10_HFS_RESET_ACK_ASSERT,/**< Assert reset request ack for 5 HFs */
        CPRI_6_HFS_OUT_OF_10,        /**< Detect reset ack or req when 3 out of 5 HFs are asserted */
        CPRI_NOT_ACTIVE,            /**< rx transparent mode active */
        CPRI_NOT_ACTIVE,            /**< tx transparent mode active */
        CPRI_NOT_ACTIVE,            /**< transmit sync should be an output or not*/
        CPRI_NOT_ACTIVE,//CPRI_ACTIVE,            /**< Sync state machine enable*/
        CPRI_NOT_ACTIVE,            /**< independent(0) / shared(1) sync mode*/
        CPRI_NOT_ACTIVE,            /**< When #external_sync_active is set to #SYNC_GENERATED_EXTERNALLY, define sync direction to positive (active) or negative (not active) */
        CPRI_NOT_ACTIVE,            /**< if active the 10ms sync pulse comes from block 1588. */
        CPRI_NOT_ACTIVE,            /**< whether this CPRI unit participates in external remote-reset indication */
        0,                          /**< number of sync errors to hunt state*/
        0x10,                       /**< Transmit CPRI Framer Buffer size. 0 -> default size*/
        0x14,                       /**< Used to select Ethernet/VSS portion in cpri frame, 0x14 is maximal Ethernet, 0x3f is maximal VSS*/
        0,                          /**< additional transmit delay (for delay measurement)*/
        0x0,                        /**< Seed for scrambling. 0 if scrambling is not desired */
        0x3,                        /**< Timer tolerance value */
        0,                          /**< TX delay to configure on the link*/
        0,                          /**< TX delay to configure on the link*/
        &cpri_interrupts1[0],        /**< pointer to beginning of cpri_interrupts array */
        NULL,                       /**< init extended features */
        CPRI_IQ_ENABLED,            /**< enable/disable IQ path - must be enabled!*/
        CPRI_VSS_ENABLED,          /**< enable/disable VSS path*/
        CPRI_ETHERNET_ENABLED,      /**< enable/disable Ethernet path*/
        CPRI_HDLC_DISABLED,         /**< enable/disable HDLC path*/
        &cpri0_iq_init_params,      /**< pointer to iq_init_params - must not be NULL!*/
        &cpri0_vss_init_params,                       /**< pointer to vss_init_params*/
        &cpri0_ethernet_init_params, /**< pointer to ethernet_init_params*/
        NULL                        /**< pointer to iq_init_params*/
    },
#if (NUM_USED_CPRI_UNITS > 4)
#error "More than 4 CPRI units not supported!"
#endif
#endif
#endif
#endif
};
 
cpri_init_params_t (*cpri_init_params)[NUM_OF_USED_CPRI_UNITS] = &cpri_init_params_array;

#if 0
cpri_global_init_params_t cpri_global_params_struct =
{
     CPRI_CORE0,                    /**<initializing_core*/
     NUM_OF_USED_CPRI_UNITS,        /**<number of active cpri units*/
     {{CPRI_2457_LINK_RATE,         /**<maximal link rate.*/
    		 CPRI_2457_LINK_RATE},        /**<minimal link rate accepted without failure*/
    		 {CPRI_2457_LINK_RATE,         /**<maximal link rate.*/
    		     		 CPRI_2457_LINK_RATE},        /**<minimal link rate accepted without failure*/
           }       /**<minimal link rate accepted without failure*/
#if 0
     {{CPRI_4914_LINK_RATE,         /**<maximal link rate.*/
       CPRI_4914_LINK_RATE},        /**<minimal link rate accepted without failure*/
       {CPRI_4914_LINK_RATE,        /**<maximal link rate.*/
        CPRI_4914_LINK_RATE}}       /**<minimal link rate accepted without failure*/
#endif
};
#else

cpri_global_init_params_t cpri_global_params_struct =
{
     CPRI_CORE0, /**<initializing_core*/
     1, //1, /**<number of active cpri units*/
#if 1
     {
    		 {
    				 CPRI_2457_LINK_RATE,         /**<maximal link rate.*/
    				 CPRI_2457_LINK_RATE        /**<minimal link rate accepted without failure*/
    		 },
    		 {
    				 CPRI_2457_LINK_RATE,         /**<maximal link rate.*/
    		 		 CPRI_2457_LINK_RATE        /**<minimal link rate accepted without failure*/
    		 },
     }       /**<minimal link rate accepted without failure*/
#else
     {
    		 {
    			 CPRI_4914_LINK_RATE,         /**<maximal link rate.*/
    			 CPRI_4914_LINK_RATE        /**<minimal link rate accepted without failure*/
    		 },
    		 {
    				 CPRI_4914_LINK_RATE,         /**<maximal link rate.*/
    				 CPRI_4914_LINK_RATE        /**<minimal link rate accepted without failure*/
    		 },
     }       /**<minimal link rate accepted without failure*/

#endif
};

#endif

cpri_global_init_params_t *cpri_global_params = &cpri_global_params_struct ;

#endif // (B4860_CPRI0 == ON || B4860_CPRI1 == ON || B4860_CPRI2 == ON || B4860_CPRI3 == ON || B4860_CPRI4 == ON || B4860_CPRI5 == ON)

#if (OS_L1_DEFENSE == ON)
l1d_init_params_t soc_l1d_init_params =
{
     MODE_1_ACTIVE | MODE_2_ACTIVE | MODE_3_ACTIVE, /**<enabled L1 defense modes*/
#if defined(B4860)
     {OS_INT_VIRQ26, OS_INT_VIRQ27, OS_INT_VIRQ28, OS_INT_VIRQ29, OS_INT_VIRQ30, OS_INT_VIRQ31}  /**<interrupt ID to be used for invoking of L1 defense */
#elif defined(B4420)
     {OS_INT_VIRQ26, OS_INT_VIRQ27}  /**<interrupt ID to be used for invoking of L1 defense */
#else //B4420
#error "B4860 or B4420 must be defined"
#endif
};
#endif //B4860_L1_DEFENSE

