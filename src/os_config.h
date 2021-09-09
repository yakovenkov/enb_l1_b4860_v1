#ifndef __OS_CONFIG_H
#define __OS_CONFIG_H

#include "smartdsp_os_device.h"


#define ON      1
#define OFF     0

/* OS Power architecture simulation */
#define B4860_SC_ONLY_BSP                   OFF   /**< ON - Work as if there is no PA, OFF - PA is in the system or needs to be mimicked */
#define OS_MIMIC_PA_ARCH                    OFF    /**< ON - mimic PA, OFF - don't mimic PA */

#if (B4860_SC_ONLY_BSP == OFF)
/* Variables from linker file l3k; For SC only mode there is no need in it  */
extern uint32_t _VirtSHARED_CTRL_b;
extern uint32_t _SHARED_CTRL_size;
extern uint32_t _Virt_PA_HetShared_b;
extern uint32_t _Virt_SC_HetShared_b;
#define SOC_HET_CTRL_BASE                   ((uint32_t)&_VirtSHARED_CTRL_b)     /**< Heterogeneous shared control virtual base address */
#define SOC_HET_PA_BASE                     ((uint32_t)&_Virt_PA_HetShared_b)   /**< Virtual base that will be used for PA shared heterogeneous heap */
#define SOC_HET_SC_BASE                     ((uint32_t)&_Virt_SC_HetShared_b)   /**< Virtual base that will be used for PA shared heterogeneous heap */
#define SOC_HET_CTRL_SIZE                   ((uint32_t)&_SHARED_CTRL_size)
#define OS_MEM_HET_DDR0_CACHEABLE_SIZE      0x1000
#endif

/* OS General Configuration **************************************************/

//#define OS_HEAP_CACHEABLE_SIZE                        0xD0000 /* Heap size */
#define OS_HEAP_CACHEABLE_SIZE                        0x10000 /* Heap size */

#define OS_STACK_SIZE                       0x10000
//#define OS_STACK_SIZE                       0x1000

#define OS_HEAP_NONCACHEABLE_SIZE           0x4000  /* Local non-cacheable heap, must be power 2 and >= 256 */

//#define OS_BACKGROUND_STACK_SIZE            0x40000
#define OS_BACKGROUND_STACK_SIZE            0x4000

#define OS_HEAP_DDR0_CACHEABLE_SIZE                   0x400000
//#define OS_HEAP_DDR0_CACHEABLE_SIZE                   0x4000

#define OS_HEAP_DDR0_NOCACHEABLE_SIZE       0x1000

#if (OS_MULTICORE == ON)
#define OS_SHARED_MEM_DDR0_NONCACHEABLE_SIZE             0x100000
#define OS_SHARED_MEM_DDR0_CACHEABLE_SIZE   0x10000000
#define OS_SHARED_CACHEABLE_MEM_SIZE                  0x100000    /* Shared Memory Size */
#endif

#define OS_LOCAL_HEAP_MNGMNT_SIZE           6500
#if OS_MULTICORE == 1
#define OS_SHARED_HEAP_MNGMNT_SIZE          6000
#endif

#if 0
#define OS_HEAP_M3_CACHEABLE_SIZE          0x1000
#define OS_HEAP_M3_NOCACHEABLE_SIZE        0x1000
#if (OS_MULTICORE == ON)
#define OS_SHARED_MEM_M3_NONCACHEABLE_SIZE 0x3000
#define OS_SHARED_MEM_M3_CACHEABLE_SIZE    0x2400
#endif
#endif

/** IPC ***/
#define	SOC_IPC								ON

#define OS_TICK                             ON /* Tick Functionality */
#define OS_TICK_PRIORITY                    OS_SWI_PRIORITY0    /* Tick Priority */
#define OS_TICK_PARAMETER                   SOC_TICK_001MS  /* Tick Parameter */

#define OS_CLKIN                            66.666667 /* (MHz) */
#define OS_SYSTEM_CLOCK                     OS_CLKIN /* provided for backward compatibility */

#define OS_HW_TIMERS                        ON /* Hardware Timers */
#define OS_TOTAL_NUM_OF_SW_TIMERS           4   /* Software Timers Number */

#define OS_TOTAL_NUM_OF_MEM_PARTS           32   /* Memory Partitions */
#define OS_TOTAL_NUM_OF_FRAME_POOLS         32   /* Frame Pools Number */



#define OS_TOTAL_NUM_OF_SWI                 32   /* Software Interrupts Number */

#define OS_TOTAL_NUM_OF_EVENT_QUEUES        64 /**< Event Queues Number */
#define OS_TOTAL_NUM_OF_QUEUES              (OS_TOTAL_NUM_OF_EVENT_QUEUES)  /**< Queues Number */
//#define OS_TOTAL_NUM_OF_QUEUES              10   /* Queues Number */

#define OS_TOTAL_NUM_OF_EVENT_SEMAPHORES    32  /**< Event Semaphores Number */

#define OS_TOTAL_NUM_OF_TASKS               64  /**< Tasks Number */
//#define OS_TOTAL_NUM_OF_TASKS               3

#if (OS_MULTICORE == ON)
#define OS_TOTAL_NUM_OF_SHARED_QUEUES       (OS_TOTAL_NUM_OF_QUEUES)    /**< Shared Queues Number */
#else
#define OS_TOTAL_NUM_OF_SHARED_QUEUES       0  /* Shared Queues Number */
#endif

#define OS_TOTAL_NUM_OF_MESSAGE_QUEUES      0

#define OS_WATCHDOGS                        OFF
#define OS_WATCHDOG_TIMEOUT                 0x0400
#define OS_WATCHDOG_PRIORITY                OS_SWI_PRIORITY15

#ifdef USER_WATCHDOG
    #define OS_WATCHDOG_SUPPORT             OS_WDT_USER_HANDLING
#else
    #define OS_WATCHDOG_SUPPORT             OS_WDT_AUTO_HANDLING
    #define OS_WATCHDOG_BEHAVIOR            DSP_WATCHDOG_DEFAULT
    #define OS_WATCHDOG_HANDLER             appWdtHandler
#endif

/* L1 Defense ****************************************************************/
#define B4860_L1_DEFENSE					ON
#define OS_L1_DEFENSE                       B4860_L1_DEFENSE
#define OS_MAX_NUM_OF_SPINLOCKS				500

/* MPIC **********************************************************************/
#define OS_SOC_MPIC_DSP_MASTER              OFF
#define OS_SOC_MPIC_PPC_MASTER              ON

/* SOC_HW_TIMER32 Devices ***************************************************************/
#define TIMER32_GROUP0             ON
#define TIMER32_GROUP1             ON
#define TIMER32_GROUP2             ON
#define TIMER32_GROUP3             ON
#define TIMER32_GROUP4             OFF
#define TIMER32_GROUP5             OFF
#define TIMER32_GROUP6             OFF
#define TIMER32_GROUP7             OFF

/* MULTICORE *****************************************************************/

/* OS_MULTICORE should be defined as 0 or 1 by the compiler */

#define OS_MULTICORE_SYNCHRONIZATION        ON  /* Multi Core Synchronization */
//#define OS_NUM_OF_CORES                     OS_SOC_MAX_NUM_OF_CORES /* Cores Number */
//#define OS_MAX_NUM_OF_CORES                 OS_SOC_MAX_NUM_OF_CORES /* Max Cores Number */
#define OS_NUM_OF_CORES                     1 /* Cores Number */
#define OS_MAX_NUM_OF_CORES                 1 /* Max Cores Number */
#define OS_MASTER_CORE                      0   /* Master Core ID */

/* Architecture Configuration ***************************************************/
/* Caution: For non-cacheable mode please modify MMU segments attribute as well */
#define MMU_DATA_CONTEXT_NUM                10
#define MMU_PROG_CONTEXT_NUM                10
#define OS_SGB_ENABLE                       ON
#define DCACHE_ENABLE                       ON  /* Enable DCACHE */
#define ICACHE_ENABLE                       ON  /* Enable ICACHE */
#define L2CACHE_ENABLE                      ON  /* Enable L2 CACHE */
#ifdef B4420
#define L3CACHE_ENABLE                      OFF
#define M3_ENABLE                           OFF
#else
#define L3CACHE_ENABLE                      ON
#define M3_ENABLE                           OFF
#endif

#ifdef B4860
#define B4860_ON    ON
#else //B4420
#define B4860_ON    OFF
#endif

/* Drivers no HAL ************************************************************/

/* CIO Devices ***************************************************************/

#define B4860_I2C1                         OFF
#define B4860_I2C2                         OFF
#define B4860_I2C3                         OFF
#define B4860_I2C4                         OFF


/* B4860QDS */
#ifdef B4860
#define B4860_CPRI0                       ON
#define B4860_CPRI1                       OFF
#define B4860_CPRI2                       OFF
#define B4860_CPRI3                       OFF
#elif defined(B4420)
#define B4860_CPRI0                       OFF
#define B4860_CPRI1                       OFF
#define B4860_CPRI2                       OFF
#define B4860_CPRI3                       OFF
#endif //B4860
#define B4860_CPRI4                       OFF
#define B4860_CPRI5                       OFF
#define CPRI_INITIALIZING_CORE            CPRI_CORE0

/* BIO Devices ***************************************************************/
#define B4860_CPRI0_ETHERNET              ON      /* ETHERNET of CPRI 0 */
#define B4860_CPRI1_ETHERNET              OFF      /* ETHERNET of CPRI 1 */
#define B4860_CPRI2_ETHERNET              OFF      /* ETHERNET of CPRI 2 */
#define B4860_CPRI3_ETHERNET              OFF      /* ETHERNET of CPRI 3 */
#define B4860_CPRI4_ETHERNET              OFF      /* ETHERNET of CPRI 4 */
#define B4860_CPRI5_ETHERNET              OFF      /* ETHERNET of CPRI 5 */
#define B4860_CPRI6_ETHERNET              OFF      /* ETHERNET of CPRI 6 */
#define B4860_CPRI7_ETHERNET              OFF      /* ETHERNET of CPRI 7 */

#define B4860_CPRI0_HDLC                  OFF     /* HDLC of CPRI 0 */
#define B4860_CPRI1_HDLC                  OFF     /* HDLC of CPRI 1 */
#define B4860_CPRI2_HDLC                  OFF     /* HDLC of CPRI 2 */
#define B4860_CPRI3_HDLC                  OFF     /* HDLC of CPRI 3 */
#define B4860_CPRI4_HDLC                  OFF     /* HDLC of CPRI 4 */
#define B4860_CPRI5_HDLC                  OFF     /* HDLC of CPRI 5 */
#define B4860_CPRI6_HDLC                  OFF     /* HDLC of CPRI 6 */
#define B4860_CPRI7_HDLC                  OFF     /* HDLC of CPRI 7 */

/* SIO Devices ***************************************************************/
#define B4860_CPRI0_IQ                    ON /* IQ of CPRI 0 - must be ON if CPRI block is used */
#define B4860_CPRI1_IQ                    OFF /* IQ of CPRI 1 - must be ON if CPRI block is used */
#define B4860_CPRI2_IQ                    OFF /* IQ of CPRI 2 - must be ON if CPRI block is used */
#define B4860_CPRI3_IQ                    OFF /* IQ of CPRI 3 - must be ON if CPRI block is used */
#define B4860_CPRI4_IQ                    OFF /* IQ of CPRI 4 - must be ON if CPRI block is used */
#define B4860_CPRI5_IQ                    OFF /* IQ of CPRI 5 - must be ON if CPRI block is used */
#define B4860_CPRI6_IQ                    OFF /* IQ of CPRI 5 - must be ON if CPRI block is used */
#define B4860_CPRI7_IQ                    OFF /* IQ of CPRI 5 - must be ON if CPRI block is used */
#define B4860_CPRI0_VSS                   ON /* VSS of CPRI 0 */
#define B4860_CPRI1_VSS                   OFF /* VSS of CPRI 1 */
#define B4860_CPRI2_VSS                   OFF /* VSS of CPRI 2 */
#define B4860_CPRI3_VSS                   OFF /* VSS of CPRI 3 */
#define B4860_CPRI4_VSS                   OFF /* VSS of CPRI 4 */
#define B4860_CPRI5_VSS                   OFF /* VSS of CPRI 5 */
#define B4860_CPRI6_VSS                   OFF /* VSS of CPRI 5 */
#define B4860_CPRI7_VSS                   OFF /* VSS of CPRI 5 */

/* COP Devices ***************************************************************/
#define MAPLE_POWER_CLOCK_REDUCE            ON

#define MAPLE_0                             ON   /**< Using Maple Device */
//#define MAPLE_1                             B4860_ON  /**< Using Maple Device */
//#define MAPLE_2                             B4860_ON  /**< Using Maple Device */
#define MAPLE_1                             OFF  /**< Using Maple Device */
#define MAPLE_2                             OFF  /**< Using Maple Device */

#define MAPLE_0_TRACE                       OFF   /**< Using Maple Trace On Device */
#define MAPLE_1_TRACE                       OFF  /**< Using Maple Trace On Device */
#define MAPLE_2_TRACE                       OFF  /**< Using Maple Trace On Device */

#define MAPLE_0_FTPE_0                      ON  /**< Using Maple 0 eFTPE 0 Device */
#define MAPLE_0_FTPE_1                      ON  /**< Using Maple 0 eFTPE 1 Device */
#define MAPLE_0_FTPE_2                      ON  /**< Using Maple 0 eFTPE 2 Device */
#define MAPLE_1_FTPE_0                      OFF  /**< Using Maple 1 eFTPE 0 Device */
#define MAPLE_1_FTPE_1                      OFF  /**< Using Maple 1 eFTPE 1 Device */
#define MAPLE_1_FTPE_2                      OFF  /**< Using Maple 1 eFTPE 2 Device */
#define MAPLE_2_FTPE_0                      OFF  /**< Using Maple 2 eFTPE 0 Device */
#define MAPLE_2_FTPE_1                      OFF  /**< Using Maple 2 eFTPE 1 Device */

#define MAPLE_0_DEPE                        OFF  /**< Using Maple 0 DEPE Device */
#define MAPLE_1_DEPE                        OFF  /**< Using Maple 1 DEPE Device */

#define MAPLE_0_CRCPE                       OFF  /**< Using Maple 0 CRCPE Device */
#define MAPLE_1_CRCPE                       OFF  /**< Using Maple 1 CRCPE Device */
#define MAPLE_2_CRCPE                       OFF  /**< Using Maple 2 CRCPE Device */

#define MAPLE_0_TVPE                        ON  /**< Using Maple 0 eTVPE Device */
#define MAPLE_1_TVPE                        OFF  /**< Using Maple 1 eTVPE Device */

#define MAPLE_0_EQPE                        OFF  /**< Using Maple 0 EQPE Device */
#define MAPLE_1_EQPE                        OFF  /**< Using Maple 1 EQPE Device */

#define MAPLE_0_PDSCH                       ON  /**< Using Maple 0 PDSCH Device */
#define MAPLE_1_PDSCH                       OFF  /**< Using Maple 1 PDSCH Device */

#define MAPLE_0_PUSCH                       ON   /**< Using Maple 0 PUSCH Device */
#define MAPLE_1_PUSCH                       OFF  /**< Using Maple 1 PUSCH Device */

#define MAPLE_0_PUFFT                       ON   /**< Using Maple 0 PUFFT Device */
//#define MAPLE_1_PUFFT                 B4860_ON   /**< Using Maple 1 PUFFT Device */
//#define MAPLE_2_PUFFT                 B4860_ON   /**< Using Maple 2 PUFFT Device */
#define MAPLE_1_PUFFT                 OFF   /**< Using Maple 1 PUFFT Device */
#define MAPLE_2_PUFFT                 OFF   /**< Using Maple 2 PUFFT Device */

/* Total number of devices */

#define OS_TOTAL_NUM_OF_CIO_DEVICES         (B4860_I2C1 + B4860_I2C2 + B4860_I2C3 + B4860_I2C4)

#define OS_TOTAL_NUM_OF_BIO_DEVICES         (B4860_CPRI0_ETHERNET + B4860_CPRI1_ETHERNET + B4860_CPRI2_ETHERNET\
                                            + B4860_CPRI3_ETHERNET + B4860_CPRI4_ETHERNET + B4860_CPRI5_ETHERNET\
                                            + B4860_CPRI6_ETHERNET + B4860_CPRI7_ETHERNET\
                                            + B4860_CPRI0_HDLC + B4860_CPRI1_HDLC + B4860_CPRI2_HDLC + B4860_CPRI3_HDLC\
                                            + B4860_CPRI4_HDLC + B4860_CPRI5_HDLC + B4860_CPRI6_HDLC + B4860_CPRI7_HDLC)


#define OS_TOTAL_NUM_OF_SIO_DEVICES         (B4860_CPRI0_IQ + B4860_CPRI1_IQ + B4860_CPRI2_IQ + B4860_CPRI3_IQ\
                                            + B4860_CPRI4_IQ + B4860_CPRI5_IQ + B4860_CPRI0_VSS + B4860_CPRI1_VSS\
                                            + B4860_CPRI2_VSS + B4860_CPRI3_VSS + B4860_CPRI4_VSS + B4860_CPRI5_VSS)

#define OS_TOTAL_NUM_OF_COP_DEVICES         ( MAPLE_0 + MAPLE_1 + MAPLE_2 + MAPLE_0_FTPE_0 + MAPLE_0_FTPE_1 + MAPLE_0_FTPE_2 \
                                            + MAPLE_1_FTPE_0 + MAPLE_1_FTPE_1 + MAPLE_1_FTPE_2 + MAPLE_2_FTPE_0 + MAPLE_2_FTPE_1 \
                                            + MAPLE_0_DEPE + MAPLE_1_DEPE + MAPLE_0_CRCPE + MAPLE_1_CRCPE + MAPLE_2_CRCPE \
                                            + MAPLE_0_TVPE + MAPLE_1_TVPE + MAPLE_0_EQPE + MAPLE_1_EQPE + MAPLE_0_PDSCH + MAPLE_1_PDSCH \
                                            + MAPLE_0_PUSCH + MAPLE_1_PUSCH \
                                            + MAPLE_0_PUFFT + MAPLE_1_PUFFT + MAPLE_2_PUFFT)


/* Kernel Awareness************************************************************/

extern volatile uint32_t KernelAwareness_b;
extern volatile uint32_t KernelAwareness_size;


#define USER_KERNEL_AWARENESS_STACK         OFF

#define KERNEL_AWARENESS_ADDR               &KernelAwareness_b

#define KERNEL_AWARENESS_STACK_SIZE         (uint32_t)&KernelAwareness_size


#endif // __OS_CONFIG_H
