#ifndef MAPLE_INIT_COMMON_H
#define MAPLE_INIT_COMMON_H

#include "smartdsp_os.h"
#include "os_mmu.h"
#include "maple.h"

/* Demo memory manager container */
typedef struct
{
    os_mem_part_t *pool;        /**< Holds the pool for managing the memory */
    uint8_t       *space;       /**< Pool of buffers being managed */
    uint8_t       *mem_manager; /**< Memory manager */
} app_mem_manager_t;

/**************************************************************************//**
 @Function      app_mmmu_segment_param_set

 @Description   Maple MMU segment addition

 @Param[in]    m_handle         - maple handle
 @Param[in]    ptr              - pointer used to probe SC MMU to find phys @ & other feature of given segment
 @Param[in]    core_mmu_task_id - mmmu task id associated w such segment
 @Param[out]   mmmu_segment     - pointer mmmu descriptor parameters
 
 @Return       OS_SUCCESS if everything goes well or OS_FAIL otherwise
*//***************************************************************************/
os_status app_mmmu_segment_param_set(os_virt_ptr ptr, uint32_t core_mmu_task_id, maple_mmu_seg_update_t * mmmu_segment);

/**************************************************************************//**
 @Function      app_mmmu_segment_add

 @Description   Maple MMU segment addition

 @Param[in]    m_handle         - maple handle
 @Param[in]    ptr              - pointer used to probe SC MMU to find phys @ & other feature of given segment
 @Param[in]    core_mmu_task_id - mmu task id associated w such segment
 
 @Return       os_mmu_segment_handle - MMMU descriptor number that was used for new segment creation
*//***************************************************************************/
maple_mmu_seg_update_t app_mmmu_segment_add(cop_dev_handle m_handle, os_virt_ptr ptr, uint32_t core_mmu_task_id);

/**************************************************************************//**
 @Function     app_mmmu_enable

 @Description  Maple MMU enable

 @Param[in]    m_handle         - maple handle
 @Param[in]    mmu_protect      - set to TRUE in order to enable MMMU protection from illegal memory accesses made by MAPLE
 @Param[in]    trash_mem        - heap to be used for trash address allocation which will be used for not permitted memory access
 @Param[in]    trash_steering   - steering bits to be used by MAPLE for accessing trash address
 
 @Return       os_mmu_segment_handle - MMMU descriptor number that was used for new segment creation
*//***************************************************************************/
os_status app_mmmu_enable(cop_dev_handle m_handle, bool mmu_protect, os_mem_type trash_mem, uint8_t trash_steering);

/**************************************************************************//**
 @Function     app_mem_manager_create

 @Description  Maple MMU enable

 @Param[in]    size         - size of every block
 @Param[in]    alignment    - alignment for every block
 @Param[in]    num          - number of block to manage
 @Param[in]    mem_type     - heap for blocks allocation
 @Param[in]    shared       - shared pool or not
 @Param[out]   mem_mngr     - pointer to memory manager structure, all fields must be NULL to be reset.
 
 @Return       Pool of buffers
*//***************************************************************************/
os_mem_part_t * app_mem_manager_create(uint32_t size, uint32_t alignment, uint32_t num, os_mem_type mem_type, bool shared, app_mem_manager_t *mem_mngr);

/**************************************************************************//**
 @Function     app_maple_error_detect

 @Description  Gets Maple MMU error and prints it

 @Param[in]    maple_handle - maple device
 @Param[in]    device       - maple device number MAPLE_LW_ID_0, MAPLE_LW_ID_1, MAPLE_W_ID_2
 @Param[in]    error_type   - error type received from error callback
 @Param[out]   mmu_err      - pointer to mmmu error structure
 
 @Return       os_mmu_segment_handle - MMMU descriptor number that was used for new segment creation
*//***************************************************************************/
void app_maple_error_detect(cop_dev_handle maple_handle, uint32_t device, uint32_t error_type, maple_mmu_err_t *mmu_err);

/**************************************************************************//**
 @Function     app_get_dtu_metric

 @Description  Accumulates benchmarks inside dtu_sum

 @Param[in]    dtu_overhead - DTU benchmarking overhead
 @Param[out]   dtu_sum      - Accumulates benchmarks inside dtu_sum
 
 @Return       Returns current benchmark
*//***************************************************************************/
uint32_t app_get_dtu_metric(uint64_t *dtu_sum, uint32_t dtu_overhead);

/**************************************************************************//**
 @Function     app_get_dtu_counters

 @Description  Accumulates 6 DTU counters benchmarks inside dtu_arr

 @Param[out]   dtu_arr      - Accumulates benchmarks inside dtu_arr
 
 @Return       None
*//***************************************************************************/
void app_get_dtu_counters(uint64_t * dtu_arr);

/**************************************************************************//**
 @Function     app_init_dtu

 @Description  Initializes DTU
 
 @Return       Returns DTU benchmarking overhead
*//***************************************************************************/
uint32_t app_init_dtu();
#endif
