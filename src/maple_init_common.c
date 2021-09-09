#include "maple_init_common.h"
#include "sc39xx_mmu.h"
#include "maple_init.h"
#include "sc39xx_dtu.h"

/***************************************************************************/
os_status app_mmmu_segment_param_set(os_virt_ptr ptr, uint32_t core_mmu_task_id, maple_mmu_seg_update_t * mmmu_segment)
{
    os_status      status  = OS_SUCCESS;
    os_mmu_attr    attr    = 0;
    os_mmu_segment_handle sc_desriptor = 0;

    // ******************************************************************
    // Get virtual start of the new descriptor and the size of descriptor as set inside SC MMU
    // ******************************************************************
    status = osMmuDataSegmentProbe((os_const_virt_ptr)(ptr), &sc_desriptor);
    OS_ASSERT_COND(status == OS_SUCCESS);
    status = osMmuDataSegmentVirtBaseGet(sc_desriptor, &mmmu_segment->attributes.virt_addr);
    OS_ASSERT_COND((status == OS_SUCCESS) && (mmmu_segment->attributes.virt_addr != NULL));
    status = osMmuDataSegmentSizeGet(sc_desriptor, &mmmu_segment->attributes.size);
    OS_ASSERT_COND(status == OS_SUCCESS);

    // ******************************************************************
    // Get physical start of the new descriptor
    // ******************************************************************
    status = osMmuDataVirtToPhys(mmmu_segment->attributes.virt_addr, &(mmmu_segment->attributes.phys_addr));
    OS_ASSERT_COND(status == OS_SUCCESS);

    // ******************************************************************
    // Copy the attributes of the existing segment from core MMU format
    // into MAPLE MMU format
    // ******************************************************************
    status = osMmuDataGetAttr(mmmu_segment->attributes.virt_addr, &attr);
    mmmu_segment->attributes.attr = (MAPLE_MMU_ATTR_ENABLE | MAPLE_MMU_ATTR_DID(core_mmu_task_id));
    if (attr & MMU_DATA_CACHEABLE_REGION)    mmmu_segment->attributes.attr  |= MAPLE_MMU_ATTR_CACHEABLE;
    if (attr & MMU_DATA_COHERENT)            mmmu_segment->attributes.attr  |= MAPLE_MMU_ATTR_COHERENT;
    if (attr & MMU_DATA_DEF_WPERM_SUPER)     mmmu_segment->attributes.attr  |= MAPLE_MMU_ATTR_WPERM;
    if (attr & MMU_DATA_DEF_RPERM_SUPER)     mmmu_segment->attributes.attr  |= MAPLE_MMU_ATTR_RPERM;
    if (attr & MMU_DATA_FLEX_SEGMENT_MODEL)  mmmu_segment->attributes.attr  |= MAPLE_MMU_ATTR_FLEX_SEGMENT;
    if (attr & MMU_DATA_WRITETHROUGH_REGION) mmmu_segment->attributes.attr  |= MAPLE_MMU_ATTR_WRITETHROUGH;
    if (attr & MMU_DATA_GUARDED)             mmmu_segment->attributes.attr  |= MAPLE_MMU_ATTR_GAURDED;
 
    return status;
}
/***************************************************************************/
maple_mmu_seg_update_t app_mmmu_segment_add(cop_dev_handle m_handle, os_virt_ptr ptr, uint32_t core_mmu_task_id)
{
    os_status               status          = OS_SUCCESS;
    maple_mmu_seg_update_t  mmu_maple_segment  = {0};

    // ************************
    // Set MMMU segment attributes
    // ************************
    status = app_mmmu_segment_param_set(ptr, core_mmu_task_id, &mmu_maple_segment);
    OS_ASSERT_COND(status == OS_SUCCESS);

    // ************************
    // Add segment to MAPLE MMU
    // ************************
 
    // Find a new segment in the MAPLE MMU - osCopDeviceCtrl() is multicore protected no need for extra spin-lock protected critical section here
    status = osCopDeviceCtrl(m_handle, MAPLE_CMD_MMU_SEGMENT_FIND,   &mmu_maple_segment.descriptor);
    OS_ASSERT_COND(status == OS_SUCCESS);

    // Update this new segment
    status = osCopDeviceCtrl(m_handle, MAPLE_CMD_MMU_SEGMENT_UPDATE, &mmu_maple_segment);
    OS_ASSERT_COND(status == OS_SUCCESS);
 
    /* MAPLE_CMD_MMU_SEGMENT_ENABLE is not required because all the segments have MAPLE_MMU_ATTR_ENABLE attribute */
    return mmu_maple_segment;
}

/***************************************************************************/
os_status app_mmmu_enable(cop_dev_handle m_handle, bool mmu_protect, os_mem_type trash_mem, uint8_t trash_steering)
{
    os_status status = OS_SUCCESS;
    maple_mmu_init_param_t mmu_init_params = {0};

    mmu_init_params.trash_vaddr = osFastAlignedMalloc(0x100, trash_mem, ARCH_CACHE_LINE_SIZE);
    OS_ASSERT_COND(mmu_init_params.trash_vaddr != NULL);
    mmu_init_params.steering_bits = trash_steering;
    status = osCopDeviceCtrl(m_handle, MAPLE_CMD_MMU_ENABLE, &mmu_init_params);
    OS_ASSERT_COND(status == OS_SUCCESS);
    if (mmu_protect == FALSE)
    {
        /* Disable MMU protection */
        status = osCopDeviceCtrl(m_handle, MAPLE_CMD_MMU_PROTECT_DISABLE, &mmu_init_params);
        OS_ASSERT_COND(status == OS_SUCCESS);
    }
    return status;
}
/***************************************************************************/
os_mem_part_t * app_mem_manager_create(uint32_t size, uint32_t alignment, uint32_t num, os_mem_type mem_type, bool shared, app_mem_manager_t *mem_mngr)
{
    if ((size > 0) && (num > 0))
    {
        if (shared)
        {
            OS_ASSERT_COND((mem_type & OS_MEM_SHARED_TYPE) == OS_MEM_SHARED_TYPE);
            if (mem_mngr->mem_manager == NULL)
            {
                mem_mngr->mem_manager = osMalloc(MEM_PART_SIZE(num), OS_MEM_SHARED_CACHEABLE);
            }
        }
        else
        {
            if (mem_mngr->mem_manager == NULL)
            {
                mem_mngr->mem_manager = osMalloc(MEM_PART_SIZE(num), OS_MEM_LOCAL_CACHEABLE);
            }
        }
        OS_ASSERT_COND(mem_mngr->mem_manager != NULL);

        if (mem_mngr->space == NULL)
        {
            mem_mngr->space = osFastMalloc(MEM_PART_DATA_SIZE(num, size, alignment), mem_type);
        }
        OS_ASSERT_COND(mem_mngr->space != NULL);

        mem_mngr->pool = osMemPartCreate(size, num, mem_mngr->space, alignment, OFFSET_0_BYTES, (os_mem_part_t *)mem_mngr->mem_manager, shared);
        OS_ASSERT_COND(mem_mngr->pool != NULL);

        return mem_mngr->pool;
    }
    else
    {
        return NULL;
    }
}
/***************************************************************************/
void app_maple_error_detect(cop_dev_handle maple_handle, uint32_t device, uint32_t error_type, maple_mmu_err_t *mmu_err)
{
    printf("Error occurred on Maple device number %d \n", (device - MAPLE_LW_ID_0));
    if(error_type & MAPLE_MMU_ERR)
    {
        osCopDeviceCtrl(maple_handle, MAPLE_CMD_MMU_ERROR_DETECT, mmu_err);
        switch (mmu_err->err_type)
        {
            case MAPLE_MMU_MULTI_HIT_ERR:
                printf("MAPLE_MMU_MULTI_HIT_ERR occurred \n");
                break;
            case MAPLE_MMU_MISS_ERR:
                printf("MAPLE_MMU_MISS_ERR occurred \n");
                break;
            case MAPLE_MMU_WRITE_ERR:
                printf("MAPLE_MMU_WRITE_ERR occurred \n");
                break;
            case MAPLE_MMU_READ_ERR:
                printf("MAPLE_MMU_READ_ERR occurred \n");
                break;
            default:
                OS_ASSERT;
                break;
        }
        printf("Error address 0x%x \n", mmu_err->err_addr);
    }
    else if (error_type & MAPLE_ECC_ERR)
    {
        printf("MAPLE_ECC_ERR occurred \n");
        OS_ASSERT;
    }
    else if (error_type & MAPLE_SYS_ERR)
    {
        printf("MAPLE_SYS_ERR occurred \n");
        OS_ASSERT;
    }
    else
    {
        printf("Unknown error type occurred \n");
        OS_ASSERT;
    }
}
/***************************************************************************/
void app_get_dtu_counters(uint64_t * dtu_arr)
{
	OS_ASSERT_COND(dtu_arr != NULL);
    dtu_arr[0] += osDtuReadCount(DTU_NO_BUBBLES);
    dtu_arr[1] += osDtuReadCount(DTU_COF);
    dtu_arr[2] += osDtuReadCount(DTU_INTERLOCK);
    dtu_arr[3] += osDtuReadCount(DTU_DATA_MEMORY_HOLDS_AND_FREEZE);
    dtu_arr[4] += osDtuReadCount(DTU_PROGRAM_STARVATION);
    dtu_arr[5] += osDtuReadCount(DTU_REWIND_CYCLES);
}

/***************************************************************************/
/*
 * Accumulates benchmarks inside dtu_sum
 * Returns current benchmark
 */
uint32_t app_get_dtu_metric(uint64_t *dtu_sum, uint32_t dtu_overhead)
{
    uint32_t a0,a1,a2,b0,b1,b2;
 
    a0 = osDtuReadCount(DTU_NO_BUBBLES);
    a1 = osDtuReadCount(DTU_COF);
    a2 = osDtuReadCount(DTU_INTERLOCK);
    b0 = osDtuReadCount(DTU_DATA_MEMORY_HOLDS_AND_FREEZE);
    b1 = osDtuReadCount(DTU_PROGRAM_STARVATION);
    b2 = osDtuReadCount(DTU_REWIND_CYCLES);
    *dtu_sum += (a0+a1+a2+b0+b1+b2-dtu_overhead);
    return (a0+a1+a2+b0+b1+b2);
}
/***************************************************************************/
uint32_t app_init_dtu()
{
    os_status status = OS_SUCCESS;
    status = osDtuInitProfiler(DTU_CEV_CCS1, DTU_CEV_CCS2);
    OS_ASSERT_COND(status == OS_SUCCESS);
    osDtuStartProfiling();
    osDtuStopProfiling();
    return osDtuReadCount(DTU_NO_BUBBLES);
}
