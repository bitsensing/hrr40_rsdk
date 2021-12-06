/*
 * Copyright 2016-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SPT_OAL_H
#define SPT_OAL_H

/**************************************************************************************************
Description:
Umbrella header which in turn includes all OAL support for RSDK modules
**************************************************************************************************/

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "rsdk_osenv.h"


#if !defined(__ZEPHYR__)
#include "oal_memmap.h"
#endif
#include "oal_comm.h"
#include "rsdk_status.h"





#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#define UNUSED_ARG(ARG) (void)(ARG)




#ifdef OAL_PRINT_ENABLE
#define OAL_SPT_PRINT(fmt, ...) \
    printf(fmt, ##__VA_ARGS__); \
    fflush(stdout)
#else
#define OAL_SPT_PRINT(fmt, ...)
#endif

#define SPT_OAL_COMM_CHANNEL1_NAME "SptIrqCap"  //max 10 characters?
#define SPT_OAL_COMM_CHANNEL2_NAME "SptNonBlk"


#ifdef HW_MOCK
extern uint32_t hw_read(uint32_t reg);
extern uint32_t fake_reg;
#define HW_WRITE(reg, val)
#define HW_READ(reg) hw_read(fake_reg)  //fake function and argument will be defined in the unit tester
#else
#define HW_WRITE(reg, val) ((reg) = (val))
#define HW_READ(reg) (reg)
#endif  //HW_MOCK

/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

typedef enum
{
    SPT_OAL_RPC_WAIT_FOR_IRQ = 1,
    SPT_OAL_RPC_TERM_SPTIRQCAP
} rsdkSptOalRpcCmd_t;

typedef enum
{
    SPT_OAL_RPC_EVT_TERM_SPTIRQCAP = 0,
    SPT_OAL_RPC_EVT_IRQ_ECS,
    SPT_OAL_RPC_EVT_IRQ_EVT1,
    SPT_OAL_RPC_EVT_IRQ_DMA,
    SPT_OAL_RPC_EVT_IRQ_DSP
} rsdkSptOalRpcEvtType_t;

//interrupt-related info to be sent from OS kernel to user-space through OAL_Comm RPC
typedef struct
{
    rsdkSptOalRpcEvtType_t evtType;
    rsdkStatus_t           isrStatus;
    uint32_t               errInfo;
} evtSharedData_t;

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
#if ((RSDK_OSENV_SA) || defined(__ZEPHYR__))
//RSDK custom implementation of OAL services, as a workaround until deciding to use the OAL "sa" support

static inline uintptr_t OAL_MapUserSpace(uint64_t offset, size_t size)
{
    UNUSED_ARG(offset);
    UNUSED_ARG(size);
    return (uintptr_t)offset;
}

static inline int32_t OAL_UnmapUserSpace(uintptr_t addr, size_t size)
{
    UNUSED_ARG(addr);
    UNUSED_ARG(size);
    return 0;
}

#endif

#ifdef __cplusplus
}
#endif

#endif  //SPT_OAL_H
