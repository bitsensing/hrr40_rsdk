/*
 * Copyright 2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include <linux/types.h>
#include <linux/stddef.h>

#include "rsdk_S32R45.h"
#include "rsdk_cte_driver_module.h"
#include "rsdk_cte_interrupt.h"
#include "cte_low_level_operations.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

/*==================================================================================================
*                                             ENUMS
==================================================================================================*/

/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

/*==================================================================================================
*                                GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

/*==================================================================================================
*                                       FUNCTIONS
==================================================================================================*/

/******************************************************************************/
/**
 * @brief   CTE PHY error interrupt handler
 */
irqreturn_t RsdkCteIrqHandlerLinux(int32_t iIrq, void *pParams)
{
    (void)iIrq;
    (void)pParams;
    CteIrqHandler();
    return IRQ_HANDLED;
}


#ifdef __cplusplus
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
