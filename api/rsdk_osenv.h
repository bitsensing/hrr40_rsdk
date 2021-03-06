/*
* Copyright 2020-2021 NXP
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef RSDK_OSENV_H
#define RSDK_OSENV_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/


/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

//identifies stand-alone build option (no OS support)
#define RSDK_OSENV_SA (!(defined(__linux__) || defined(__QNX__) || defined(__INTEGRITY) || defined(__ZEPHYR__)))

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
*                                    FUNCTION PROTOTYPES
==================================================================================================*/

#ifdef __cplusplus
}
#endif

#endif /*RSDK_OSENV_H*/
