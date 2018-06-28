/*
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef OAL_COMM_KERNEL_H
#define OAL_COMM_KERNEL_H

#include "oal_utils.h"

__BEGIN_DECLS

struct oal_dispatcher;
typedef struct oal_dispatcher oal_dispatcher_t;

struct OAL_RPCService;
typedef struct OAL_RPCService* OAL_RPCService_t;

/**
 * This type must be defined by the user and will be used by the
 * server in order to keep user's private data.
 */
struct OAL_ServiceData;
typedef struct OAL_ServiceData* OAL_ServiceData_t;

/**
 * @brief Dispatch function
 *
 * @param d[in]    The dispatcher
 * @param func[in] The ID of the function to be called via RPC
 * @param in[in]   The input arguments (linearized)
 * @param len[in]  The size of the input arguments
 *
 * @return The return code. 0 for success, a non-zero value otherwise.
 * This value will be passed to user space application as a return of RPC.
 */
typedef uint32_t (*OAL_dispatch_t)(oal_dispatcher_t *d, uint32_t func, char *in,
		int len);

/**
 * @brief OAL_RPCRegister Start a RPC service
 *
 * @param name[in]     The name of the service
 * @param dispatch[in] The dispatch function
 *
 * @return NULL if the call fails, a non NULL value otherwise
 */
OAL_RPCService_t OAL_RPCRegister(char *name, OAL_dispatch_t dispatch);

/**
 * @brief Set service's private data. Usually called from <tt>OAL_RPCRegister</tt>
 * in order to save driver's context using <tt>aData</tt>.
 *
 * @param[in] aServ   The service
 * @param[in] aData   The data to be keept
 *
 * @return 0 for success or a negative value otherwise
 */
int OAL_RPCSetPrivateData(OAL_RPCService_t aServ, OAL_ServiceData_t aData);

/**
 * @brief Get service's private data. Usually called from <tt>OAL_dispatch_t</tt>
 * to retrieve driver's context after saving it with <tt>OAL_RPCSetPrivateData</tt>.
 *
 * @param[in]  aServ   The service
 * @param[out] aData   A reference to service's data
 *
 * @return 0 for success or a negative value otherwise
 */
int OAL_RPCGetPrivateData(OAL_RPCService_t aServ, OAL_ServiceData_t *apData);

/**
 * @brief OAL_RPCCleanup Stops the RPC service
 *
 * @param serv[in]    The service
 *
 * @return 0 for success, a negative value otherwise
 */
int OAL_RPCCleanup(const OAL_RPCService_t serv);

/**
 * @brief OAL_RPCAppendReply Append data to OS call reply.
 * Usually it is used by dispatch function as a helper to send data
 * to user space.
 *
 * @param d [in]   The dispatcher
 * @param data[in] The data to be appended
 * @param size[in] The size of the data in bytes
 *
 * @return 0 for success or a negative value otherwise
 */
int OAL_RPCAppendReply(oal_dispatcher_t *d, char *data, size_t size);

/**
 * @brief Get source service that issued the call
 *
 * @param[in] apDisp The dispatcher
 *
 * @return service instance for success, NULL otherwise
 */
OAL_RPCService_t OAL_RPCGetService(oal_dispatcher_t *apDisp);

/**
 * @brief Get process identifier of a calling process (client)
 *
 * @param[in]  apDispatch    The dispatcher
 * @param[out] apClientPID   The PID of the client
 *
 * @return 0 for success or a negative value otherwise
 */
int OAL_RPCGetClientPID(oal_dispatcher_t *apDispatch, pid_t *apClientPID);

__END_DECLS

#endif /* OAL_COMM_KERNEL_H */