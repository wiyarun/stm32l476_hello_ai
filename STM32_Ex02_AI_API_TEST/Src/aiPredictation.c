/**
  ******************************************************************************
  * @file    aiValidation.c
  * @author  MCD Vertical Application Team
  * @brief   AI Validation application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) YYYY STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Description:
 *
 * Main entry points for AI validation on-target process.
 *
 * History:
 *  - v1.0 - Initial version
 *
 */

#ifndef HAS_INSPECTOR
#define HAS_INSPECTOR
#endif

/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include <aiPredictation.h>
#include <aiTestUtility.h>
#include <bsp_ai.h>

/* AI header files */
#include "ai_platform.h"
#include "core_datatypes.h"   /* AI_PLATFORM_RUNTIME_xxx definition */

#ifdef HAS_INSPECTOR
#include "ai_network_inspector.h"
#include "core_net_inspect.h"
#include "log.h"
#endif

#ifndef AI_MNETWORK_NUMBER
#define AI_MNETWORK_NUMBER (1)
#include "network.h"
#include "network_data.h"
#endif

#ifdef HAS_INSPECTOR
#if MIN_HEAP_SIZE < 0x800
#error Minimum HEAP size is expected (>=2KiB). Conf. should be modified in CubeMX.
#endif
#endif

/* */
#include "aiPbMgr.h"


/* -----------------------------------------------------------------------------
 * TEST-related definitions
 * -----------------------------------------------------------------------------
 */

/* APP configuration 0: disabled 1: enabled */
#define _APP_DEBUG_         			0

#define _APP_VERSION_MAJOR_     (0x01)
#define _APP_VERSION_MINOR_     (0x00)
#define _APP_VERSION_   ((_APP_VERSION_MAJOR_ << 8) | _APP_VERSION_MINOR_)

#define _APP_NAME_   "AI Validation"

/* -----------------------------------------------------------------------------
 * AI-related functions
 * -----------------------------------------------------------------------------
 */
const float input_test_up[270] = {
-1.1168685	,	8.076866	,	1.607201	,
-0.27240697	,	7.8861814	,	1.1849703	,
0.9942854	,	10.800936	,	3.0645783	,
-1.0351465	,	13.565866	,	1.6889231	,
-1.3075534	,	11.223166	,	1.8387469	,
-3.9771416	,	8.771504	,	1.8387469	,
-1.879608	,	5.284695	,	2.2609777	,
-2.4516625	,	3.0237172	,	1.879608	,
0.5720546	,	6.4015636	,	10.038197	,
-4.2086873	,	5.7477865	,	4.140586	,
-2.0294318	,	8.539958	,	2.1111538	,
0.38136974	,	8.853226	,	1.3075534	,
-0.6537767	,	7.504812	,	0.7218784	,
-0.3405087	,	8.308413	,	1.4982382	,
-2.1111538	,	12.980191	,	0.10896278	,
-6.3198414	,	12.067628	,	0.38136974	,
0.10896278	,	5.325556	,	1.879608	,
-1.879608	,	4.671779	,	2.5333846	,
-1.7978859	,	7.6273947	,	5.175732	,
-1.2666923	,	8.076866	,	6.4015636	,
-3.9090397	,	12.067628	,	-0.88532263	,
0.6946377	,	16.317177	,	-0.040861044	,
1.1168685	,	12.258313	,	-0.27240697	,
1.2666923	,	8.730643	,	-0.14982383	,
-0.14982383	,	8.308413	,	0.38136974	,
-0.9942854	,	13.756551	,	1.7297841	,
-3.636633	,	13.484144	,	-0.5720546	,
-3.9771416	,	10.882658	,	1.4573772	,
0.040861044	,	6.701211	,	1.0351465	,
0.19068487	,	2.8330324	,	3.1463003	,
-2.2201166	,	5.2165933	,	5.9793324	,
-5.597963	,	8.117727	,	2.7921712	,
-0.88532263	,	9.384419	,	1.7570249	,
-0.19068487	,	9.043911	,	1.5390993	,
-0.88532263	,	7.43671	,	1.920469	,
-1.3756552	,	14.137921	,	2.7649305	,
-1.334794	,	12.367276	,	1.0351465	,
-3.9090397	,	8.308413	,	1.2258313	,
-1.4982382	,	4.0588636	,	0.9942854	,
-1.525479	,	1.5390993	,	2.8738933	,
-2.3699405	,	9.615966	,	10.147159	,
-3.8681788	,	3.2961242	,	2.4925237	,
-0.5720546	,	5.175732	,	1.4165162	,
-1.1441092	,	8.308413	,	1.1441092	,
1.1168685	,	8.730643	,	2.7240696	,
-4.7943625	,	13.947236	,	1.2258313	,
-1.334794	,	12.258313	,	2.0294318	,
-4.2904096	,	8.62168	,	1.4165162	,
-1.0760075	,	3.8681788	,	1.1441092	,
-1.879608	,	2.4108016	,	4.630918	,
-4.099725	,	12.912089	,	6.5513873	,
-2.4516625	,	10.56939	,	3.2961242	,
-1.2258313	,	7.164303	,	1.6480621	,
-1.1441092	,	8.689782	,	1.6889231	,
-1.0351465	,	8.580819	,	3.214402	,
-2.4108016	,	14.628253	,	3.2961242	,
0.46309182	,	10.691973	,	2.7921712	,
-4.7943625	,	6.891896	,	-1.1849703	,
-0.5720546	,	3.2961242	,	1.1849703	,
-3.1054392	,	2.152015	,	4.3312707	,
-3.214402	,	11.032481	,	4.8216033	,
-2.7240696	,	8.430995	,	1.4982382	,
-1.2666923	,	7.8589406	,	1.5390993	,
-2.070293	,	7.205164	,	1.879608	,
-0.29964766	,	9.302697	,	3.3778462	,
-1.56634	,	13.252599	,	2.2201166	,
-3.445948	,	13.756551	,	2.4516625	,
-4.2495484	,	9.847511	,	2.3699405	,
-1.0760075	,	7.7772183	,	1.6889231	,
-1.0351465	,	2.7649305	,	2.5333846	,
-3.5957718	,	6.3198414	,	11.114203	,
-4.2086873	,	4.2086873	,	2.2609777	,
-0.6946377	,	6.742072	,	1.4982382	,
-1.2258313	,	9.384419	,	0.61291564	,
0.08172209	,	7.6273947	,	2.7649305	,
-0.95342433	,	14.1787815	,	2.7649305	,
-2.7240696	,	14.09706	,	2.9556155	,
-4.7943625	,	7.9270425	,	0.3405087	,
-1.3075534	,	4.862464	,	0.27240697	,
-1.4165162	,	0.38136974	,	2.6423476	,
-3.214402	,	6.0882955	,	12.026767	,
-4.2904096	,	9.724928	,	2.0294318	,
-3.255263	,	7.43671	,	1.3075534	,
0.19068487	,	8.471856	,	-0.88532263	,
-0.08172209	,	8.962189	,	-0.9942854	,
-0.46309182	,	9.765789	,	1.3756552	,
-5.053149	,	11.223166	,	-0.23154591	,
-7.8861814	,	12.789507	,	-1.879608	,
1.8387469	,	9.084772	,	1.920469	,
0.14982383	,	3.8273177	,	1.920469	,

};

const float input_test_walking[270] = {
-3.26	,	18.24	,	0.04	,
7.4	,	10.53	,	1.31	,
4.63	,	6.85	,	-1.88	,
-4.14	,	5.75	,	-1.65	,
-0.23	,	3.26	,	-2.18	,
5.79	,	3.6	,	4.33	,
14.25	,	15.64	,	-4.56	,
-4.79	,	14.79	,	4.06	,
-4.56	,	11.92	,	-3.06	,
7.67	,	12.03	,	2.03	,
6.7	,	9.7	,	7.08	,
0.31	,	6.21	,	3.95	,
-0.15	,	9.47	,	-3.3	,
1.65	,	16.17	,	-1.04	,
3.02	,	10.34	,	0.84	,
1.69	,	4.9	,	-1.33	,
-2.56	,	3.95	,	-1.84	,
0.31	,	5.28	,	4.63	,
5.01	,	12.95	,	10.19	,
1.73	,	12.64	,	10.34	,
0.08	,	6.82	,	-0.53	,
4.06	,	5.86	,	-10.04	,
7.55	,	13.95	,	-8.81	,
4.52	,	12.79	,	-3.49	,
0.08	,	9.96	,	1.18	,
0.38	,	7.59	,	-2.3	,
5.18	,	6.51	,	-1.14	,
6.47	,	8.35	,	-6.09	,
6.02	,	13.33	,	3.72	,
11.3	,	12.18	,	3.02	,
5.13	,	14.56	,	1.76	,
-6.02	,	19.38	,	-6.17	,
2.26	,	14.56	,	-2.53	,
0.84	,	8.66	,	-4.25	,
0.08	,	2.56	,	-5.33	,
-5.98	,	1.04	,	-5.01	,
-0.91	,	3.57	,	-1.12	,
9.08	,	17.82	,	12.87	,
1.54	,	9.43	,	5.28	,
-0.91	,	13.44	,	3.68	,
-5.41	,	8.05	,	-7.4	,
-2.83	,	11.37	,	-10	,
7.74	,	13.82	,	-6.47	,
-0.5	,	9.08	,	0.5	,
-0.3	,	10.23	,	-2.98	,
3.76	,	7.16	,	-1.33	,
4.79	,	7.46	,	-4.06	,
5.41	,	10.12	,	1.04	,
7.35	,	11.6	,	-2.14	,
10.65	,	11.11	,	5.22	,
3.15	,	13.99	,	-0.08	,
-5.67	,	18.39	,	-6.51	,
-5.67	,	18.39	,	-6.51	,
5.48	,	14.18	,	-1.99	,
-0.15	,	8.24	,	-5.33	,
-0.34	,	4.52	,	-3.79	,
-5.71	,	3.64	,	-3.21	,
-2.75	,	6.7	,	1.5	,
9.23	,	14.1	,	11.11	,
2.72	,	5.63	,	3.21	,
-4.02	,	15.6	,	3.26	,
-5.41	,	7.46	,	-9.47	,
2.56	,	16.66	,	-9.62	,
4.63	,	10.46	,	-0.61	,
1.73	,	8.66	,	-1.8	,
3.45	,	8.28	,	-2.14	,
7.12	,	8.35	,	0.15	,
7.44	,	8.35	,	-1.18	,
7.35	,	10.19	,	3.45	,
7.74	,	13.06	,	0.72	,
-3.02	,	16.89	,	2.41	,
-3.98	,	17.97	,	-5.79	,
3.53	,	11.18	,	-2.45	,
2.3	,	6.97	,	-1.92	,
1.31	,	3.45	,	-2.07	,
-1.69	,	4.48	,	-1.84	,
1.65	,	7.78	,	3.21	,
3.41	,	9.7	,	6.66	,
2.72	,	10	,	7.27	,
-2.75	,	13.67	,	2.26	,
-0.95	,	9.08	,	-4.25	,
4.79	,	12.53	,	-3.3	,
4.9	,	10.23	,	0.23	,
2.3	,	9.04	,	-3.11	,
2.49	,	8.39	,	1.23	,
6.47	,	8.54	,	2.53	,
3.64	,	8.24	,	2.49	,
4.18	,	7.16	,	1.95	,
3.26	,	7.55	,	1.73	,
2.45	,	8.92	,	-2.34	,

};


const float input_test_jogging[270] = {
-0.6946377	,	12.680544	,	0.50395286	,
5.012288	,	11.264028	,	0.95342433	,
4.903325	,	10.882658	,	-0.08172209	,
-0.61291564	,	18.496431	,	3.0237172	,
-1.1849703	,	12.108489	,	7.205164	,
1.3756552	,	-2.4925237	,	-6.510526	,
-0.61291564	,	10.56939	,	5.706926	,
-0.50395286	,	13.947236	,	7.0553403	,
-8.430995	,	11.413852	,	5.134871	,
0.95342433	,	1.3756552	,	1.6480621	,
-8.19945	,	19.57244	,	2.7240696	,
1.4165162	,	5.7886477	,	2.982856	,
-1.879608	,	-2.982856	,	-0.29964766	,
-6.1291566	,	6.851035	,	-8.158588	,
5.829509	,	18.0061	,	8.539958	,
6.2789803	,	2.982856	,	2.9147544	,
-1.56634	,	8.308413	,	-1.4573772	,
3.5276701	,	13.593107	,	9.425281	,
-2.0294318	,	-5.706926	,	-10.18802	,
2.7649305	,	10.337844	,	-9.724928	,
3.568531	,	13.6748295	,	1.5390993	,
-0.50395286	,	3.8681788	,	3.718355	,
-2.3018389	,	1.6889231	,	0.08172209	,
-3.568531	,	19.57244	,	6.510526	,
-0.8036005	,	-3.2961242	,	-4.630918	,
0.50395286	,	10.841797	,	13.525005	,
5.706926	,	15.595298	,	6.1700177	,
-8.662541	,	7.273266	,	4.0180025	,
-1.334794	,	1.2258313	,	2.3699405	,
-4.5900574	,	19.57244	,	4.7126403	,
3.8681788	,	3.759216	,	0.84446156	,
-1.7978859	,	1.5390993	,	8.730643	,
7.668256	,	11.264028	,	-1.3075534	,
-2.3699405	,	14.2877445	,	8.281172	,
2.7240696	,	1.4573772	,	0.88532263	,
-3.5957718	,	18.659876	,	-0.6537767	,
3.9499009	,	4.140586	,	3.990762	,
0.46309182	,	-2.4108016	,	2.4108016	,
3.7864566	,	14.137921	,	-3.1463003	,
3.336985	,	19.231932	,	6.5513873	,
5.6660647	,	3.7864566	,	0.53119355	,
0.23154591	,	0.7627395	,	0.7627395	,
-4.8216033	,	19.57244	,	8.158588	,
1.8387469	,	-1.1168685	,	-2.7921712	,
-3.2961242	,	10.079058	,	13.824653	,
11.604536	,	17.079916	,	1.334794	,
-3.173541	,	14.015338	,	5.706926	,
0.61291564	,	1.1168685	,	2.5606253	,
-7.8861814	,	19.57244	,	1.9885708	,
3.1463003	,	5.243834	,	4.671779	,
-3.0237172	,	-4.3312707	,	-3.336985	,
-0.08172209	,	11.917805	,	-7.8861814	,
-1.0351465	,	14.818938	,	4.6036777	,
-2.4516625	,	2.5333846	,	3.486809	,
-1.3756552	,	2.070293	,	-0.19068487	,
-2.4925237	,	19.57244	,	6.469665	,
1.4573772	,	-5.243834	,	-4.372132	,
-1.4165162	,	9.80665	,	5.7477865	,
-1.2666923	,	14.709975	,	6.2108784	,
-3.6774938	,	3.173541	,	3.7864566	,
1.8387469	,	2.7649305	,	-1.7570249	,
-1.2666923	,	19.313654	,	6.3198414	,
2.4108016	,	-7.6546354	,	-6.1291566	,
-0.61291564	,	16.358038	,	4.944186	,
0.040861044	,	17.502148	,	2.5333846	,
-7.6546354	,	7.8180795	,	4.372132	,
-1.2666923	,	0.7218784	,	0.8036005	,
-5.012288	,	19.57244	,	5.5162406	,
1.9477097	,	2.7921712	,	2.070293	,
-5.053149	,	1.6480621	,	7.6273947	,
9.384419	,	13.443283	,	1.0351465	,
-5.434519	,	13.211738	,	6.4424243	,
-0.61291564	,	1.879608	,	1.4165162	,
4.7126403	,	-6.5513873	,	-6.0201936	,
-1.7570249	,	9.302697	,	-6.428804	,
-0.9125633	,	10.501288	,	-0.27240697	,
2.6014864	,	19.381754	,	4.440233	,
5.7886477	,	3.214402	,	1.1441092	,
-1.9885708	,	12.4489975	,	-2.7240696	,
1.4165162	,	16.780268	,	8.471856	,
0.42223078	,	-8.267551	,	-7.3549876	,
-3.568531	,	10.95076	,	-0.8036005	,
-4.671779	,	11.727119	,	0.38136974	,
-2.1383946	,	1.6889231	,	3.5276701	,
-1.334794	,	2.4925237	,	-0.3405087	,
-2.9147544	,	19.57244	,	7.5865335	,
3.5276701	,	-3.9499009	,	-1.920469	,
-4.0588636	,	10.038197	,	14.2877445	,
7.5865335	,	13.33432	,	3.8681788	,
-5.175732	,	14.1787815	,	5.5162406	,

};

static struct ai_network_exec_ctx {
    ai_handle network;
    ai_network_report report;
#ifdef HAS_INSPECTOR
    ai_handle inspector;
    ai_inspector_entry_id net_id;
    ai_inspector_net_report inspector_report;
    ai_inspector_net_entry entry;
    ai_u32 n_cb_in;
    ai_u32 n_cb_out;
    const reqMsg *creq;
    respMsg *cresp;
    bool no_data;
    uint64_t tcom;
    uint64_t tnodes;
#endif
} net_exec_ctx[AI_MNETWORK_NUMBER] = {0};


#define AI_BUFFER_NULL(ptr_)  \
  AI_BUFFER_OBJ_INIT( \
    AI_BUFFER_FORMAT_NONE|AI_BUFFER_FMT_FLAG_CONST, \
    0, 0, 0, 0, \
    AI_HANDLE_PTR(ptr_))


AI_ALIGNED(4)
static ai_u8 activations[AI_MNETWORK_DATA_ACTIVATIONS_SIZE];

static ai_float in_data[AI_MNETWORK_IN_1_SIZE];
static ai_float out_data[AI_MNETWORK_OUT_1_SIZE];

#ifdef HAS_INSPECTOR
static void aiOnExecNode_cb(const ai_handle cookie,
        const ai_inspect_node_info* node_info,
        ai_node_exec_stage stage) {

    struct ai_network_exec_ctx *ctx = (struct ai_network_exec_ctx*)cookie;

    if (stage == AI_NODE_EXEC_PRE_FORWARD_STAGE) {
        ctx->n_cb_in++;
        ctx->tcom += dwtGetCycles();
        dwtReset();
    } else if (stage == AI_NODE_EXEC_POST_FORWARD_STAGE) {
        uint32_t type;
        uint32_t dur = dwtGetCycles();
        dwtReset();
        ctx->tnodes += dur;
        ctx->n_cb_out++;
        if (ctx->n_cb_out == ctx->report.n_nodes)
            type = EnumLayerType_LAYER_TYPE_INTERNAL_LAST;
        else
            type = EnumLayerType_LAYER_TYPE_INTERNAL;
        type = type << 16;
        if (ctx->no_data)
            type |= PB_BUFFER_TYPE_SEND_WITHOUT_DATA;

        aiPbMgrSendAiBuffer(ctx->creq, ctx->cresp, EnumState_S_PROCESSING,
                type | node_info->type,
                node_info->id,
                dwtCyclesToFloatMs(dur),
                &node_info->out);
    }
}
#endif


static struct ai_network_exec_ctx *aiExecCtx(const char *nn_name, int pos)
{
    struct ai_network_exec_ctx *cur = NULL;

    if (!nn_name)
        return NULL;

    if (!nn_name[0]) {
        if ((pos >= 0) && (pos < AI_MNETWORK_NUMBER))
            cur = &net_exec_ctx[pos];
    } else {
        int idx;
        for (idx=0; idx < AI_MNETWORK_NUMBER; idx++) {
            cur = &net_exec_ctx[idx];
            if (cur->network &&
                    (strlen(cur->report.model_name) == strlen(nn_name)) &&
                    (strncmp(cur->report.model_name, nn_name,
                            strlen(cur->report.model_name)) == 0)) {
                break;
            }
            cur = NULL;
        }
    }
    return cur;
}

__STATIC_INLINE
void aiSetPbContext(struct ai_network_exec_ctx *ctx,
        const reqMsg *creq, respMsg *cresp)
{
#ifdef HAS_INSPECTOR
    if (!ctx)
        return;

    ctx->creq = creq;
    ctx->cresp = cresp;
#endif
}

static int aiBootstrap(const char *nn_name, const int idx)
{
    ai_error err;

    /* Creating the network */
    printf("Creating the network \"%s\"..\r\n", nn_name);
    err = ai_mnetwork_create(nn_name, &net_exec_ctx[idx].network, NULL);
    if (err.type) {
        aiLogErr(err, "ai_mnetwork_create");
        return -1;
    }

    /* Query the created network to get relevant info from it */
    if (ai_mnetwork_get_info(net_exec_ctx[idx].network,
            &net_exec_ctx[idx].report)) {
        aiPrintNetworkInfo(&net_exec_ctx[idx].report);
    } else {
        err = ai_mnetwork_get_error(net_exec_ctx[idx].network);
        aiLogErr(err, "ai_mnetwork_get_info");
        ai_mnetwork_destroy(net_exec_ctx[idx].network);
        net_exec_ctx[idx].network = AI_HANDLE_NULL;
        return -2;
    }

    /* Initialize the instance */
    printf("Initializing the network\r\n");
    /* build params structure to provide the reference of the
     * activation and weight buffers */
    const ai_network_params params = {
            AI_BUFFER_NULL(NULL),
            AI_BUFFER_NULL(activations) };

    if (!ai_mnetwork_init(net_exec_ctx[idx].network, &params)) {
        err = ai_mnetwork_get_error(net_exec_ctx[idx].network);
        aiLogErr(err, "ai_mnetwork_init");
        ai_mnetwork_destroy(net_exec_ctx[idx].network);
        net_exec_ctx[idx].network = AI_HANDLE_NULL;
        return -4;
    }
    return 0;
}

#ifdef HAS_INSPECTOR
static int aiInspectorGetReport(struct ai_network_exec_ctx *ctx)
{
    int res = 0;
    if (!ctx)
        return -1;

    if ((ctx->inspector != AI_HANDLE_NULL) &&
            (ctx->net_id != AI_INSPECTOR_NETWORK_BIND_FAILED))
    {
        if (ai_inspector_get_report(ctx->inspector,
                ctx->net_id, &ctx->inspector_report) != true)
            res = -2;
    }
    return res;
}
#endif


static void aiInspectorSendReport(const reqMsg *req, respMsg *resp,
        EnumState state, struct ai_network_exec_ctx *ctx,
        const ai_float dur_ms)
{
#ifdef HAS_INSPECTOR
    if (aiInspectorGetReport(ctx) == 0) {
        resp->which_payload = respMsg_report_tag;
        resp->payload.report.id = ctx->inspector_report.id;
        resp->payload.report.elapsed_ms = dur_ms;
        resp->payload.report.n_nodes = ctx->inspector_report.n_nodes;
        resp->payload.report.signature = ctx->inspector_report.signature;
        resp->payload.report.num_inferences = ctx->inspector_report.num_inferences;
        aiPbMgrSendResp(req, resp, state);
        aiPbMgrWaitAck();
    } else {
        aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
                EnumError_E_GENERIC);
    }
#endif
}


static void aiInspectorUnBind(struct ai_network_exec_ctx *ctx)
{
#ifdef HAS_INSPECTOR
    if (!ctx)
        return;

    if ((ctx->inspector != AI_HANDLE_NULL) &&
            (ctx->net_id != AI_INSPECTOR_NETWORK_BIND_FAILED))
        ai_inspector_unbind_network(ctx->inspector,
                ctx->net_id);

    ctx->net_id = AI_INSPECTOR_NETWORK_BIND_FAILED;
#endif
}

static int aiInspectorBind(struct ai_network_exec_ctx *ctx)
{
    int res = 0;
#ifdef HAS_INSPECTOR
    if (!ctx)
        return 0;

    aiInspectorUnBind(ctx);

    ctx->n_cb_in  = 0;
    ctx->n_cb_out = 0;

    if (ctx->inspector) {
        ctx->net_id = ai_inspector_bind_network(
                ctx->inspector,
                &ctx->entry);

        if (ctx->net_id == AI_INSPECTOR_NETWORK_BIND_FAILED)
            res = -1;
    }

#endif
    return res;
}

static int aiInspectorInitAndBind(struct ai_network_exec_ctx *ctx)
{
#ifdef HAS_INSPECTOR
    ctx->n_cb_in  = 0;
    ctx->n_cb_out = 0;

    ai_inspector_config cfg = {
      .validation_mode = VALIDATION_INSPECT,
      .log_level = LOG_SUDO,
      .log_quiet = false,
      .on_report_destroy = NULL,
      .on_exec_node = &aiOnExecNode_cb,
      .cookie = NULL,
    };

    ai_handle phandle;
    ai_network_params pparams;

    if (ctx->network == AI_HANDLE_NULL)
        return -1;

    ai_mnetwork_get_private_handle(ctx->network,
            &phandle,
            &pparams);

    cfg.cookie = (ai_handle)ctx;

    if (ai_inspector_create(&ctx->inspector, &cfg) )
    {
        ctx->entry.handle = phandle;
        ctx->entry.params = pparams;
        aiInspectorBind(ctx);
    }


#endif
    return 0;
}

static int aiInspectorUnbindAndDestroy(struct ai_network_exec_ctx *ctx)
{
#ifdef HAS_INSPECTOR
    aiInspectorUnBind(ctx);
    if (ctx->inspector != AI_HANDLE_NULL)
        ai_inspector_destroy(ctx->inspector);
    ctx->inspector = AI_HANDLE_NULL;
#endif
    return 0;
}


static int aiInit(void)
{
    int res = -1;
    const char *nn_name;
    int idx;

    printf("\r\nAI platform (API %d.%d.%d - RUNTIME %d.%d.%d)\r\n",
            AI_PLATFORM_API_MAJOR,
            AI_PLATFORM_API_MINOR,
            AI_PLATFORM_API_MICRO,
            AI_PLATFORM_RUNTIME_MAJOR,
            AI_PLATFORM_RUNTIME_MINOR,
            AI_PLATFORM_RUNTIME_MICRO);

    /* Clean all network exec context */
    for (idx=0; idx < AI_MNETWORK_NUMBER; idx++) {
        net_exec_ctx[idx].network = AI_HANDLE_NULL;
#ifdef HAS_INSPECTOR
        net_exec_ctx[idx].inspector = AI_HANDLE_NULL;
        net_exec_ctx[idx].net_id = AI_INSPECTOR_NETWORK_BIND_FAILED;
#endif
    }

    /* Discover and init the embedded network */
    idx = 0;
    do {
        nn_name = ai_mnetwork_find(NULL, idx);
        if (nn_name) {
            printf("\r\nFound network \"%s\"\r\n", nn_name);
            res = aiBootstrap(nn_name, idx);
            if (res)
                nn_name = NULL;
        }
        idx++;
    } while (nn_name);

    return res;
}

static void aiDeInit(void)
{
    ai_error err;
    int idx;

    printf("Releasing the network(s)...\r\n");

    for (idx=0; idx<AI_MNETWORK_NUMBER; idx++) {
        if (net_exec_ctx[idx].network != AI_HANDLE_NULL) {
            if (ai_mnetwork_destroy(net_exec_ctx[idx].network)
                    != AI_HANDLE_NULL) {
                err = ai_mnetwork_get_error(net_exec_ctx[idx].network);
                aiLogErr(err, "ai_mnetwork_destroy");
            }
            net_exec_ctx[idx].network = AI_HANDLE_NULL;
        }
    }
}

/* -----------------------------------------------------------------------------
 * Specific APP/test functions
 * -----------------------------------------------------------------------------
 */

static void aiPbCmdNNInfo(const reqMsg *req, respMsg *resp, void *param)
{
    struct ai_network_exec_ctx *ctx;

    ctx = aiExecCtx(req->name, req->param);
    if (ctx)
        aiPbMgrSendNNInfo(req, resp, EnumState_S_IDLE,
                &ctx->report);
    else
        aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
                EnumError_E_INVALID_PARAM);
}

static void aiPbCmdNNRun(const reqMsg *req, respMsg *resp, void *param)
{
    ai_i32 batch;
    uint32_t tend;
    bool res;
    struct ai_network_exec_ctx *ctx;
    bool inspector_mode = false;
    uint32_t ints;

    ai_buffer ai_input[1];
    ai_buffer ai_output[1];

    ctx = aiExecCtx(req->name, -1);
    if (!ctx) {
        aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
                EnumError_E_INVALID_PARAM);
        return;
    }

#ifdef HAS_INSPECTOR
    ctx->no_data = false;
    if ((req->param & EnumRunParam_P_RUN_MODE_INSPECTOR) ==
            EnumRunParam_P_RUN_MODE_INSPECTOR)
        inspector_mode = true;

    if ((req->param & EnumRunParam_P_RUN_MODE_INSPECTOR_WITHOUT_DATA) ==
            EnumRunParam_P_RUN_MODE_INSPECTOR_WITHOUT_DATA) {
        inspector_mode = true;
        ctx->no_data = true;
    }

    ctx->tcom = 0ULL;
    ctx->tnodes = 0ULL;
#endif

    ai_input[0] = ctx->report.inputs;
    ai_output[0] = ctx->report.outputs;

    ai_input[0].n_batches  = 1;
    ai_input[0].data = AI_HANDLE_PTR(in_data);
    ai_output[0].n_batches = 1;
    ai_output[0].data = AI_HANDLE_PTR(out_data);

    /* 1 Send a ACK (ready to receive a buffer) */
    aiPbMgrSendAck(req, resp, EnumState_S_WAITING,
            aiPbAiBufferSize(&ai_input[0]));

    /* 2 Read buffer */
    res = aiPbMgrReceiveAiBuffer(req, resp, EnumState_S_PROCESSING,
            &ai_input[0], true);
    if (res != true)
        return;

    ints = disableInts();

    /* Update the PN context for inspector callbacks */
    aiSetPbContext(ctx, req, resp);

    if (inspector_mode)
        aiInspectorInitAndBind(ctx);

    /* Processing */
    dwtReset();

    batch = ai_mnetwork_run(ctx->network,
            &ai_input[0], &ai_output[0]);
    if (batch != 1) {
        aiLogErr(ai_mnetwork_get_error(ctx->network),
                "ai_mnetwork_run");
        aiPbMgrSendAck(req, resp, EnumState_S_ERROR,
                EnumError_E_GENERIC);
        return;
    }
    tend = dwtGetCycles();

#ifdef HAS_INSPECTOR
    tend = ctx->tcom + ctx->tnodes + tend;
#endif

    if (inspector_mode)
        aiInspectorSendReport(req, resp, EnumState_S_PROCESSING, ctx,
                dwtCyclesToFloatMs(tend));

    /* 3 Write buffer */
    aiPbMgrSendAiBuffer(req, resp, EnumState_S_DONE,
            EnumLayerType_LAYER_TYPE_OUTPUT << 16 | 0,
            0, dwtCyclesToFloatMs(tend),
            &ai_output[0]);

    if (inspector_mode)
        aiInspectorUnbindAndDestroy(ctx);

    restoreInts(ints);
}

static int aiValidationCore(void)
{
    return aiPbMgrWaitAndProcess();
}

/* -----------------------------------------------------------------------------
 * Exported/Public functions
 * -----------------------------------------------------------------------------
 */

static aiPbCmdFunc pbCmdFuncTab[] = {
#ifdef HAS_INSPECTOR
        AI_PB_CMD_SYNC((void *)EnumCapability_CAP_INSPECTOR),
#else
        AI_PB_CMD_SYNC(NULL),
#endif
        AI_PB_CMD_SYS_INFO(NULL),
        { EnumCmd_CMD_NETWORK_INFO, &aiPbCmdNNInfo, NULL },
        { EnumCmd_CMD_NETWORK_RUN, &aiPbCmdNNRun, NULL },
#if defined(AI_PB_TEST)
        AI_PB_CMD_TEST(NULL),
#endif
        AI_PB_CMD_END,
};

int aiPredictationInit(void)
{
//    aiPbMgrInit(pbCmdFuncTab);

//    aiTestHeader(_APP_NAME_, _APP_VERSION_MAJOR_, _APP_VERSION_MINOR_);

//    dwtIpInit();
//    crcIpInit();
    logDeviceConf();
		int r;

    r = aiInit();
    if (r) {
        printf("\r\nE:  aiInit() r=%d\r\n", r);
        HAL_Delay(2000);
        return r;
    } else {
        printf("\r\n");
        printf("-------------------------------------------\r\n");
        printf("| READY to predict data from the sensor... |\r\n");
        printf("-------------------------------------------\r\n");
        printf("\r\n");
        
    }

    //ioDisableWrite();

		
    return 0;
}



int aiPredictationProcess(void)
{
		int r;
		uint32_t idx = 0;
		uint32_t i;
		struct ai_network_exec_ctx *ctx;
		ai_i32 batch;
    
		ai_buffer ai_input[1];
    ai_buffer ai_output[1];
		predict_activity_t activity_from_predict;
	
		for (idx=0; idx < AI_MNETWORK_NUMBER; idx++) {
         ctx = &net_exec_ctx[idx];
				 printf( "Model Name, %s\n\r", ctx->report.model_name);
				 printf( "Model format, 0x%02X\n\r", ctx->report.inputs.format);
				 printf( "Model n_batches, %d\n\r", ctx->report.inputs.n_batches);
				 printf( "Model channels, %d\n\r", ctx->report.inputs.channels);			   
				 printf( "Model height, %d\n\r", ctx->report.inputs.height);
				 printf( "Model width, %d\n\r", ctx->report.inputs.width);
				 if(ctx->network)
				 {
						printf( "Model network hanlder, is null\n\r");
				 }else{
						printf( "Model network hanlder, is't null\n\r");
				 }
				 
				 
				 
				 
		}
		
		printf( "AI Input type Jogging \n\r");	
		printf( "AI Input size %d\n\r", AI_MNETWORK_IN_1_SIZE);
		
		for(i=0;i<AI_MNETWORK_IN_1_SIZE;i++)
		{
				in_data[i] = input_test_jogging[i];
		}
		
		ai_input[0].n_batches  = 1;
    ai_input[0].data = AI_HANDLE_PTR(in_data);
    ai_output[0].n_batches = 1;
    ai_output[0].data = AI_HANDLE_PTR(out_data);
		
		printf( "Start Predict\n\r");
		
		batch = ai_mnetwork_run(ctx->network,
            &ai_input[0], &ai_output[0]);
		
		printf( "End Predict\n\r");
		printf( "ai run mnetwork %d\n\r", batch);
		printf( "ai OUTSIZE %d\n\r", AI_MNETWORK_OUT_1_SIZE);
		
		
		for(i=0;i<AI_MNETWORK_OUT_1_SIZE;i++)
		{
				printf("%.02f,",(float)out_data[i]);
		}
		
		printf("\n\r");
		activity_from_predict = decode_activity_type(out_data, AI_MNETWORK_OUT_1_SIZE);
			if(activity_from_predict == DOWNSTARIRS)
		{
				printf("Activity predict:: DOWNSTARIRS\n\r");
		}else if(activity_from_predict == JOGGING)
		{
				printf("Activity predict:: JOGGING\n\r");
		}else if(activity_from_predict == SITTING)
		{
				printf("Activity predict:: SITTING\n\r");
		}else if(activity_from_predict == STANDING)
		{
				printf("Activity predict:: STANDING\n\r");
		}else if(activity_from_predict == UPSTAIRS)
		{
				printf("Activity predict:: UPSTAIRS\n\r");
		}else if(activity_from_predict == WALKING)
		{
				printf("Activity predict:: WALKING\n\r");
		}else{
				printf("Activity predict:: Unknow\n\r");
		}		

		printf("\n\r");
		printf("\n\r");
		printf( "AI Input type Walking \n\r");	
		printf( "AI Input size %d\n\r", AI_MNETWORK_IN_1_SIZE);
		
		for(i=0;i<AI_MNETWORK_IN_1_SIZE;i++)
		{
				in_data[i] = input_test_walking[i];
		}
		
		ai_input[0].n_batches  = 1;
    ai_input[0].data = AI_HANDLE_PTR(in_data); 
    ai_output[0].n_batches = 1;
    ai_output[0].data = AI_HANDLE_PTR(out_data);
		printf( "Start Predict\n\r");
		
		batch = ai_mnetwork_run(ctx->network,
            &ai_input[0], &ai_output[0]);
		
		printf( "End Predict\n\r");
		printf( "ai run mnetwork %d\n\r", batch);
		printf( "ai OUTSIZE %d\n\r", AI_MNETWORK_OUT_1_SIZE);
		
		
		for(i=0;i<AI_MNETWORK_OUT_1_SIZE;i++)
		{
				printf("%.02f,",(float)out_data[i]);
		}
		printf("\n\r");
		
		activity_from_predict = decode_activity_type(out_data, AI_MNETWORK_OUT_1_SIZE);
			if(activity_from_predict == DOWNSTARIRS)
		{
				printf("Activity predict:: DOWNSTARIRS\n\r");
		}else if(activity_from_predict == JOGGING)
		{
				printf("Activity predict:: JOGGING\n\r");
		}else if(activity_from_predict == STANDING)
		{
				printf("Activity predict:: STANDING\n\r");
		}else if(activity_from_predict == UPSTAIRS)
		{
				printf("Activity predict:: UPSTAIRS\n\r");
		}else if(activity_from_predict == WALKING)
		{
				printf("Activity predict:: WALKING\n\r");
		}else{
				printf("Activity predict:: Unknow\n\r");
		}
		
		
		
		
		printf("\n\r");
		printf("\n\r");
		printf( "AI Input type UP\n\r");	
		printf( "AI Input size %d\n\r", AI_MNETWORK_IN_1_SIZE);
		
		for(i=0;i<AI_MNETWORK_IN_1_SIZE;i++)
		{
				in_data[i] = input_test_up[i];
		}
		
		ai_input[0].n_batches  = 1;
    ai_input[0].data = AI_HANDLE_PTR(in_data); 
    ai_output[0].n_batches = 1;
    ai_output[0].data = AI_HANDLE_PTR(out_data);
		printf( "Start Predict\n\r");
		
		batch = ai_mnetwork_run(ctx->network,
            &ai_input[0], &ai_output[0]);
		
		printf( "End Predict\n\r");
		printf( "ai run mnetwork %d\n\r", batch);
		printf( "ai OUTSIZE %d\n\r", AI_MNETWORK_OUT_1_SIZE);
		
		
		for(i=0;i<AI_MNETWORK_OUT_1_SIZE;i++)
		{
				printf("%.02f,",(float)out_data[i]);
		}
		printf("\n\r");
		
		activity_from_predict = decode_activity_type(out_data, AI_MNETWORK_OUT_1_SIZE);
		if(activity_from_predict == DOWNSTARIRS)
		{
				printf("Activity predict:: DOWNSTARIRS\n\r");
		}else if(activity_from_predict == JOGGING)
		{
				printf("Activity predict:: JOGGING\n\r");
		}else if(activity_from_predict == STANDING)
		{
				printf("Activity predict:: STANDING\n\r");
		}else if(activity_from_predict == UPSTAIRS)
		{
				printf("Activity predict:: UPSTAIRS\n\r");
		}else if(activity_from_predict == WALKING)
		{
				printf("Activity predict:: WALKING\n\r");
		}else{
				printf("Activity predict:: Unknow\n\r");
		}
		
		
//    do {
//        r = aiValidationCore();
//    } while (r==0);

    return r;
}

void aiPredictationDeInit(void)
{
    printf("\r\n");
    aiDeInit();
    printf("bye bye ...\r\n");
}

int32_t decode_activity_type(const float data_output[], uint32_t output_size)
{
		predict_activity_t res = -1;
	
		float max_value = 0;
		for(uint32_t i=0;i<output_size;i++)
		{
				if(data_output[i] > max_value)
				{
						res = i;
						max_value = data_output[i];
				}
		}			
		
		return res;
}


