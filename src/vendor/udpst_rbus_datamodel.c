/*
 * udpst_rbus_datamodel.c  —  v5.5
 *
 * Included via #include "udpst_rbus_datamodel.c"
 *
 * 100% aligned to TR-181 2.20.1 USP spec:
 *   - 49 Input args  (all Table 1, excluding DELETED params)
 *   - 33 scalar Output args (Status, timing, AtMax, Summary)
   - IncrementalResult.{i} Output table args cached from interval events
 *   - 9  Event args  (IPLayerCapacityIncrementalResult! per spec)
 *   - NO persistent device properties — data goes to USP controller only
 *
 * Flow (per spec):
 *   USP Controller → OperateRequest(IPLayerCapacity(), input_args)
 *   Device         → OperateResp(Status=Running)         [immediate]
 *   Device         → Notify(IncrementalResult!, 9 params) x N intervals
 *   Device         → OperateResp(Status=Complete, all 33 output args)
 *
 * v5.0 fixes:
 *   - SIGABRT fix: USP_SIGNAL_* calls moved off rbus thread onto
 *     a dedicated vendor thread via self-pipe
 *   - No more direct USP_SIGNAL_OperationComplete from rbus callback
 *   - Proper lock snapshot before reading g_result for log lines
 *   - snprintf replaces strncpy in C() macro (no silent truncation)
 *   - rbusValue_GetString NULL guard added
 *   - VENDOR_Stop joins vendor thread before closing rbus handle
 *
 * v5.1 fixes:
 *   - Forward declaration of UDPST_OnIncrementalResult added so
 *     UDPST_EnsureHandle can reference it before its definition
 *   - All rbusValue_GetString(v, "") changed to rbusValue_GetString(v, NULL)
 *     with if (_s) guards — fixes -Werror=incompatible-pointer-types
 *
 * v5.2 fixes:
 *   - Standard compliance: IPLayerCapacityIncrementalResult! is used only
 *     for the 9 TR-181 incremental fields. It is no longer used to carry
 *     final Output.* completion data.
 *   - Final Output.* is received from a private internal rbus completion
 *     event and completed using USP_SIGNAL_OperationComplete(). The private
 *     event is not registered as a USP/TR-181 data model event.
 *
 * v5.3 fixes:
 *   - Adds Output.IncrementalResult.{i}.* support by caching each standard
 *     incremental event and adding dynamic dotted output args to
 *     USP_SIGNAL_OperationComplete().
 *   - Registers IncrementalResult.{i} output argument templates.
 *
 * v5.5 fixes:
 *   - Adds Output.ModalResult.{i}.* support for bimodal mode.
 *   - Caches ModalResult entries from internal completion event and adds
 *     them to USP_SIGNAL_OperationComplete().
 *
 * Spec: TR-181 2.20.1, TR-471 Issue 4 (Sept 2024)
 * Author: Chandra Pokuru, Charter / RDKM
 */

#ifndef __UDPST_DATAMODEL_C__
#define __UDPST_DATAMODEL_C__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <rbus.h>
#include "usp_err_codes.h"
#include "vendor_defs.h"
#include "vendor_api.h"
#include "usp_api.h"

/* =========================================================================
 * Own rbus handle
 * ======================================================================= */
static rbusHandle_t    udpst_rbus_handle = NULL;
#define UDPST_CONSUMER_COMPONENT "usp_udpst_consumer"

/* =========================================================================
 * TR-181 2.20.1 element names
 * ======================================================================= */
#define U_METHOD  "Device.IP.Diagnostics.IPLayerCapacity()"
#define U_EVENT   "Device.IP.Diagnostics.IPLayerCapacityIncrementalResult!"
#define U_INTERNAL_COMPLETE "eRT.com.cisco.spvtg.ccsp.udpst.IPLayerCapacityComplete!"

/* =========================================================================
 * Async operation instance — saved from operate handler
 * Passed to USP_SIGNAL_OperationComplete on completion
 * ======================================================================= */
static int             g_async_instance = -1;
static pthread_mutex_t g_async_lock     = PTHREAD_MUTEX_INITIALIZER;

/* =========================================================================
 * Result cache — all TR-181 2.20.1 Output fields
 * Internal only — NOT exposed as device properties per spec
 * Populated from private internal completion event, used to build OperateResp
 * ======================================================================= */

#define UDPST_MAX_INCREMENTAL_RESULTS 3600
#define UDPST_MAX_MODAL_RESULTS       16

typedef struct {
    char ip_layer_capacity[32];
    char time_of_sub_interval[40];
    char loss_ratio[32];
    char rtt_range[32];
    char pdv_range[32];
    char min_owd[32];
    char reordered_ratio[32];
    char replicated_ratio[32];
    char interface_eth_mbps[32];
} udpst_incremental_result_t;

typedef struct {
    char max_ip_layer_capacity[32];
    char time_of_max[40];
    char max_eth_no_fcs[32];
    char max_eth_with_fcs[32];
    char max_eth_with_fcs_vlan[32];
    char loss_ratio_at_max[32];
    char rtt_range_at_max[32];
    char rtt_min_at_max[32];
    char rtt_max_at_max[32];
    char pdv_range_at_max[32];
    char min_owd_at_max[32];
    char reordered_at_max[32];
    char replicated_at_max[32];
    char eth_mbps_at_max[32];
} udpst_modal_result_t;

typedef struct {
    char status[64];
    char status_code[16];
    char status_message[256];
    char bom_time[40];
    char eom_time[40];
    char tmax_used[16];
    char tmax_rtt_used[16];
    char test_interval[16];
    char ts_resolution_used[16];
    char active_flows[16];
    char max_ip_layer_capacity[32];
    char time_of_max[40];
    char max_eth_no_fcs[32];
    char max_eth_with_fcs[32];
    char max_eth_with_fcs_vlan[32];
    char loss_ratio_at_max[32];
    char rtt_range_at_max[32];
    char rtt_min_at_max[32];
    char rtt_max_at_max[32];
    char pdv_range_at_max[32];
    char min_owd_at_max[32];
    char reordered_at_max[32];
    char replicated_at_max[32];
    char eth_mbps_at_max[32];
    char ip_layer_cap_summary[32];
    char loss_ratio_summary[32];
    char rtt_range_summary[32];
    char pdv_range_summary[32];
    char min_owd_summary[32];
    char min_rtt_summary[32];
    char reordered_summary[32];
    char replicated_summary[32];
    char eth_mbps_summary[32];

    unsigned int num_incremental_results;
    udpst_incremental_result_t incremental[UDPST_MAX_INCREMENTAL_RESULTS];

    unsigned int num_modal_results;
    udpst_modal_result_t modal[UDPST_MAX_MODAL_RESULTS];
} udpst_result_t;

static udpst_result_t  g_result;
static pthread_mutex_t g_result_lock = PTHREAD_MUTEX_INITIALIZER;

/* =========================================================================
 * Self-pipe: rbus callback writes, vendor thread reads.
 *
 * USP_SIGNAL_OperationComplete / USP_SIGNAL_DataModelEvent must ONLY be
 * called from a thread that obuspa knows about (i.e. one started via
 * VENDOR_Init or explicitly registered).  The rbus event callback runs
 * on rbus's internal thread pool — calling USP_SIGNAL_* from there
 * triggers obuspa's internal thread assertion and causes SIGABRT.
 *
 * Solution: the rbus callback only does a pipe write (async-signal-safe).
 * The vendor thread (started in UDPST_VENDOR_Init) reads the pipe and
 * calls USP_SIGNAL_* — fully safe.
 * ======================================================================= */

/* Message types */
#define MSG_TYPE_INTERVAL   1
#define MSG_TYPE_COMPLETE   2

typedef struct {
    int            msg_type;        /* MSG_TYPE_INTERVAL or MSG_TYPE_COMPLETE */
    int            instance;        /* async op instance (complete only)      */
    /* per-interval fields */
    int            interval_num;
    char           cap[32];
    char           ts[40];
    char           loss[32];
    char           rtt[32];
    char           pdv[32];
    char           owd[32];
    char           reord[32];
    char           replic[32];
    char           eth[32];
} udpst_pipe_msg_t;

static int       g_pipe_rd          = -1;
static int       g_pipe_wr          = -1;
static pthread_t g_vendor_tid;
static int       g_vendor_tid_valid = 0;

/* =========================================================================
 * Forward declaration — UDPST_OnIncrementalResult is defined later in
 * this file but is referenced by UDPST_EnsureHandle as a callback pointer.
 * ======================================================================= */
static void UDPST_OnIncrementalResult(rbusHandle_t handle,
                                       rbusEvent_t const *event,
                                       rbusEventSubscription_t *sub);
static void UDPST_OnInternalComplete(rbusHandle_t handle,
                                     rbusEvent_t const *event,
                                     rbusEventSubscription_t *sub);

/* =========================================================================
 * Build OperateResp output args — all 33 TR-181 2.20.1 Output fields
 * Caller MUST hold g_result_lock
 * ======================================================================= */
static void UDPST_BuildOutputArgs(kv_vector_t *out)
{
    USP_ARG_Add(out, "Status",                    g_result.status);
    USP_ARG_Add(out, "StatusCode",                g_result.status_code);
    USP_ARG_Add(out, "StatusMessage",             g_result.status_message);
    USP_ARG_Add(out, "BOMTime",                   g_result.bom_time);
    USP_ARG_Add(out, "EOMTime",                   g_result.eom_time);
    USP_ARG_Add(out, "TmaxUsed",                  g_result.tmax_used);
    USP_ARG_Add(out, "TmaxRTTUsed",               g_result.tmax_rtt_used);
    USP_ARG_Add(out, "TestInterval",              g_result.test_interval);
    USP_ARG_Add(out, "TimestampResolutionUsed",   g_result.ts_resolution_used);
    USP_ARG_Add(out, "ActiveFlows",               g_result.active_flows);
    USP_ARG_Add(out, "MaxIPLayerCapacity",         g_result.max_ip_layer_capacity);
    USP_ARG_Add(out, "TimeOfMax",                  g_result.time_of_max);
    USP_ARG_Add(out, "MaxETHCapacityNoFCS",        g_result.max_eth_no_fcs);
    USP_ARG_Add(out, "MaxETHCapacityWithFCS",      g_result.max_eth_with_fcs);
    USP_ARG_Add(out, "MaxETHCapacityWithFCSVLAN",  g_result.max_eth_with_fcs_vlan);
    USP_ARG_Add(out, "LossRatioAtMax",             g_result.loss_ratio_at_max);
    USP_ARG_Add(out, "RTTRangeAtMax",              g_result.rtt_range_at_max);
    USP_ARG_Add(out, "RTTMinAtMax",                g_result.rtt_min_at_max);
    USP_ARG_Add(out, "RTTMaxAtMax",                g_result.rtt_max_at_max);
    USP_ARG_Add(out, "PDVRangeAtMax",              g_result.pdv_range_at_max);
    USP_ARG_Add(out, "MinOnewayDelayAtMax",        g_result.min_owd_at_max);
    USP_ARG_Add(out, "ReorderedRatioAtMax",        g_result.reordered_at_max);
    USP_ARG_Add(out, "ReplicatedRatioAtMax",       g_result.replicated_at_max);
    USP_ARG_Add(out, "InterfaceEthMbpsAtMax",      g_result.eth_mbps_at_max);
    USP_ARG_Add(out, "IPLayerCapacitySummary",     g_result.ip_layer_cap_summary);
    USP_ARG_Add(out, "LossRatioSummary",           g_result.loss_ratio_summary);
    USP_ARG_Add(out, "RTTRangeSummary",            g_result.rtt_range_summary);
    USP_ARG_Add(out, "PDVRangeSummary",            g_result.pdv_range_summary);
    USP_ARG_Add(out, "MinOnewayDelaySummary",      g_result.min_owd_summary);
    USP_ARG_Add(out, "MinRTTSummary",              g_result.min_rtt_summary);
    USP_ARG_Add(out, "ReorderedRatioSummary",      g_result.reordered_summary);
    USP_ARG_Add(out, "ReplicatedRatioSummary",     g_result.replicated_summary);
    USP_ARG_Add(out, "InterfaceEthMbpsSummary",    g_result.eth_mbps_summary);

    /*
     * Output.IncrementalResult.{i}.*
     *
     * TR-181 models this as an output object table. In OBUSPA command
     * output, pass it as dotted output argument names. Instance numbers
     * are 1-based and sequential with no gaps.
     */
    char key[128];

    for (unsigned int i = 0; i < g_result.num_incremental_results; i++) {
        unsigned int inst = i + 1;

        snprintf(key, sizeof(key), "IncrementalResult.%u.IPLayerCapacity", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].ip_layer_capacity);

        snprintf(key, sizeof(key), "IncrementalResult.%u.TimeOfSubInterval", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].time_of_sub_interval);

        snprintf(key, sizeof(key), "IncrementalResult.%u.LossRatio", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].loss_ratio);

        snprintf(key, sizeof(key), "IncrementalResult.%u.RTTRange", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].rtt_range);

        snprintf(key, sizeof(key), "IncrementalResult.%u.PDVRange", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].pdv_range);

        snprintf(key, sizeof(key), "IncrementalResult.%u.MinOnewayDelay", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].min_owd);

        snprintf(key, sizeof(key), "IncrementalResult.%u.ReorderedRatio", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].reordered_ratio);

        snprintf(key, sizeof(key), "IncrementalResult.%u.ReplicatedRatio", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].replicated_ratio);

        snprintf(key, sizeof(key), "IncrementalResult.%u.InterfaceEthMbps", inst);
        USP_ARG_Add(out, key, g_result.incremental[i].interface_eth_mbps);
    }

    /* Output.ModalResult.{i}.* — present only when bimodal mode is enabled */
    for (unsigned int i = 0; i < g_result.num_modal_results; i++) {
        unsigned int inst = i + 1;

        snprintf(key, sizeof(key), "ModalResult.%u.MaxIPLayerCapacity", inst);
        USP_ARG_Add(out, key, g_result.modal[i].max_ip_layer_capacity);

        snprintf(key, sizeof(key), "ModalResult.%u.TimeOfMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].time_of_max);

        snprintf(key, sizeof(key), "ModalResult.%u.MaxETHCapacityNoFCS", inst);
        USP_ARG_Add(out, key, g_result.modal[i].max_eth_no_fcs);

        snprintf(key, sizeof(key), "ModalResult.%u.MaxETHCapacityWithFCS", inst);
        USP_ARG_Add(out, key, g_result.modal[i].max_eth_with_fcs);

        snprintf(key, sizeof(key), "ModalResult.%u.MaxETHCapacityWithFCSVLAN", inst);
        USP_ARG_Add(out, key, g_result.modal[i].max_eth_with_fcs_vlan);

        snprintf(key, sizeof(key), "ModalResult.%u.LossRatioAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].loss_ratio_at_max);

        snprintf(key, sizeof(key), "ModalResult.%u.RTTRangeAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].rtt_range_at_max);

        snprintf(key, sizeof(key), "ModalResult.%u.RTTMinAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].rtt_min_at_max);

        snprintf(key, sizeof(key), "ModalResult.%u.RTTMaxAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].rtt_max_at_max);

        snprintf(key, sizeof(key), "ModalResult.%u.PDVRangeAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].pdv_range_at_max);

        snprintf(key, sizeof(key), "ModalResult.%u.MinOnewayDelayAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].min_owd_at_max);

        snprintf(key, sizeof(key), "ModalResult.%u.ReorderedRatioAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].reordered_at_max);

        snprintf(key, sizeof(key), "ModalResult.%u.ReplicatedRatioAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].replicated_at_max);

        snprintf(key, sizeof(key), "ModalResult.%u.InterfaceEthMbpsAtMax", inst);
        USP_ARG_Add(out, key, g_result.modal[i].eth_mbps_at_max);
    }
}
static void UDPST_LogOutputArgs(void)
{
    fprintf(stderr, "[UDPST][OUTPUT] Status=%s\n", g_result.status);
    fprintf(stderr, "[UDPST][OUTPUT] StatusCode=%s\n", g_result.status_code);
    fprintf(stderr, "[UDPST][OUTPUT] StatusMessage=%s\n", g_result.status_message);
    fprintf(stderr, "[UDPST][OUTPUT] BOMTime=%s\n", g_result.bom_time);
    fprintf(stderr, "[UDPST][OUTPUT] EOMTime=%s\n", g_result.eom_time);
    fprintf(stderr, "[UDPST][OUTPUT] TmaxUsed=%s\n", g_result.tmax_used);
    fprintf(stderr, "[UDPST][OUTPUT] TmaxRTTUsed=%s\n", g_result.tmax_rtt_used);
    fprintf(stderr, "[UDPST][OUTPUT] TestInterval=%s\n", g_result.test_interval);
    fprintf(stderr, "[UDPST][OUTPUT] TimestampResolutionUsed=%s\n", g_result.ts_resolution_used);
    fprintf(stderr, "[UDPST][OUTPUT] ActiveFlows=%s\n", g_result.active_flows);

    fprintf(stderr, "[UDPST][OUTPUT] MaxIPLayerCapacity=%s\n", g_result.max_ip_layer_capacity);
    fprintf(stderr, "[UDPST][OUTPUT] TimeOfMax=%s\n", g_result.time_of_max);
    fprintf(stderr, "[UDPST][OUTPUT] MaxETHCapacityNoFCS=%s\n", g_result.max_eth_no_fcs);
    fprintf(stderr, "[UDPST][OUTPUT] MaxETHCapacityWithFCS=%s\n", g_result.max_eth_with_fcs);
    fprintf(stderr, "[UDPST][OUTPUT] MaxETHCapacityWithFCSVLAN=%s\n", g_result.max_eth_with_fcs_vlan);

    fprintf(stderr, "[UDPST][OUTPUT] LossRatioAtMax=%s\n", g_result.loss_ratio_at_max);
    fprintf(stderr, "[UDPST][OUTPUT] RTTRangeAtMax=%s\n", g_result.rtt_range_at_max);
    fprintf(stderr, "[UDPST][OUTPUT] RTTMinAtMax=%s\n", g_result.rtt_min_at_max);
    fprintf(stderr, "[UDPST][OUTPUT] RTTMaxAtMax=%s\n", g_result.rtt_max_at_max);
    fprintf(stderr, "[UDPST][OUTPUT] PDVRangeAtMax=%s\n", g_result.pdv_range_at_max);
    fprintf(stderr, "[UDPST][OUTPUT] MinOnewayDelayAtMax=%s\n", g_result.min_owd_at_max);
    fprintf(stderr, "[UDPST][OUTPUT] ReorderedRatioAtMax=%s\n", g_result.reordered_at_max);
    fprintf(stderr, "[UDPST][OUTPUT] ReplicatedRatioAtMax=%s\n", g_result.replicated_at_max);
    fprintf(stderr, "[UDPST][OUTPUT] InterfaceEthMbpsAtMax=%s\n", g_result.eth_mbps_at_max);

    fprintf(stderr, "[UDPST][OUTPUT] IPLayerCapacitySummary=%s\n", g_result.ip_layer_cap_summary);
    fprintf(stderr, "[UDPST][OUTPUT] LossRatioSummary=%s\n", g_result.loss_ratio_summary);
    fprintf(stderr, "[UDPST][OUTPUT] RTTRangeSummary=%s\n", g_result.rtt_range_summary);
    fprintf(stderr, "[UDPST][OUTPUT] PDVRangeSummary=%s\n", g_result.pdv_range_summary);
    fprintf(stderr, "[UDPST][OUTPUT] MinOnewayDelaySummary=%s\n", g_result.min_owd_summary);
    fprintf(stderr, "[UDPST][OUTPUT] MinRTTSummary=%s\n", g_result.min_rtt_summary);
    fprintf(stderr, "[UDPST][OUTPUT] ReorderedRatioSummary=%s\n", g_result.reordered_summary);
    fprintf(stderr, "[UDPST][OUTPUT] ReplicatedRatioSummary=%s\n", g_result.replicated_summary);
    fprintf(stderr, "[UDPST][OUTPUT] InterfaceEthMbpsSummary=%s\n", g_result.eth_mbps_summary);

    fprintf(stderr, "[UDPST][OUTPUT] IncrementalResultNumberOfEntries=%u\n",
            g_result.num_incremental_results);

    for (unsigned int i = 0; i < g_result.num_incremental_results; i++) {
        unsigned int inst = i + 1;

        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.IPLayerCapacity=%s\n",
                inst, g_result.incremental[i].ip_layer_capacity);
        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.TimeOfSubInterval=%s\n",
                inst, g_result.incremental[i].time_of_sub_interval);
        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.LossRatio=%s\n",
                inst, g_result.incremental[i].loss_ratio);
        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.RTTRange=%s\n",
                inst, g_result.incremental[i].rtt_range);
        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.PDVRange=%s\n",
                inst, g_result.incremental[i].pdv_range);
        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.MinOnewayDelay=%s\n",
                inst, g_result.incremental[i].min_owd);
        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.ReorderedRatio=%s\n",
                inst, g_result.incremental[i].reordered_ratio);
        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.ReplicatedRatio=%s\n",
                inst, g_result.incremental[i].replicated_ratio);
        fprintf(stderr, "[UDPST][OUTPUT] IncrementalResult.%u.InterfaceEthMbps=%s\n",
                inst, g_result.incremental[i].interface_eth_mbps);
    }

    fprintf(stderr, "[UDPST][OUTPUT] ModalResultNumberOfEntries=%u\n",
            g_result.num_modal_results);

    for (unsigned int i = 0; i < g_result.num_modal_results; i++) {
        unsigned int inst = i + 1;

        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.MaxIPLayerCapacity=%s\n",
                inst, g_result.modal[i].max_ip_layer_capacity);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.TimeOfMax=%s\n",
                inst, g_result.modal[i].time_of_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.MaxETHCapacityNoFCS=%s\n",
                inst, g_result.modal[i].max_eth_no_fcs);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.MaxETHCapacityWithFCS=%s\n",
                inst, g_result.modal[i].max_eth_with_fcs);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.MaxETHCapacityWithFCSVLAN=%s\n",
                inst, g_result.modal[i].max_eth_with_fcs_vlan);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.LossRatioAtMax=%s\n",
                inst, g_result.modal[i].loss_ratio_at_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.RTTRangeAtMax=%s\n",
                inst, g_result.modal[i].rtt_range_at_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.RTTMinAtMax=%s\n",
                inst, g_result.modal[i].rtt_min_at_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.RTTMaxAtMax=%s\n",
                inst, g_result.modal[i].rtt_max_at_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.PDVRangeAtMax=%s\n",
                inst, g_result.modal[i].pdv_range_at_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.MinOnewayDelayAtMax=%s\n",
                inst, g_result.modal[i].min_owd_at_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.ReorderedRatioAtMax=%s\n",
                inst, g_result.modal[i].reordered_at_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.ReplicatedRatioAtMax=%s\n",
                inst, g_result.modal[i].replicated_at_max);
        fprintf(stderr, "[UDPST][OUTPUT] ModalResult.%u.InterfaceEthMbpsAtMax=%s\n",
                inst, g_result.modal[i].eth_mbps_at_max);
    }
}
/* =========================================================================
 * Vendor thread — the ONLY place USP_SIGNAL_* is called.
 *
 * Reads udpst_pipe_msg_t structs from the self-pipe written by the
 * rbus event callback.  Because this thread is started from VENDOR_Init,
 * obuspa considers it a valid vendor thread and does not abort.
 * ======================================================================= */
static void *UDPST_VendorThread(void *arg)
{
    (void)arg;
    udpst_pipe_msg_t msg;

    fprintf(stderr, "[UDPST] vendor thread started (tid=%lu)\n",
            (unsigned long)pthread_self());

    while (1) {
        /* Blocking read — exits cleanly when write end is closed */
        ssize_t n = read(g_pipe_rd, &msg, sizeof(msg));
        if (n <= 0) {
            fprintf(stderr, "[UDPST] vendor thread pipe closed — exiting\n");
            break;
        }
        if (n != (ssize_t)sizeof(msg)) {
            fprintf(stderr,
                    "[UDPST] vendor thread short read %zd — skip\n", n);
            continue;
        }

        if (msg.msg_type == MSG_TYPE_INTERVAL) {
            /* ── Per-interval: USP_SIGNAL_DataModelEvent ── */
            kv_vector_t *args = USP_ARG_Create();
            USP_ARG_Add(args, "IPLayerCapacity",   msg.cap);
            USP_ARG_Add(args, "TimeOfSubInterval", msg.ts);
            USP_ARG_Add(args, "LossRatio",         msg.loss);
            USP_ARG_Add(args, "RTTRange",          msg.rtt);
            USP_ARG_Add(args, "PDVRange",          msg.pdv);
            USP_ARG_Add(args, "MinOnewayDelay",    msg.owd);
            USP_ARG_Add(args, "ReorderedRatio",    msg.reord);
            USP_ARG_Add(args, "ReplicatedRatio",   msg.replic);
            USP_ARG_Add(args, "InterfaceEthMbps",  msg.eth);
            USP_SIGNAL_DataModelEvent(U_EVENT, args);

            fprintf(stderr,
                    "[UDPST] interval %d — capacity=%s Mbps loss=%s\n",
                    msg.interval_num, msg.cap, msg.loss);

        } else if (msg.msg_type == MSG_TYPE_COMPLETE) {
            /* ── Completion: USP_SIGNAL_OperationComplete ── */
            if (msg.instance < 0) {
                fprintf(stderr,
                        "[UDPST] no async instance (rbuscli invocation) — "
                        "dropping completion\n");
                continue;
            }

            kv_vector_t *out_args = USP_ARG_Create();

            /*
             * Build output args directly from the global result cache.
             * The completion pipe message is intentionally small; do not
             * copy the huge IncrementalResult cache through the pipe.
             */
            pthread_mutex_lock(&g_result_lock);
            UDPST_BuildOutputArgs(out_args);
            UDPST_LogOutputArgs();
            fprintf(stderr,
                    "[UDPST] USP_SIGNAL_OperationComplete inst=%d "
                    "MaxCap=%s Summary=%s\n",
                    msg.instance,
                    g_result.max_ip_layer_capacity,
                    g_result.ip_layer_cap_summary);
            pthread_mutex_unlock(&g_result_lock);

            USP_SIGNAL_OperationComplete(msg.instance,
                                          USP_ERR_OK, NULL, out_args);
        } else {
            fprintf(stderr,
                    "[UDPST] vendor thread unknown msg_type=%d\n",
                    msg.msg_type);
        }
    }
    return NULL;
}

/* =========================================================================
 * Handle guard
 * ======================================================================= */
static int UDPST_EnsureHandle(void)
{
    if (udpst_rbus_handle != NULL)
        return USP_ERR_OK;

    rbusError_t rc = rbus_open(&udpst_rbus_handle, UDPST_CONSUMER_COMPONENT);
    if (rc != RBUS_ERROR_SUCCESS) {
        fprintf(stderr, "[UDPST] rbus_open failed rc=%d\n", rc);
        udpst_rbus_handle = NULL;
        return USP_ERR_INTERNAL_ERROR;
    }

    /* Re-subscribe whenever we open a fresh handle */
    rc = rbusEvent_Subscribe(udpst_rbus_handle,
                             U_EVENT,
                             UDPST_OnIncrementalResult,
                             NULL, 0);
    if (rc != RBUS_ERROR_SUCCESS)
        fprintf(stderr,
                "[UDPST] EnsureHandle: rbusEvent_Subscribe failed rc=%d\n",
                rc);
    else
        fprintf(stderr,
                "[UDPST] EnsureHandle: re-subscribed to %s\n", U_EVENT);

    return USP_ERR_OK;
}

/* =========================================================================
 * rbus EVENT callback: IPLayerCapacityIncrementalResult!
 *
 * *** NEVER calls USP_SIGNAL_* directly — runs on rbus thread ***
 *
 * Standard TR-181 event: per-interval only, 9 spec params.
 * The vendor thread reads the pipe and calls USP_SIGNAL_DataModelEvent().
 * ======================================================================= */
static void UDPST_OnIncrementalResult(rbusHandle_t handle,
                                       rbusEvent_t const *event,
                                       rbusEventSubscription_t *sub)
{
    (void)handle; (void)sub;

    if (g_pipe_wr < 0) {
        fprintf(stderr, "[UDPST] pipe not ready — dropping event\n");
        return;
    }

    rbusObject_t d = event->data;
    if (!d) return;

    rbusValue_t v;
    udpst_pipe_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    /* ── Per-interval event: 9 spec params ── */
    v = rbusObject_GetValue(d, "Interval");
    if (v) msg.interval_num = rbusValue_GetInt32(v);

    /*
     * S() — same NULL-safe pattern as C() above but for the smaller
     * per-interval string fields.
     */
    #define S(key, dst) do { \
        v = rbusObject_GetValue(d, key); \
        if (v) { \
            const char *_s = rbusValue_GetString(v, NULL); \
            if (_s) snprintf(dst, sizeof(dst), "%s", _s); \
        } \
    } while(0)

    S("IPLayerCapacity",   msg.cap);
    S("TimeOfSubInterval", msg.ts);
    S("LossRatio",         msg.loss);
    S("RTTRange",          msg.rtt);
    S("PDVRange",          msg.pdv);
    S("MinOnewayDelay",    msg.owd);
    S("ReorderedRatio",    msg.reord);
    S("ReplicatedRatio",   msg.replic);
    S("InterfaceEthMbps",  msg.eth);
    #undef S

    /*
     * Only bridge interval notifications into USP when this adapter owns an
     * active async operation instance. This prevents direct rbuscli provider
     * tests from generating USP notifications or polluting the result cache.
     */
    pthread_mutex_lock(&g_async_lock);
    int inst = g_async_instance;
    pthread_mutex_unlock(&g_async_lock);

    if (inst < 0) {
        fprintf(stderr,
                "[UDPST] interval event received but no active USP async "
                "instance — ignoring\n");
        return;
    }

    pthread_mutex_lock(&g_result_lock);
    if (g_result.num_incremental_results < UDPST_MAX_INCREMENTAL_RESULTS) {
        unsigned int idx = g_result.num_incremental_results;

        snprintf(g_result.incremental[idx].ip_layer_capacity,
                 sizeof(g_result.incremental[idx].ip_layer_capacity),
                 "%s", msg.cap);
        snprintf(g_result.incremental[idx].time_of_sub_interval,
                 sizeof(g_result.incremental[idx].time_of_sub_interval),
                 "%s", msg.ts);
        snprintf(g_result.incremental[idx].loss_ratio,
                 sizeof(g_result.incremental[idx].loss_ratio),
                 "%s", msg.loss);
        snprintf(g_result.incremental[idx].rtt_range,
                 sizeof(g_result.incremental[idx].rtt_range),
                 "%s", msg.rtt);
        snprintf(g_result.incremental[idx].pdv_range,
                 sizeof(g_result.incremental[idx].pdv_range),
                 "%s", msg.pdv);
        snprintf(g_result.incremental[idx].min_owd,
                 sizeof(g_result.incremental[idx].min_owd),
                 "%s", msg.owd);
        snprintf(g_result.incremental[idx].reordered_ratio,
                 sizeof(g_result.incremental[idx].reordered_ratio),
                 "%s", msg.reord);
        snprintf(g_result.incremental[idx].replicated_ratio,
                 sizeof(g_result.incremental[idx].replicated_ratio),
                 "%s", msg.replic);
        snprintf(g_result.incremental[idx].interface_eth_mbps,
                 sizeof(g_result.incremental[idx].interface_eth_mbps),
                 "%s", msg.eth);

        g_result.num_incremental_results++;
    } else {
        fprintf(stderr,
                "[UDPST] incremental cache full (%u) — dropping interval\n",
                (unsigned int)UDPST_MAX_INCREMENTAL_RESULTS);
    }
    pthread_mutex_unlock(&g_result_lock);

    msg.msg_type = MSG_TYPE_INTERVAL;

    if (write(g_pipe_wr, &msg, sizeof(msg)) != (ssize_t)sizeof(msg))
        fprintf(stderr, "[UDPST] pipe write (interval %d) failed\n",
                msg.interval_num);
}

static void UDPST_CopyRbusValueAsString(rbusObject_t obj,
                                        const char *key,
                                        char *dst,
                                        size_t dst_len)
{
    rbusValue_t v;

    if (!obj || !key || !dst || dst_len == 0)
        return;

    v = rbusObject_GetValue(obj, key);
    if (!v)
        return;

    switch (rbusValue_GetType(v)) {
        case RBUS_STRING: {
            const char *s = rbusValue_GetString(v, NULL);
            snprintf(dst, dst_len, "%s", s ? s : "");
            break;
        }

        case RBUS_UINT32:
            snprintf(dst, dst_len, "%u", rbusValue_GetUInt32(v));
            break;

        case RBUS_INT32:
            snprintf(dst, dst_len, "%d", rbusValue_GetInt32(v));
            break;

        case RBUS_BOOLEAN:
            snprintf(dst, dst_len, "%s",
                     rbusValue_GetBoolean(v) ? "true" : "false");
            break;

        case RBUS_DOUBLE:
            snprintf(dst, dst_len, "%.9f", rbusValue_GetDouble(v));
            break;

        case RBUS_SINGLE:
            snprintf(dst, dst_len, "%.9f", (double)rbusValue_GetSingle(v));
            break;

        default:
            dst[0] = '\0';
            break;
    }
}


/* =========================================================================
 * Internal rbus completion callback: NOT a USP/TR-181 public event.
 *
 * The backend/provider publishes final Output.* on U_INTERNAL_COMPLETE only.
 * This callback converts that private IPC message into the standard USP
 * async command completion for Device.IP.Diagnostics.IPLayerCapacity().
 * ======================================================================= */
static void UDPST_OnInternalComplete(rbusHandle_t handle,
                                     rbusEvent_t const *event,
                                     rbusEventSubscription_t *sub)
{
    (void)handle; (void)sub;

    if (g_pipe_wr < 0) {
        fprintf(stderr, "[UDPST] pipe not ready — dropping completion\n");
        return;
    }

    rbusObject_t d = event->data;
    if (!d) return;

    rbusValue_t v = rbusObject_GetValue(d, "Status");
    const char *st = v ? rbusValue_GetString(v, NULL) : NULL;
    if (!st) st = "";

    fprintf(stderr,
            "[UDPST] internal completion Status=%s (posting to vendor thread)\n",
            st);

    udpst_pipe_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    pthread_mutex_lock(&g_result_lock);
    snprintf(g_result.status, sizeof(g_result.status), "%s", st);

   #define C(key, dst) \
    UDPST_CopyRbusValueAsString(d, key, dst, sizeof(dst))

    C("StatusCode",              g_result.status_code);
    C("StatusMessage",           g_result.status_message);
    C("BOMTime",                 g_result.bom_time);
    C("EOMTime",                 g_result.eom_time);
    C("TmaxUsed",                g_result.tmax_used);
    C("TmaxRTTUsed",             g_result.tmax_rtt_used);
    C("TestInterval",            g_result.test_interval);
    C("TimestampResolutionUsed", g_result.ts_resolution_used);
    C("ActiveFlows",             g_result.active_flows);
    C("MaxIPLayerCapacity",      g_result.max_ip_layer_capacity);
    C("TimeOfMax",               g_result.time_of_max);
    C("MaxETHCapacityNoFCS",     g_result.max_eth_no_fcs);
    C("MaxETHCapacityWithFCS",   g_result.max_eth_with_fcs);
    C("MaxETHCapacityWithFCSVLAN", g_result.max_eth_with_fcs_vlan);
    C("LossRatioAtMax",          g_result.loss_ratio_at_max);
    C("RTTRangeAtMax",           g_result.rtt_range_at_max);
    C("RTTMinAtMax",             g_result.rtt_min_at_max);
    C("RTTMaxAtMax",             g_result.rtt_max_at_max);
    C("PDVRangeAtMax",           g_result.pdv_range_at_max);
    C("MinOnewayDelayAtMax",     g_result.min_owd_at_max);
    C("ReorderedRatioAtMax",     g_result.reordered_at_max);
    C("ReplicatedRatioAtMax",    g_result.replicated_at_max);
    C("InterfaceEthMbpsAtMax",   g_result.eth_mbps_at_max);
    C("IPLayerCapacitySummary",  g_result.ip_layer_cap_summary);
    C("LossRatioSummary",        g_result.loss_ratio_summary);
    C("RTTRangeSummary",         g_result.rtt_range_summary);
    C("PDVRangeSummary",         g_result.pdv_range_summary);
    C("MinOnewayDelaySummary",   g_result.min_owd_summary);
    C("MinRTTSummary",           g_result.min_rtt_summary);
    C("ReorderedRatioSummary",   g_result.reordered_summary);
    C("ReplicatedRatioSummary",  g_result.replicated_summary);
    C("InterfaceEthMbpsSummary", g_result.eth_mbps_summary);

    /* ModalResult.{i} table from internal completion event */
    char _modal_count[16] = "0";
    C("ModalResultNumberOfEntries", _modal_count);
    g_result.num_modal_results = (unsigned int)atoi(_modal_count);
    if (g_result.num_modal_results > UDPST_MAX_MODAL_RESULTS)
        g_result.num_modal_results = UDPST_MAX_MODAL_RESULTS;

    for (unsigned int i = 0; i < g_result.num_modal_results; i++) {
        unsigned int inst = i + 1;
        char key[128];

        snprintf(key, sizeof(key), "ModalResult.%u.MaxIPLayerCapacity", inst);
        C(key, g_result.modal[i].max_ip_layer_capacity);
        snprintf(key, sizeof(key), "ModalResult.%u.TimeOfMax", inst);
        C(key, g_result.modal[i].time_of_max);
        snprintf(key, sizeof(key), "ModalResult.%u.MaxETHCapacityNoFCS", inst);
        C(key, g_result.modal[i].max_eth_no_fcs);
        snprintf(key, sizeof(key), "ModalResult.%u.MaxETHCapacityWithFCS", inst);
        C(key, g_result.modal[i].max_eth_with_fcs);
        snprintf(key, sizeof(key), "ModalResult.%u.MaxETHCapacityWithFCSVLAN", inst);
        C(key, g_result.modal[i].max_eth_with_fcs_vlan);
        snprintf(key, sizeof(key), "ModalResult.%u.LossRatioAtMax", inst);
        C(key, g_result.modal[i].loss_ratio_at_max);
        snprintf(key, sizeof(key), "ModalResult.%u.RTTRangeAtMax", inst);
        C(key, g_result.modal[i].rtt_range_at_max);
        snprintf(key, sizeof(key), "ModalResult.%u.RTTMinAtMax", inst);
        C(key, g_result.modal[i].rtt_min_at_max);
        snprintf(key, sizeof(key), "ModalResult.%u.RTTMaxAtMax", inst);
        C(key, g_result.modal[i].rtt_max_at_max);
        snprintf(key, sizeof(key), "ModalResult.%u.PDVRangeAtMax", inst);
        C(key, g_result.modal[i].pdv_range_at_max);
        snprintf(key, sizeof(key), "ModalResult.%u.MinOnewayDelayAtMax", inst);
        C(key, g_result.modal[i].min_owd_at_max);
        snprintf(key, sizeof(key), "ModalResult.%u.ReorderedRatioAtMax", inst);
        C(key, g_result.modal[i].reordered_at_max);
        snprintf(key, sizeof(key), "ModalResult.%u.ReplicatedRatioAtMax", inst);
        C(key, g_result.modal[i].replicated_at_max);
        snprintf(key, sizeof(key), "ModalResult.%u.InterfaceEthMbpsAtMax", inst);
        C(key, g_result.modal[i].eth_mbps_at_max);
    }
    #undef C

    pthread_mutex_unlock(&g_result_lock);

    pthread_mutex_lock(&g_async_lock);
    msg.instance     = g_async_instance;
    g_async_instance = -1;
    pthread_mutex_unlock(&g_async_lock);

    msg.msg_type = MSG_TYPE_COMPLETE;

    if (write(g_pipe_wr, &msg, sizeof(msg)) != (ssize_t)sizeof(msg))
        fprintf(stderr, "[UDPST] pipe write (complete) failed\n");
}
/* =========================================================================
 * Subscribe to provider event
 * ======================================================================= */
static void UDPST_SubscribeEvents(void)
{
    rbusError_t rc = rbusEvent_Subscribe(udpst_rbus_handle,
                                          U_EVENT,
                                          UDPST_OnIncrementalResult,
                                          NULL, 0);
    if (rc != RBUS_ERROR_SUCCESS)
        fprintf(stderr,
                "[UDPST] rbusEvent_Subscribe failed rc=%d event=%s\n",
                rc, U_EVENT);
    else
        fprintf(stderr, "[UDPST] subscribed to %s\n", U_EVENT);

    rc = rbusEvent_Subscribe(udpst_rbus_handle,
                              U_INTERNAL_COMPLETE,
                              UDPST_OnInternalComplete,
                              NULL, 0);
    if (rc != RBUS_ERROR_SUCCESS)
        fprintf(stderr,
                "[UDPST] rbusEvent_Subscribe failed rc=%d event=%s\n",
                rc, U_INTERNAL_COMPLETE);
    else
        fprintf(stderr, "[UDPST] subscribed to internal completion %s\n",
                U_INTERNAL_COMPLETE);
}

/* =========================================================================
 * USP OPERATE handler: Device.IP.Diagnostics.IPLayerCapacity()
 *
 * Signature matches USP_REGISTER_AsyncOperation:
 *   int handler(dm_req_t *req, kv_vector_t *in, int instance)
 *
 * Saves instance for the vendor thread to use in
 * USP_SIGNAL_OperationComplete.
 * ======================================================================= */
static int UDPST_Op_IPLayerCapacity(dm_req_t *req, kv_vector_t *in,
                                     int instance)
{
    (void)req;

    if (UDPST_EnsureHandle() != USP_ERR_OK)
        return USP_ERR_INTERNAL_ERROR;

    if (g_pipe_wr < 0) {
        fprintf(stderr,
                "[UDPST] pipe not initialised — reject operate\n");
        return USP_ERR_INTERNAL_ERROR;
    }

    /* Reject if a test is already running */
    pthread_mutex_lock(&g_async_lock);
    if (g_async_instance >= 0) {
        fprintf(stderr,
                "[UDPST] test already running (inst=%d) — "
                "reject new request\n", g_async_instance);
        pthread_mutex_unlock(&g_async_lock);
        return USP_ERR_COMMAND_FAILURE;
    }
    g_async_instance = instance;
    pthread_mutex_unlock(&g_async_lock);

    /* Reset result cache */
    pthread_mutex_lock(&g_result_lock);
    memset(&g_result, 0, sizeof(g_result));
    snprintf(g_result.status, sizeof(g_result.status), "Running");
    pthread_mutex_unlock(&g_result_lock);

    /* Build rbus input object from all 49 TR-181 2.20.1 Input args */
    rbusObject_t in_obj;
    rbusObject_Init(&in_obj, NULL);

    #define ADD_STR(rbus_key, usp_key, def) do { \
        const char *_v = USP_ARG_Get(in, usp_key, def); \
        if (_v && strlen(_v) > 0) { \
            rbusValue_t _rv; rbusValue_Init(&_rv); \
            rbusValue_SetString(_rv, _v); \
            rbusObject_SetValue(in_obj, rbus_key, _rv); \
            rbusValue_Release(_rv); \
        } \
    } while(0)

    #define ADD_UINT(rbus_key, usp_key, def) do { \
        const char *_v = USP_ARG_Get(in, usp_key, def); \
        rbusValue_t _rv; rbusValue_Init(&_rv); \
        rbusValue_SetUInt32(_rv, (uint32_t)atoi(_v ? _v : def)); \
        rbusObject_SetValue(in_obj, rbus_key, _rv); \
        rbusValue_Release(_rv); \
    } while(0)

    #define ADD_BOOL(rbus_key, usp_key, def) do { \
        const char *_v = USP_ARG_Get(in, usp_key, def); \
        rbusValue_t _rv; rbusValue_Init(&_rv); \
        rbusValue_SetBoolean(_rv, (_v && strcmp(_v, "true") == 0)); \
        rbusObject_SetValue(in_obj, rbus_key, _rv); \
        rbusValue_Release(_rv); \
    } while(0)

    ADD_STR ("ServerList",                     "ServerList",                     "");
    ADD_STR ("Role",                           "Role",                           "Receiver");
    ADD_STR ("Interface",                      "Interface",                      "");
    ADD_BOOL("JumboFramesPermitted",           "JumboFramesPermitted",           "true");
    ADD_UINT("MTU",                            "MTU",                            "1500");
    ADD_BOOL("LocalInterfaceRateIncluded",     "LocalInterfaceRateIncluded",     "true");
    ADD_BOOL("AuthenticationEnabled",          "AuthenticationEnabled",          "false");
    ADD_STR ("AuthenticationCode",             "AuthenticationCode",             "");
    ADD_STR ("AuthenticationAlias",            "AuthenticationAlias",            "");
    ADD_STR ("AuthenticationKeyFileLocation",  "AuthenticationKeyFileLocation",  "");
    ADD_UINT("FlowCount",                      "FlowCount",                      "0");
    ADD_UINT("MaximumFlows",                   "MaximumFlows",                   "0");
    ADD_UINT("EthernetPriority",               "EthernetPriority",               "0");
    ADD_UINT("DSCP",                           "DSCP",                           "0");
    ADD_STR ("ProtocolVersion",                "ProtocolVersion",                "Any");
    ADD_UINT("UDPPayloadMin",                  "UDPPayloadMin",                  "0");
    ADD_UINT("UDPPayloadMax",                  "UDPPayloadMax",                  "0");
    ADD_STR ("UDPPayloadContent",              "UDPPayloadContent",              "zeroes");
    ADD_UINT("PortMin",                        "PortMin",                        "49152");
    ADD_UINT("PortMax",                        "PortMax",                        "65535");
    ADD_UINT("PortOptionalMin",                "PortOptionalMin",                "0");
    ADD_UINT("PortOptionalMax",                "PortOptionalMax",                "0");
    ADD_STR ("TestType",                       "TestType",                       "Search");
    ADD_BOOL("IPDVEnable",                     "IPDVEnable",                     "false");
    ADD_BOOL("IPRREnable",                     "IPRREnable",                     "false");
    ADD_BOOL("RIPREnable",                     "RIPREnable",                     "false");
    ADD_UINT("PreambleDuration",               "PreambleDuration",               "2");
    ADD_UINT("MaximumTestBandwidth",           "MaximumTestBandwidth",           "0");
    ADD_UINT("StartSendingRate",               "StartSendingRate",               "500");
    ADD_UINT("StartSendingRateIndex",          "StartSendingRateIndex",          "0");
    ADD_UINT("NumberTestSubIntervals",         "NumberTestSubIntervals",         "10");
    ADD_UINT("NumberFirstModeTestSubIntervals","NumberFirstModeTestSubIntervals","0");
    ADD_UINT("TestSubInterval",                "TestSubInterval",                "1000");
    ADD_UINT("StatusFeedbackInterval",         "StatusFeedbackInterval",         "50");
    ADD_UINT("TimeoutNoTestTraffic",           "TimeoutNoTestTraffic",           "1000");
    ADD_UINT("TimeoutNoStatusMessage",         "TimeoutNoStatusMessage",         "1000");
    ADD_UINT("Tmax",                           "Tmax",                           "1000");
    ADD_UINT("TmaxRTT",                        "TmaxRTT",                        "3000");
    ADD_UINT("TimestampResolution",            "TimestampResolution",            "1");
    ADD_UINT("SeqErrThresh",                   "SeqErrThresh",                   "10");
    ADD_BOOL("ReordDupIgnoreEnable",           "ReordDupIgnoreEnable",           "true");
    ADD_UINT("LowerThresh",                    "LowerThresh",                    "30");
    ADD_UINT("UpperThresh",                    "UpperThresh",                    "90");
    ADD_UINT("RetryThresh",                    "RetryThresh",                    "5");
    ADD_UINT("HighSpeedDelta",                 "HighSpeedDelta",                 "10");
    ADD_STR ("RateAdjAlgorithm",               "RateAdjAlgorithm",               "B");
    ADD_UINT("SlowAdjThresh",                  "SlowAdjThresh",                  "3");
    ADD_UINT("HSpeedThresh",                   "HSpeedThresh",                   "1000");
    ADD_UINT("BandwidthMbps",                  "BandwidthMbps",                  "1000");

    #undef ADD_STR
    #undef ADD_UINT
    #undef ADD_BOOL

    rbusObject_t out_obj = NULL;
    rbusError_t rc = rbusMethod_Invoke(udpst_rbus_handle,
                                        U_METHOD, in_obj, &out_obj);
    rbusObject_Release(in_obj);

    if (rc != RBUS_ERROR_SUCCESS) {
        fprintf(stderr, "[UDPST] rbusMethod_Invoke failed rc=%d\n", rc);
        pthread_mutex_lock(&g_async_lock);
        g_async_instance = -1;
        pthread_mutex_unlock(&g_async_lock);
        return USP_ERR_INTERNAL_ERROR;
    }

    if (out_obj) rbusObject_Release(out_obj);

    fprintf(stderr,
            "[UDPST] IPLayerCapacity() invoked inst=%d — "
            "awaiting IncrementalResult events\n", instance);
    return USP_ERR_OK;
}

/* =========================================================================
 * Register with obuspa — TR-181 2.20.1 spec-only
 * ======================================================================= */
static void UDPST_RegisterDataModel(void)
{
    static char *in_args[] = {
        "ServerList", "Role",
        "Interface", "JumboFramesPermitted", "MTU",
        "LocalInterfaceRateIncluded",
        "AuthenticationEnabled", "AuthenticationCode",
        "AuthenticationAlias", "AuthenticationKeyFileLocation",
        "FlowCount", "MaximumFlows",
        "EthernetPriority", "DSCP", "ProtocolVersion",
        "UDPPayloadMin", "UDPPayloadMax", "UDPPayloadContent",
        "PortMin", "PortMax", "PortOptionalMin", "PortOptionalMax",
        "TestType",
        "IPDVEnable", "IPRREnable", "RIPREnable",
        "PreambleDuration", "MaximumTestBandwidth",
        "StartSendingRate", "StartSendingRateIndex",
        "NumberTestSubIntervals", "NumberFirstModeTestSubIntervals",
        "TestSubInterval", "StatusFeedbackInterval",
        "TimeoutNoTestTraffic", "TimeoutNoStatusMessage",
        "Tmax", "TmaxRTT", "TimestampResolution",
        "SeqErrThresh", "ReordDupIgnoreEnable",
        "LowerThresh", "UpperThresh",
        "RetryThresh", "HighSpeedDelta",
        "RateAdjAlgorithm", "SlowAdjThresh", "HSpeedThresh",
        "BandwidthMbps"
    };  /* 49 args */

    static char *out_args[] = {
        "Status", "StatusCode", "StatusMessage",
        "BOMTime", "EOMTime",
        "TmaxUsed", "TmaxRTTUsed",
        "TestInterval", "TimestampResolutionUsed",
        "ActiveFlows",
        "MaxIPLayerCapacity", "TimeOfMax",
        "MaxETHCapacityNoFCS", "MaxETHCapacityWithFCS",
        "MaxETHCapacityWithFCSVLAN",
        "LossRatioAtMax", "RTTRangeAtMax",
        "RTTMinAtMax", "RTTMaxAtMax",
        "PDVRangeAtMax", "MinOnewayDelayAtMax",
        "ReorderedRatioAtMax", "ReplicatedRatioAtMax",
        "InterfaceEthMbpsAtMax",
        "IPLayerCapacitySummary",
        "LossRatioSummary", "RTTRangeSummary", "PDVRangeSummary",
        "MinOnewayDelaySummary", "MinRTTSummary",
        "ReorderedRatioSummary", "ReplicatedRatioSummary",
        "InterfaceEthMbpsSummary",

        /* Output.IncrementalResult.{i}.* table templates */
        "IncrementalResult.{i}.IPLayerCapacity",
        "IncrementalResult.{i}.TimeOfSubInterval",
        "IncrementalResult.{i}.LossRatio",
        "IncrementalResult.{i}.RTTRange",
        "IncrementalResult.{i}.PDVRange",
        "IncrementalResult.{i}.MinOnewayDelay",
        "IncrementalResult.{i}.ReorderedRatio",
        "IncrementalResult.{i}.ReplicatedRatio",
        "IncrementalResult.{i}.InterfaceEthMbps",

        /* Output.ModalResult.{i}.* table templates */
        "ModalResult.{i}.MaxIPLayerCapacity",
        "ModalResult.{i}.TimeOfMax",
        "ModalResult.{i}.MaxETHCapacityNoFCS",
        "ModalResult.{i}.MaxETHCapacityWithFCS",
        "ModalResult.{i}.MaxETHCapacityWithFCSVLAN",
        "ModalResult.{i}.LossRatioAtMax",
        "ModalResult.{i}.RTTRangeAtMax",
        "ModalResult.{i}.RTTMinAtMax",
        "ModalResult.{i}.RTTMaxAtMax",
        "ModalResult.{i}.PDVRangeAtMax",
        "ModalResult.{i}.MinOnewayDelayAtMax",
        "ModalResult.{i}.ReorderedRatioAtMax",
        "ModalResult.{i}.ReplicatedRatioAtMax",
        "ModalResult.{i}.InterfaceEthMbpsAtMax"
    };  /* 56 args: 33 scalar + 9 IncrementalResult + 14 ModalResult templates */

    static char *event_args[] = {
        "IPLayerCapacity", "TimeOfSubInterval",
        "LossRatio", "RTTRange", "PDVRange", "MinOnewayDelay",
        "ReorderedRatio", "ReplicatedRatio", "InterfaceEthMbps"
    };  /* 9 args per TR-181 2.20.1 */

    USP_REGISTER_AsyncOperation(U_METHOD, UDPST_Op_IPLayerCapacity, NULL);
    USP_REGISTER_OperationArguments(U_METHOD, in_args, 49, out_args, 56);
    USP_REGISTER_Event(U_EVENT);
    USP_REGISTER_EventArguments(U_EVENT, event_args, 9);

    fprintf(stderr,
            "[UDPST] registered TR-181 2.20.1:\n"
            "[UDPST]   %s (49 in / 56 out: 33 scalar + IncrementalResult + ModalResult tables)\n"
            "[UDPST]   %s (9 params)\n",
            U_METHOD, U_EVENT);
}

/* =========================================================================
 * Lifecycle
 * ======================================================================= */
static int UDPST_VENDOR_Init(void)
{
    /* ── Open self-pipe ── */
    int pfd[2];
    if (pipe(pfd) != 0) {
        fprintf(stderr, "[UDPST] pipe() failed\n");
        return USP_ERR_INTERNAL_ERROR;
    }
    g_pipe_rd = pfd[0];
    g_pipe_wr = pfd[1];

    /* ── Start vendor thread (obuspa-registered context) ── */
    if (pthread_create(&g_vendor_tid, NULL, UDPST_VendorThread, NULL) != 0) {
        fprintf(stderr, "[UDPST] pthread_create vendor thread failed\n");
        close(g_pipe_rd); g_pipe_rd = -1;
        close(g_pipe_wr); g_pipe_wr = -1;
        return USP_ERR_INTERNAL_ERROR;
    }
    g_vendor_tid_valid = 1;

    /* ── Reset state ── */
    memset(&g_result, 0, sizeof(g_result));
    snprintf(g_result.status, sizeof(g_result.status), "None");
    g_async_instance = -1;

    /* ── Open rbus handle ── */
    if (udpst_rbus_handle != NULL) {
        rbus_close(udpst_rbus_handle);
        udpst_rbus_handle = NULL;
    }
    rbusError_t rc = rbus_open(&udpst_rbus_handle, UDPST_CONSUMER_COMPONENT);
    if (rc != RBUS_ERROR_SUCCESS) {
        fprintf(stderr, "[UDPST] rbus_open failed rc=%d\n", rc);
        close(g_pipe_wr); g_pipe_wr = -1;
        pthread_join(g_vendor_tid, NULL);
        close(g_pipe_rd); g_pipe_rd = -1;
        g_vendor_tid_valid = 0;
        return USP_ERR_INTERNAL_ERROR;
    }

    UDPST_RegisterDataModel();
    UDPST_SubscribeEvents();

    fprintf(stdout,
            "[UDPST] v5.4 TR-181 2.20.1 IPLayerCapacity registered\n"
            "[UDPST]   Operation : %s\n"
            "[UDPST]   Event     : %s\n",
            U_METHOD, U_EVENT);
    return USP_ERR_OK;
}

static int UDPST_VENDOR_Start(void)
{
    return USP_ERR_OK;
}

static int UDPST_VENDOR_Stop(void)
{
    /* Unsubscribe first so no more rbus callbacks fire */
    if (udpst_rbus_handle != NULL) {
        rbusEvent_Unsubscribe(udpst_rbus_handle, U_EVENT);
        rbusEvent_Unsubscribe(udpst_rbus_handle, U_INTERNAL_COMPLETE);
        rbus_close(udpst_rbus_handle);
        udpst_rbus_handle = NULL;
    }

    /* Close write end — vendor thread read() returns 0 and exits */
    if (g_pipe_wr >= 0) {
        close(g_pipe_wr);
        g_pipe_wr = -1;
    }

    /* Wait for vendor thread to drain and exit cleanly */
    if (g_vendor_tid_valid) {
        pthread_join(g_vendor_tid, NULL);
        g_vendor_tid_valid = 0;
    }

    if (g_pipe_rd >= 0) {
        close(g_pipe_rd);
        g_pipe_rd = -1;
    }

    return USP_ERR_OK;
}

#endif /* __UDPST_DATAMODEL_C__ */
