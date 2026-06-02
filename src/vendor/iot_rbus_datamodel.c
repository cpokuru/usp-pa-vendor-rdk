/*
   Copyright [2024] [RDK Management]
   Licensed under the Apache License, Version 2.0

   iot_rbus_datamodel.c
   USP-PA rbus CONSUMER for Barton IoT provider (BartonIoTProvider).
   Included directly into vendor.c via #include "iot_rbus_datamodel.c"

   Fixes applied:
   1. NULL-guard + re-open on iot_rbus_handle in IoT_RbusGetStr / IoT_RbusInvoke.
   2. IoT_Op_GetStatus now calls rbus_get(B_PROP_STATUS) instead of
      rbusMethod_Invoke — BartonIoTProvider exposes Status as a property,
      not as a callable rbus method.
   3. Removed "dummy" arg trick from no-arg methods; pass NULL inParams
      so the provider does not reject with RBUS_ERROR_INVALID_INPUT.
*/

#ifndef __IOT_DATAMODEL_C__
#define __IOT_DATAMODEL_C__

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "usp_err_codes.h"
#include "vendor_defs.h"
#include "vendor_api.h"
#include "usp_api.h"

#include <rbus.h>

/* =========================================================================
 * Own rbus handle — separate from vendor.c bus_handle and LCM rbus_handle
 * ======================================================================= */
static rbusHandle_t iot_rbus_handle = NULL;
#define IOT_CONSUMER_COMPONENT  "usp_iot_consumer"

/* =========================================================================
 * Barton rbus element names — verified from manual + barton_rbus_provider.h
 * Provider component: BartonIoTProvider
 * Namespace: Device.IoT.*
 * ======================================================================= */

/* Properties */
#define B_PROP_STATUS           "Device.IoT.Status"
#define B_PROP_DEVICE_COUNT     "Device.IoT.DeviceCount"
#define B_PROP_DEVICES_JSON     "Device.IoT.Devices"
#define B_PROP_DISC_ACTIVE      "Device.IoT.Discovery.Active"

/* Methods — rbus key names verified from manual rbuscli examples */
#define B_METHOD_GET_STATUS     "Device.IoT.GetStatus()"
#define B_METHOD_DISC_START     "Device.IoT.Discovery.Start()"
#define B_METHOD_DISC_STOP      "Device.IoT.Discovery.Stop()"
#define B_METHOD_LIST_DEVICES   "Device.IoT.Device.List()"
#define B_METHOD_GET_DEVICE     "Device.IoT.Device.Get()"
#define B_METHOD_REMOVE_DEVICE  "Device.IoT.Device.Remove()"
#define B_METHOD_READ_RESOURCE  "Device.IoT.Resource.Read()"
#define B_METHOD_WRITE_RESOURCE "Device.IoT.Resource.Write()"
#define B_METHOD_EXEC_RESOURCE  "Device.IoT.Resource.Execute()"
#define B_METHOD_QUERY_RESOURCE "Device.IoT.Resource.Query()"
#define B_METHOD_READ_METADATA  "Device.IoT.Metadata.Read()"
#define B_METHOD_WRITE_METADATA "Device.IoT.Metadata.Write()"
#define B_METHOD_GET_PROPERTY   "Device.IoT.GetProperty()"
#define B_METHOD_SET_PROPERTY   "Device.IoT.SetProperty()"
#define B_METHOD_COMMISSION     "Device.IoT.Matter.Commission()"
#define B_METHOD_OPEN_COMM_WIN  "Device.IoT.Matter.OpenCommissioningWindow()"

/* Events */
#define B_EVT_DEVICE_ADDED      "Device.IoT.DeviceAdded!"
#define B_EVT_DEVICE_REMOVED    "Device.IoT.DeviceRemoved!"
#define B_EVT_RESOURCE_UPDATED  "Device.IoT.ResourceUpdated!"
#define B_EVT_DISC_STARTED      "Device.IoT.DiscoveryStarted!"
#define B_EVT_DISC_STOPPED      "Device.IoT.DiscoveryStopped!"

/* =========================================================================
 * Handle guard — re-opens rbus if handle was closed/nulled by a Stop cycle
 * ======================================================================= */
static int IoT_EnsureHandle(void)
{
    if (iot_rbus_handle != NULL)
        return USP_ERR_OK;

    fprintf(stderr, "[IoT] iot_rbus_handle is NULL — re-opening rbus\n");
    rbusError_t rc = rbus_open(&iot_rbus_handle, IOT_CONSUMER_COMPONENT);
    if (rc != RBUS_ERROR_SUCCESS)
    {
        fprintf(stderr, "[IoT] re-open rbus_open(%s) failed rc=%d\n",
                IOT_CONSUMER_COMPONENT, rc);
        iot_rbus_handle = NULL;
        return USP_ERR_INTERNAL_ERROR;
    }
    fprintf(stderr, "[IoT] re-open rbus_open OK handle=%p\n",
            (void*)iot_rbus_handle);
    return USP_ERR_OK;
}

/* =========================================================================
 * Helpers
 * ======================================================================= */
static int IoT_RbusGetStr(const char *path, char *buf, int len)
{
    if (IoT_EnsureHandle() != USP_ERR_OK)
        return USP_ERR_INTERNAL_ERROR;

    rbusValue_t val = NULL;
    rbusError_t rc  = rbus_get(iot_rbus_handle, path, &val);
    fprintf(stderr, "[IoT] rbus_get(%s) rc=%d handle=%p\n",
            path, rc, (void*)iot_rbus_handle);
    if (rc != RBUS_ERROR_SUCCESS)
        return USP_ERR_INTERNAL_ERROR;
    USP_STRNCPY(buf, rbusValue_GetString(val, NULL), len);
    rbusValue_Release(val);
    return USP_ERR_OK;
}

static int IoT_RbusInvoke(const char *method,
                           rbusObject_t in_obj,
                           rbusObject_t *out_obj)
{
    if (IoT_EnsureHandle() != USP_ERR_OK)
        return USP_ERR_INTERNAL_ERROR;

    rbusError_t rc = rbusMethod_Invoke(iot_rbus_handle, method,
                                       in_obj, out_obj);
    if (rc != RBUS_ERROR_SUCCESS)
    {
        fprintf(stderr, "[IoT] rbusMethod_Invoke(%s) failed rc=%d\n",
                method, rc);
        return USP_ERR_INTERNAL_ERROR;
    }
    return USP_ERR_OK;
}

/* =========================================================================
 * USP GET handlers
 * Signature: int handler(dm_req_t *req, char *buf, int len)
 * ======================================================================= */

static int IoT_GetStatus(dm_req_t *req, char *buf, int len)
{
    (void)req;
    return IoT_RbusGetStr(B_PROP_STATUS, buf, len);
}

static int IoT_GetDeviceCount(dm_req_t *req, char *buf, int len)
{
    (void)req;
    if (IoT_EnsureHandle() != USP_ERR_OK)
        return USP_ERR_INTERNAL_ERROR;

    rbusValue_t val = NULL;
    rbusError_t rc  = rbus_get(iot_rbus_handle, B_PROP_DEVICE_COUNT, &val);
    if (rc != RBUS_ERROR_SUCCESS) return USP_ERR_INTERNAL_ERROR;
    USP_SNPRINTF(buf, len, "%u", rbusValue_GetUInt32(val));
    rbusValue_Release(val);
    return USP_ERR_OK;
}

static int IoT_GetDevices(dm_req_t *req, char *buf, int len)
{
    (void)req;
    return IoT_RbusGetStr(B_PROP_DEVICES_JSON, buf, len);
}

static int IoT_GetDiscoveryActive(dm_req_t *req, char *buf, int len)
{
    (void)req;
    if (IoT_EnsureHandle() != USP_ERR_OK)
        return USP_ERR_INTERNAL_ERROR;

    rbusValue_t val = NULL;
    rbusError_t rc  = rbus_get(iot_rbus_handle, B_PROP_DISC_ACTIVE, &val);
    if (rc != RBUS_ERROR_SUCCESS) return USP_ERR_INTERNAL_ERROR;
    USP_STRNCPY(buf, rbusValue_GetBoolean(val) ? "true" : "false", len);
    rbusValue_Release(val);
    return USP_ERR_OK;
}

/* =========================================================================
 * USP OPERATE handlers
 * Signature: int handler(dm_req_t *req, char *command_key,
 *                        kv_vector_t *in, kv_vector_t *out)
 * ======================================================================= */

/* Device.IoT.GetStatus()
 * FIX: BartonIoTProvider exposes Status as a property, not a callable rbus
 *      method.  Read the property directly via rbus_get instead of
 *      rbusMethod_Invoke to avoid RBUS_ERROR_INVALID_INPUT (rc=2).
 */
static int IoT_Op_GetStatus(dm_req_t *req, char *command_key,
                             kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key; (void)in;
    char buf[256] = {0};
    int err = IoT_RbusGetStr(B_PROP_STATUS, buf, sizeof(buf));
    if (err != USP_ERR_OK) return err;
    USP_ARG_Add(out, "Status", buf);
    return USP_ERR_OK;
}

/* Device.IoT.Discovery.Start()
 * rbus in:  deviceClass (string), timeout (uint32), setupCode (string, opt)
 * rbus out: success (bool)
 * FIX: no dummy args; optional setupCode only added if non-empty.
 */
static int IoT_Op_DiscStart(dm_req_t *req, char *command_key,
                             kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);

    rbusValue_t vDc, vTo;
    rbusValue_Init(&vDc);
    rbusValue_SetString(vDc, USP_ARG_Get(in, "DeviceClass", "matter"));
    rbusValue_Init(&vTo);
    rbusValue_SetUInt32(vTo, (uint32_t)atoi(USP_ARG_Get(in, "Timeout", "60")));
    rbusObject_SetValue(in_obj, "deviceClass", vDc);
    rbusObject_SetValue(in_obj, "timeout",     vTo);
    rbusValue_Release(vDc);
    rbusValue_Release(vTo);

    const char *code = USP_ARG_Get(in, "SetupCode", NULL);
    if (code && strlen(code) > 0)
    {
        rbusValue_t vCode; rbusValue_Init(&vCode);
        rbusValue_SetString(vCode, code);
        rbusObject_SetValue(in_obj, "setupCode", vCode);
        rbusValue_Release(vCode);
    }

    int err = IoT_RbusInvoke(B_METHOD_DISC_START, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "success");
    USP_ARG_Add(out, "Status", (v && rbusValue_GetBoolean(v)) ? "Success" : "Failed");
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Discovery.Stop()
 * rbus in:  none
 * rbus out: success (bool)
 * FIX: pass NULL inParams — no-arg method, dummy caused INVALID_INPUT.
 */
static int IoT_Op_DiscStop(dm_req_t *req, char *command_key,
                            kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key; (void)in;
    rbusObject_t out_obj = NULL;

    int err = IoT_RbusInvoke(B_METHOD_DISC_STOP, NULL, &out_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "success");
    USP_ARG_Add(out, "Status", (v && rbusValue_GetBoolean(v)) ? "Success" : "Failed");
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Device.List()
 * rbus in:  deviceClass (string, optional filter)
 * rbus out: devices (string JSON array)
 * FIX: pass NULL when no filter rather than a dummy value.
 */
static int IoT_Op_ListDevices(dm_req_t *req, char *command_key,
                               kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj = NULL, out_obj = NULL;

    const char *filter = USP_ARG_Get(in, "DeviceClass", NULL);
    if (filter && strlen(filter) > 0)
    {
        rbusObject_Init(&in_obj, NULL);
        rbusValue_t vDc; rbusValue_Init(&vDc);
        rbusValue_SetString(vDc, filter);
        rbusObject_SetValue(in_obj, "deviceClass", vDc);
        rbusValue_Release(vDc);
    }
    /* else in_obj stays NULL — provider treats as "list all" */

    int err = IoT_RbusInvoke(B_METHOD_LIST_DEVICES, in_obj, &out_obj);
    if (in_obj) rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "devices");
    if (v) USP_ARG_Add(out, "Devices", rbusValue_GetString(v, NULL));
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Device.Get()
 * rbus in:  deviceId (string)
 * rbus out: device (string JSON object)
 */
static int IoT_Op_GetDevice(dm_req_t *req, char *command_key,
                             kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vId; rbusValue_Init(&vId);
    rbusValue_SetString(vId, USP_ARG_Get(in, "DeviceId", ""));
    rbusObject_SetValue(in_obj, "deviceId", vId);
    rbusValue_Release(vId);

    int err = IoT_RbusInvoke(B_METHOD_GET_DEVICE, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "device");
    if (v) USP_ARG_Add(out, "Device", rbusValue_GetString(v, NULL));
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Device.Remove()
 * rbus in:  deviceId (string)
 * rbus out: success (bool)
 */
static int IoT_Op_RemoveDevice(dm_req_t *req, char *command_key,
                                kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vId; rbusValue_Init(&vId);
    rbusValue_SetString(vId, USP_ARG_Get(in, "DeviceId", ""));
    rbusObject_SetValue(in_obj, "deviceId", vId);
    rbusValue_Release(vId);

    int err = IoT_RbusInvoke(B_METHOD_REMOVE_DEVICE, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "success");
    USP_ARG_Add(out, "Status", (v && rbusValue_GetBoolean(v)) ? "Success" : "Failed");
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Resource.Read()
 * rbus in:  uri (string)
 * rbus out: value (string)
 */
static int IoT_Op_ReadResource(dm_req_t *req, char *command_key,
                                kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vUri; rbusValue_Init(&vUri);
    rbusValue_SetString(vUri, USP_ARG_Get(in, "Uri", ""));
    rbusObject_SetValue(in_obj, "uri", vUri);
    rbusValue_Release(vUri);

    int err = IoT_RbusInvoke(B_METHOD_READ_RESOURCE, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "value");
    if (v) USP_ARG_Add(out, "Value", rbusValue_GetString(v, NULL));
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Resource.Write()
 * rbus in:  uri (string), value (string)
 * rbus out: success (bool)
 */
static int IoT_Op_WriteResource(dm_req_t *req, char *command_key,
                                 kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vUri, vVal;
    rbusValue_Init(&vUri); rbusValue_SetString(vUri, USP_ARG_Get(in, "Uri",   ""));
    rbusValue_Init(&vVal); rbusValue_SetString(vVal, USP_ARG_Get(in, "Value", ""));
    rbusObject_SetValue(in_obj, "uri",   vUri);
    rbusObject_SetValue(in_obj, "value", vVal);
    rbusValue_Release(vUri);
    rbusValue_Release(vVal);

    int err = IoT_RbusInvoke(B_METHOD_WRITE_RESOURCE, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "success");
    USP_ARG_Add(out, "Status", (v && rbusValue_GetBoolean(v)) ? "Success" : "Failed");
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Resource.Execute()
 * rbus in:  uri (string), payload (string, optional)
 * rbus out: success (bool), response (string)
 */
static int IoT_Op_ExecResource(dm_req_t *req, char *command_key,
                                kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vUri, vPayload;
    rbusValue_Init(&vUri);     rbusValue_SetString(vUri,     USP_ARG_Get(in, "Uri",     ""));
    rbusValue_Init(&vPayload); rbusValue_SetString(vPayload, USP_ARG_Get(in, "Payload", ""));
    rbusObject_SetValue(in_obj, "uri",     vUri);
    rbusObject_SetValue(in_obj, "payload", vPayload);
    rbusValue_Release(vUri);
    rbusValue_Release(vPayload);

    int err = IoT_RbusInvoke(B_METHOD_EXEC_RESOURCE, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t vSuc  = rbusObject_GetValue(out_obj, "success");
    rbusValue_t vResp = rbusObject_GetValue(out_obj, "response");
    USP_ARG_Add(out, "Status", (vSuc && rbusValue_GetBoolean(vSuc)) ? "Success" : "Failed");
    if (vResp) USP_ARG_Add(out, "Response", rbusValue_GetString(vResp, NULL));
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Resource.Query()
 * rbus in:  pattern (string wildcard)
 * rbus out: resources (string JSON array)
 */
static int IoT_Op_QueryResources(dm_req_t *req, char *command_key,
                                  kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vPat; rbusValue_Init(&vPat);
    rbusValue_SetString(vPat, USP_ARG_Get(in, "Pattern", "*"));
    rbusObject_SetValue(in_obj, "pattern", vPat);
    rbusValue_Release(vPat);

    int err = IoT_RbusInvoke(B_METHOD_QUERY_RESOURCE, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "resources");
    if (v) USP_ARG_Add(out, "Resources", rbusValue_GetString(v, NULL));
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Metadata.Read()
 * rbus in:  uri (string)
 * rbus out: value (string)
 */
static int IoT_Op_ReadMetadata(dm_req_t *req, char *command_key,
                                kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vUri; rbusValue_Init(&vUri);
    rbusValue_SetString(vUri, USP_ARG_Get(in, "Uri", ""));
    rbusObject_SetValue(in_obj, "uri", vUri);
    rbusValue_Release(vUri);

    int err = IoT_RbusInvoke(B_METHOD_READ_METADATA, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "value");
    if (v) USP_ARG_Add(out, "Value", rbusValue_GetString(v, NULL));
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Metadata.Write()
 * rbus in:  uri (string), value (string)
 * rbus out: success (bool)
 */
static int IoT_Op_WriteMetadata(dm_req_t *req, char *command_key,
                                 kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vUri, vVal;
    rbusValue_Init(&vUri); rbusValue_SetString(vUri, USP_ARG_Get(in, "Uri",   ""));
    rbusValue_Init(&vVal); rbusValue_SetString(vVal, USP_ARG_Get(in, "Value", ""));
    rbusObject_SetValue(in_obj, "uri",   vUri);
    rbusObject_SetValue(in_obj, "value", vVal);
    rbusValue_Release(vUri);
    rbusValue_Release(vVal);

    int err = IoT_RbusInvoke(B_METHOD_WRITE_METADATA, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "success");
    USP_ARG_Add(out, "Status", (v && rbusValue_GetBoolean(v)) ? "Success" : "Failed");
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.GetProperty()
 * rbus in:  key (string)
 * rbus out: value (string)
 */
static int IoT_Op_GetProperty(dm_req_t *req, char *command_key,
                               kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vKey; rbusValue_Init(&vKey);
    rbusValue_SetString(vKey, USP_ARG_Get(in, "Key", ""));
    rbusObject_SetValue(in_obj, "key", vKey);
    rbusValue_Release(vKey);

    int err = IoT_RbusInvoke(B_METHOD_GET_PROPERTY, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "value");
    if (v) USP_ARG_Add(out, "Value", rbusValue_GetString(v, NULL));
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.SetProperty()
 * rbus in:  key (string), value (string)
 * rbus out: success (bool)
 */
static int IoT_Op_SetProperty(dm_req_t *req, char *command_key,
                               kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vKey, vVal;
    rbusValue_Init(&vKey); rbusValue_SetString(vKey, USP_ARG_Get(in, "Key",   ""));
    rbusValue_Init(&vVal); rbusValue_SetString(vVal, USP_ARG_Get(in, "Value", ""));
    rbusObject_SetValue(in_obj, "key",   vKey);
    rbusObject_SetValue(in_obj, "value", vVal);
    rbusValue_Release(vKey);
    rbusValue_Release(vVal);

    int err = IoT_RbusInvoke(B_METHOD_SET_PROPERTY, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "success");
    USP_ARG_Add(out, "Status", (v && rbusValue_GetBoolean(v)) ? "Success" : "Failed");
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Matter.Commission()
 * rbus in:  setupPayload (string), timeout (uint32)
 * rbus out: success (bool)
 */
static int IoT_Op_Commission(dm_req_t *req, char *command_key,
                              kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vPayload, vTimeout;
    rbusValue_Init(&vPayload);
    rbusValue_SetString(vPayload, USP_ARG_Get(in, "SetupPayload", ""));
    rbusValue_Init(&vTimeout);
    rbusValue_SetUInt32(vTimeout, (uint32_t)atoi(USP_ARG_Get(in, "Timeout", "120")));
    rbusObject_SetValue(in_obj, "setupPayload", vPayload);
    rbusObject_SetValue(in_obj, "timeout",      vTimeout);
    rbusValue_Release(vPayload);
    rbusValue_Release(vTimeout);

    int err = IoT_RbusInvoke(B_METHOD_COMMISSION, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t v = rbusObject_GetValue(out_obj, "success");
    USP_ARG_Add(out, "Status", (v && rbusValue_GetBoolean(v)) ? "Success" : "Failed");
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* Device.IoT.Matter.OpenCommissioningWindow()
 * rbus in:  nodeId (string), timeout (uint32)
 * rbus out: manualCode (string), qrCode (string)
 */
static int IoT_Op_OpenCommWindow(dm_req_t *req, char *command_key,
                                  kv_vector_t *in, kv_vector_t *out)
{
    (void)req; (void)command_key;
    rbusObject_t in_obj, out_obj = NULL;
    rbusObject_Init(&in_obj, NULL);
    rbusValue_t vNode, vTimeout;
    rbusValue_Init(&vNode);
    rbusValue_SetString(vNode, USP_ARG_Get(in, "NodeId", "0"));
    rbusValue_Init(&vTimeout);
    rbusValue_SetUInt32(vTimeout, (uint32_t)atoi(USP_ARG_Get(in, "Timeout", "180")));
    rbusObject_SetValue(in_obj, "nodeId",  vNode);
    rbusObject_SetValue(in_obj, "timeout", vTimeout);
    rbusValue_Release(vNode);
    rbusValue_Release(vTimeout);

    int err = IoT_RbusInvoke(B_METHOD_OPEN_COMM_WIN, in_obj, &out_obj);
    rbusObject_Release(in_obj);
    if (err != USP_ERR_OK) return err;
    rbusValue_t vManual = rbusObject_GetValue(out_obj, "manualCode");
    rbusValue_t vQr     = rbusObject_GetValue(out_obj, "qrCode");
    if (vManual) USP_ARG_Add(out, "ManualCode", rbusValue_GetString(vManual, NULL));
    if (vQr)     USP_ARG_Add(out, "QrCode",     rbusValue_GetString(vQr,     NULL));
    rbusObject_Release(out_obj);
    return USP_ERR_OK;
}

/* =========================================================================
 * rbus EVENT callbacks — Barton fires → forward as USP notifications
 * ======================================================================= */

static void IoT_OnDeviceAdded(rbusHandle_t handle __attribute__((unused)),
                               rbusEvent_t const *event,
                               rbusEventSubscription_t *sub __attribute__((unused)))
{
    rbusValue_t vId      = rbusObject_GetValue(event->data, "deviceId");
    rbusValue_t vUri     = rbusObject_GetValue(event->data, "uri");
    rbusValue_t vClass   = rbusObject_GetValue(event->data, "deviceClass");
    rbusValue_t vVersion = rbusObject_GetValue(event->data, "classVersion");
    kv_vector_t *args    = USP_ARG_Create();
    USP_ARG_Add(args, "DeviceId",     vId      ? rbusValue_GetString(vId,      NULL) : "");
    USP_ARG_Add(args, "Uri",          vUri     ? rbusValue_GetString(vUri,     NULL) : "");
    USP_ARG_Add(args, "DeviceClass",  vClass   ? rbusValue_GetString(vClass,   NULL) : "");
    USP_ARG_Add(args, "ClassVersion", vVersion ? rbusValue_GetString(vVersion, NULL) : "");
    USP_SIGNAL_DataModelEvent(B_EVT_DEVICE_ADDED, args);
}

static void IoT_OnDeviceRemoved(rbusHandle_t handle __attribute__((unused)),
                                 rbusEvent_t const *event,
                                 rbusEventSubscription_t *sub __attribute__((unused)))
{
    rbusValue_t vId    = rbusObject_GetValue(event->data, "deviceId");
    rbusValue_t vClass = rbusObject_GetValue(event->data, "deviceClass");
    kv_vector_t *args  = USP_ARG_Create();
    USP_ARG_Add(args, "DeviceId",    vId    ? rbusValue_GetString(vId,    NULL) : "");
    USP_ARG_Add(args, "DeviceClass", vClass ? rbusValue_GetString(vClass, NULL) : "");
    USP_SIGNAL_DataModelEvent(B_EVT_DEVICE_REMOVED, args);
}

static void IoT_OnResourceUpdated(rbusHandle_t handle __attribute__((unused)),
                                   rbusEvent_t const *event,
                                   rbusEventSubscription_t *sub __attribute__((unused)))
{
    rbusValue_t vUri  = rbusObject_GetValue(event->data, "uri");
    rbusValue_t vVal  = rbusObject_GetValue(event->data, "value");
    kv_vector_t *args = USP_ARG_Create();
    USP_ARG_Add(args, "Uri",   vUri ? rbusValue_GetString(vUri, NULL) : "");
    USP_ARG_Add(args, "Value", vVal ? rbusValue_GetString(vVal, NULL) : "");
    USP_SIGNAL_DataModelEvent(B_EVT_RESOURCE_UPDATED, args);
}

static void IoT_OnDiscStarted(rbusHandle_t handle __attribute__((unused)),
                               rbusEvent_t const *event __attribute__((unused)),
                               rbusEventSubscription_t *sub __attribute__((unused)))
{
    USP_SIGNAL_DataModelEvent(B_EVT_DISC_STARTED, USP_ARG_Create());
}

static void IoT_OnDiscStopped(rbusHandle_t handle __attribute__((unused)),
                               rbusEvent_t const *event __attribute__((unused)),
                               rbusEventSubscription_t *sub __attribute__((unused)))
{
    USP_SIGNAL_DataModelEvent(B_EVT_DISC_STOPPED, USP_ARG_Create());
}

/* =========================================================================
 * Subscribe to Barton events (non-fatal if Barton not yet running)
 * ======================================================================= */
static void IoT_SubscribeEvents(void)
{
    struct { const char *name; rbusEventHandler_t cb; } evts[] = {
        { B_EVT_DEVICE_ADDED,     IoT_OnDeviceAdded     },
        { B_EVT_DEVICE_REMOVED,   IoT_OnDeviceRemoved   },
        { B_EVT_RESOURCE_UPDATED, IoT_OnResourceUpdated },
        { B_EVT_DISC_STARTED,     IoT_OnDiscStarted     },
        { B_EVT_DISC_STOPPED,     IoT_OnDiscStopped     },
    };
    for (int i = 0; i < (int)(sizeof(evts)/sizeof(evts[0])); i++)
    {
        rbusError_t rc = rbusEvent_Subscribe(iot_rbus_handle,
                                              evts[i].name, evts[i].cb,
                                              NULL, 0);
        if (rc != RBUS_ERROR_SUCCESS)
            fprintf(stderr, "[IoT] rbusEvent_Subscribe(%s) failed rc=%d"
                            " (Barton may not be running yet)\n",
                            evts[i].name, rc);
    }
}

/* =========================================================================
 * Register Device.IoT.* with obuspa
 * ======================================================================= */
static void IoT_RegisterDataModel(void)
{
    /* ---- Properties ---- */
    USP_REGISTER_VendorParam_ReadOnly("Device.IoT.Status",
                                       IoT_GetStatus,          DM_STRING);
    USP_REGISTER_VendorParam_ReadOnly("Device.IoT.DeviceNumberOfEntries",
                                       IoT_GetDeviceCount,     DM_STRING);
    USP_REGISTER_VendorParam_ReadOnly("Device.IoT.Devices",
                                       IoT_GetDevices,         DM_STRING);
    USP_REGISTER_VendorParam_ReadOnly("Device.IoT.Discovery.Active",
                                       IoT_GetDiscoveryActive, DM_STRING);

    /* ---- Operations ---- */
    static char *getStatus_out[]   = { "Status" };
    static char *discStart_in[]    = { "DeviceClass", "Timeout", "SetupCode" };
    static char *discStart_out[]   = { "Status" };
    static char *discStop_out[]    = { "Status" };
    static char *listDev_in[]      = { "DeviceClass" };
    static char *listDev_out[]     = { "Devices" };
    static char *getDev_in[]       = { "DeviceId" };
    static char *getDev_out[]      = { "Device" };
    static char *removeDev_in[]    = { "DeviceId" };
    static char *removeDev_out[]   = { "Status" };
    static char *readRes_in[]      = { "Uri" };
    static char *readRes_out[]     = { "Value" };
    static char *writeRes_in[]     = { "Uri", "Value" };
    static char *writeRes_out[]    = { "Status" };
    static char *execRes_in[]      = { "Uri", "Payload" };
    static char *execRes_out[]     = { "Status", "Response" };
    static char *queryRes_in[]     = { "Pattern" };
    static char *queryRes_out[]    = { "Resources" };
    static char *readMeta_in[]     = { "Uri" };
    static char *readMeta_out[]    = { "Value" };
    static char *writeMeta_in[]    = { "Uri", "Value" };
    static char *writeMeta_out[]   = { "Status" };
    static char *getProp_in[]      = { "Key" };
    static char *getProp_out[]     = { "Value" };
    static char *setProp_in[]      = { "Key", "Value" };
    static char *setProp_out[]     = { "Status" };
    static char *commission_in[]   = { "SetupPayload", "Timeout" };
    static char *commission_out[]  = { "Status" };
    static char *openWin_in[]      = { "NodeId", "Timeout" };
    static char *openWin_out[]     = { "ManualCode", "QrCode" };

    USP_REGISTER_SyncOperation("Device.IoT.GetStatus()", IoT_Op_GetStatus);
    USP_REGISTER_OperationArguments("Device.IoT.GetStatus()",
                                     NULL, 0, getStatus_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Discovery.Start()", IoT_Op_DiscStart);
    USP_REGISTER_OperationArguments("Device.IoT.Discovery.Start()",
                                     discStart_in, 3, discStart_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Discovery.Stop()", IoT_Op_DiscStop);
    USP_REGISTER_OperationArguments("Device.IoT.Discovery.Stop()",
                                     NULL, 0, discStop_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Device.List()", IoT_Op_ListDevices);
    USP_REGISTER_OperationArguments("Device.IoT.Device.List()",
                                     listDev_in, 1, listDev_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Device.Get()", IoT_Op_GetDevice);
    USP_REGISTER_OperationArguments("Device.IoT.Device.Get()",
                                     getDev_in, 1, getDev_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Device.Remove()", IoT_Op_RemoveDevice);
    USP_REGISTER_OperationArguments("Device.IoT.Device.Remove()",
                                     removeDev_in, 1, removeDev_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Resource.Read()", IoT_Op_ReadResource);
    USP_REGISTER_OperationArguments("Device.IoT.Resource.Read()",
                                     readRes_in, 1, readRes_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Resource.Write()", IoT_Op_WriteResource);
    USP_REGISTER_OperationArguments("Device.IoT.Resource.Write()",
                                     writeRes_in, 2, writeRes_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Resource.Execute()", IoT_Op_ExecResource);
    USP_REGISTER_OperationArguments("Device.IoT.Resource.Execute()",
                                     execRes_in, 2, execRes_out, 2);

    USP_REGISTER_SyncOperation("Device.IoT.Resource.Query()", IoT_Op_QueryResources);
    USP_REGISTER_OperationArguments("Device.IoT.Resource.Query()",
                                     queryRes_in, 1, queryRes_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Metadata.Read()", IoT_Op_ReadMetadata);
    USP_REGISTER_OperationArguments("Device.IoT.Metadata.Read()",
                                     readMeta_in, 1, readMeta_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Metadata.Write()", IoT_Op_WriteMetadata);
    USP_REGISTER_OperationArguments("Device.IoT.Metadata.Write()",
                                     writeMeta_in, 2, writeMeta_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.GetProperty()", IoT_Op_GetProperty);
    USP_REGISTER_OperationArguments("Device.IoT.GetProperty()",
                                     getProp_in, 1, getProp_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.SetProperty()", IoT_Op_SetProperty);
    USP_REGISTER_OperationArguments("Device.IoT.SetProperty()",
                                     setProp_in, 2, setProp_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Matter.Commission()", IoT_Op_Commission);
    USP_REGISTER_OperationArguments("Device.IoT.Matter.Commission()",
                                     commission_in, 2, commission_out, 1);

    USP_REGISTER_SyncOperation("Device.IoT.Matter.OpenCommissioningWindow()",
                                 IoT_Op_OpenCommWindow);
    USP_REGISTER_OperationArguments("Device.IoT.Matter.OpenCommissioningWindow()",
                                     openWin_in, 2, openWin_out, 2);

    /* ---- Events ---- */
    static char *evtAddedArgs[]   = { "DeviceId", "Uri", "DeviceClass", "ClassVersion" };
    static char *evtRemovedArgs[] = { "DeviceId", "DeviceClass" };
    static char *evtResArgs[]     = { "Uri", "Value" };

    USP_REGISTER_Event(B_EVT_DEVICE_ADDED);
    USP_REGISTER_EventArguments(B_EVT_DEVICE_ADDED,     evtAddedArgs,   4);
    USP_REGISTER_Event(B_EVT_DEVICE_REMOVED);
    USP_REGISTER_EventArguments(B_EVT_DEVICE_REMOVED,   evtRemovedArgs, 2);
    USP_REGISTER_Event(B_EVT_RESOURCE_UPDATED);
    USP_REGISTER_EventArguments(B_EVT_RESOURCE_UPDATED, evtResArgs,     2);
    USP_REGISTER_Event(B_EVT_DISC_STARTED);
    USP_REGISTER_Event(B_EVT_DISC_STOPPED);
}

/* =========================================================================
 * Lifecycle
 * ======================================================================= */

static int IoT_VENDOR_Init(void)
{
    /* Close any stale handle left from a previous partial init */
    if (iot_rbus_handle != NULL)
    {
        rbus_close(iot_rbus_handle);
        iot_rbus_handle = NULL;
    }

    rbusError_t rc = rbus_open(&iot_rbus_handle, IOT_CONSUMER_COMPONENT);
    if (rc != RBUS_ERROR_SUCCESS)
    {
        fprintf(stderr, "[IoT] rbus_open(%s) failed rc=%d\n",
                IOT_CONSUMER_COMPONENT, rc);
        return USP_ERR_INTERNAL_ERROR;
    }
    fprintf(stderr, "[IoT] rbus_open OK handle=%p\n", (void*)iot_rbus_handle);

    IoT_RegisterDataModel();
    IoT_SubscribeEvents();

    fprintf(stdout, "[IoT] Device.IoT.* registered with obuspa"
                    " (4 props, 16 ops, 5 events)\n");
    return USP_ERR_OK;
}

static int IoT_VENDOR_Start(void)
{
    return USP_ERR_OK;
}

static int IoT_VENDOR_Stop(void)
{
    if (iot_rbus_handle != NULL)
    {
        rbusEvent_Unsubscribe(iot_rbus_handle, B_EVT_DEVICE_ADDED);
        rbusEvent_Unsubscribe(iot_rbus_handle, B_EVT_DEVICE_REMOVED);
        rbusEvent_Unsubscribe(iot_rbus_handle, B_EVT_RESOURCE_UPDATED);
        rbusEvent_Unsubscribe(iot_rbus_handle, B_EVT_DISC_STARTED);
        rbusEvent_Unsubscribe(iot_rbus_handle, B_EVT_DISC_STOPPED);
        rbus_close(iot_rbus_handle);
        iot_rbus_handle = NULL;
    }
    return USP_ERR_OK;
}

#endif /* __IOT_DATAMODEL_C__ */
