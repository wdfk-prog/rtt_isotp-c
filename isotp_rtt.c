/**
 * @file isotp_rtt.c
 * @brief RT-Thread adapter layer for the isotp-c library (https://github.com/SimonCahill/isotp-c).
 *
 * This file provides a thread-safe, event-driven wrapper around the platform-agnostic
 * isotp-c library, making it easy to use within an RT-Thread environment.
 *
 * Key Features:
 * - Manages multiple ISO-TP links concurrently.
 * - Provides blocking, thread-safe send and receive APIs.
 * - Handles protocol timing and state machines in a dedicated background thread.
 * - Decouples CAN ISR from protocol processing.
 * - Supports unique naming for RTOS objects for easier debugging.
 * 
 * @author wdfk-prog ()
 * @version 1.0
 * @date 2025-11-05
 * 
 * @copyright Copyright (c) 2025  
 * 
 * @note :
 * @par 修改日志:
 * Date       Version Author      Description
 * 2025-11-05 1.0     wdfk-prog   first version
 */
#include "isotp_rtt.h"
#include <string.h>

#define DBG_TAG "isotp.rtt"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* Private Defines */
#define EVENT_FLAG_TX_DONE (1 << 0) ///< Event flag: A complete PDU has been successfully transmitted.
#define EVENT_FLAG_RX_DONE (1 << 1) ///< Event flag: A complete PDU has been successfully received.
#define EVENT_FLAG_ERROR   (1 << 2) ///< Event flag: An error occurred during transmission or reception.

/**
 * @brief Internal structure representing a single ISO-TP link instance tailored for RT-Thread.
 *
 * This structure encapsulates the core IsoTpLink object and adds RT-Thread specific
 * resources like events for synchronization and mutexes for thread safety.
 */
struct isotp_rtt_link
{
    IsoTpLink link;                 ///< The underlying isotp-c library link instance.
    rt_device_t can_dev;            ///< The associated RT-Thread CAN device for this link.
    uint32_t recv_arbitration_id;   ///< The CAN arbitration ID this link listens to for incoming messages.
    rt_uint8_t send_ide;            ///< The CAN ID type (Standard/Extended) to use for sending frames.
    rt_uint8_t send_rtr;            ///< The CAN frame type (Data/Remote) to use for sending frames.

    struct rt_event event;          ///< Event set for synchronizing blocking API calls with asynchronous callbacks.
    rt_mutex_t send_mutex;          ///< Mutex to ensure thread-safe sending on this specific link.

    /* Receive buffer information, provided by the user during creation */
    uint8_t *rx_buf_ptr;            ///< Pointer to the user-provided buffer for assembling incoming PDUs.
    uint16_t rx_buf_size;           ///< The total size of the user-provided receive buffer.
    uint16_t rx_actual_size;        ///< The actual size of the last received PDU.
    rt_bool_t rx_truncated;         ///< Flag indicating if the last received PDU was truncated.

    struct rt_list_node node;       ///< Node for linking this instance into the global list of links.
};

/* Global Resources */
/**
 * @brief Head of the global linked list that manages all active isotp_rtt_link instances.
 */
static struct rt_list_node g_link_list_head = RT_LIST_OBJECT_INIT(g_link_list_head);

/**
 * @brief Helper function to atomically print a title and hex data using ULOG.
 * @note  This function constructs a complete string in a temporary buffer before
 *        making a single call to LOG_D. This prevents log messages from being
 *        interleaved by other threads, which can happen if rt_kprintf and ulog_hexdump
 *        are called separately.
 * @param title A descriptive title for the hex data.
 * @param data  Pointer to the data buffer to be printed.
 * @param size  The size of the data in bytes.
 */
static void print_hex_data(const char *title, const uint8_t *data, uint16_t size)
{
#if (DBG_LVL >= DBG_LOG)
    char log_buf[256];
    int offset = 0;
    offset += rt_snprintf(log_buf + offset, sizeof(log_buf) - offset, "%s [%d bytes]:", title, size);

    for (uint16_t i = 0; i < size; i++)
    {
        if (offset >= sizeof(log_buf) - 4)
        {
            rt_snprintf(log_buf + offset, sizeof(log_buf) - offset, " ...");
            break;
        }
        offset += rt_snprintf(log_buf + offset, sizeof(log_buf) - offset, " %02X", data[i]);
    }

    LOG_D(log_buf);
#endif
}


/*************************************************************************************************/
/** @name Shim Functions for isotp-c
 *  @{
 *  @brief These functions are the required callbacks (dependency injection) that bridge
 *         the platform-agnostic isotp-c library with the RT-Thread operating system.
 */
/*************************************************************************************************/

/**
 * @brief  Sends a single CAN frame. This is called by the isotp-c library whenever
 *         it needs to transmit a protocol frame (FF, CF, FC).
 * @param  arbitration_id The CAN ID for the message to be sent.
 * @param  data Pointer to the 8-byte (or less) data payload.
 * @param  size The size of the data payload (0-8 bytes).
 * @param  user_send_can_arg The user-defined argument, which we use to pass our isotp_rtt_link struct.
 * @return ISOTP_RET_OK on success, ISOTP_RET_ERROR on failure.
 */
int isotp_user_send_can(const uint32_t arbitration_id, const uint8_t *data, const uint8_t size, void *user_send_can_arg)
{
    struct isotp_rtt_link *rtt_link = (struct isotp_rtt_link *)user_send_can_arg;
    struct rt_can_msg msg;

    if (!rtt_link || !rtt_link->can_dev)
        return ISOTP_RET_ERROR;

    msg.id = arbitration_id;
    msg.ide = rtt_link->send_ide;
    msg.rtr = rtt_link->send_rtr;
    msg.len = size;
    rt_memcpy(msg.data, data, size);

#if (DBG_LVL >= DBG_LOG)
    {
        char title_buf[32];
        rt_snprintf(title_buf, sizeof(title_buf), "[TX] ID: 0x%lX", arbitration_id);
        print_hex_data(title_buf, msg.data, size);
    }
#endif

    return (rt_device_write(rtt_link->can_dev, 0, &msg, sizeof(msg)) == sizeof(msg)) ? ISOTP_RET_OK : ISOTP_RET_ERROR;
}

/**
 * @brief  Provides a microsecond-resolution timestamp to the isotp-c library.
 * @note   This is critical for protocol timing (e.g., timeouts, STmin).
 * @return A 32-bit microsecond timestamp.
 */
uint32_t isotp_user_get_us(void)
{
    //TODO: 使用更高精度的API
    return (uint32_t)((rt_uint64_t)rt_tick_get() * 1000000 / RT_TICK_PER_SECOND);
}

/**
 * @brief  Acts as the logging sink for the isotp-c library's internal debug messages.
 * @param  format The format string (printf-style).
 * @param  ... Variadic arguments for the format string.
 */
void isotp_user_debug(const char *format, ...)
{
#if defined(RT_USING_ULOG) && defined(ULOG_BACKEND_USING_CONSOLE)
    va_list args;
    va_start(args, format);
    ulog_voutput(DBG_LVL, DBG_TAG, RT_TRUE, RT_NULL, 0, 0, 0, format, args);
    va_end(args);
#else
    va_list args;
    rt_kprintf("[%s/D] ", DBG_TAG);
    va_start(args, format);
    rt_vprintf(format, args);
    va_end(args);
    rt_kprintf("\n");
#endif
}
/** @} */


/*************************************************************************************************/
/** @name Internal Event Callbacks
 *  @{
 *  @brief These static functions are called by the isotp-c library upon completion of
 *         asynchronous operations. Their primary role is to signal waiting threads.
 */
/*************************************************************************************************/

/**
 * @brief  Called by isotp-c when a complete PDU has been transmitted successfully.
 * @note   Its sole purpose is to post an event to unblock any thread waiting in `isotp_rtt_send`.
 * @param  link_ptr A pointer to the core IsoTpLink instance.
 * @param  size The size of the PDU that was sent.
 * @param  user_arg The user argument, which points to our isotp_rtt_link struct.
 */
static void _isotp_rtt_tx_done_cb(void *link_ptr, uint32_t size, void *user_arg)
{
    struct isotp_rtt_link *rtt_link = (struct isotp_rtt_link *)user_arg;
    rt_event_send(&rtt_link->event, EVENT_FLAG_TX_DONE);
}

/**
 * @brief  Called by isotp-c when a complete PDU has been received and assembled.
 * @note   This function only needs to record the final size and post an event to unblock
 *         any thread waiting in `isotp_rtt_receive`.
 * @param  link_ptr A pointer to the core IsoTpLink instance.
 * @param  data Pointer to the start of the received data.
 * @param  size The size of the fully assembled PDU.
 * @param  user_arg The user argument, which points to our isotp_rtt_link struct.
 */
static void _isotp_rtt_rx_done_cb(void *link_ptr, const uint8_t *data, uint32_t size, void *user_arg)
{
    struct isotp_rtt_link *rtt_link = (struct isotp_rtt_link *)user_arg;

    uint16_t final_size = size;
    rtt_link->rx_truncated = RT_FALSE;

    if (size > rtt_link->rx_buf_size)
    {
        final_size = rtt_link->rx_buf_size;
        rtt_link->rx_truncated = RT_TRUE;
        LOG_W("RX buffer truncated! Link[0x%p] received %d bytes, but buffer size is %d.", rtt_link, size, rtt_link->rx_buf_size);
    }
    rtt_link->rx_actual_size = final_size;
    rt_event_send(&rtt_link->event, EVENT_FLAG_RX_DONE);
}
/** @} */


/*************************************************************************************************/
/** @name Internal Polling Thread
 *  @{
 */
/*************************************************************************************************/

/**
 * @brief  The entry point for the background polling thread.
 * @note   This thread is crucial. It periodically calls `isotp_poll()` for every active
 *         link. `isotp_poll()` is responsible for handling all time-dependent aspects
 *         of the protocol, such as message timeouts and separation time delays (STmin).
 * @param  parameter Unused.
 */
static void _poll_thread_entry(void *parameter)
{
    struct isotp_rtt_link *rtt_link, *next_rtt_link;
    while (1)
    {
        rt_list_for_each_entry_safe(rtt_link, next_rtt_link, &g_link_list_head, node)
        {
            isotp_poll(&rtt_link->link);
        }
        rt_thread_mdelay(PKG_ISOTP_C_POLL_INTERVAL_MS);
    }
}

/**
 * @brief  Auto-initialization function for the adapter layer.
 * @note   This function is called automatically by the RT-Thread INIT_APP_EXPORT mechanism.
 *         Its only job is to create and start the background polling thread.
 * @return RT_EOK on success, -RT_ERROR on failure.
 */
static int _isotp_rtt_init(void)
{
    rt_thread_t tid = rt_thread_create("isotp_poll",
                                       _poll_thread_entry,
                                       RT_NULL,
                                       PKG_ISOTP_C_POLL_THREAD_STACK_SIZE,
                                       PKG_ISOTP_C_POLL_THREAD_PRIORITY,
                                       10);
    if (tid)
    {
        rt_thread_startup(tid);
    }
    else
    {
        LOG_E("Failed to create isotp_poll thread.");
        return -RT_ERROR;
    }
    return RT_EOK;
}
INIT_APP_EXPORT(_isotp_rtt_init);
/** @} */


/*************************************************************************************************/
/** @name Public API Implementation
 *  @{
 *  @brief The public functions exposed to the user application.
 */
/*************************************************************************************************/

/**
 * @brief  Processes a received CAN message.
 * @warning This function MUST be called from a thread context (e.g., a workqueue or a dedicated
 *          consumer thread), NOT from a CAN ISR. This is because `isotp_on_can_message` may
 *          trigger an immediate transmission (e.g., a Flow Control frame), which is a
 *          blocking operation unsuitable for an ISR.
 * @param  msg A pointer to the received `rt_can_msg` structure.
 */
void isotp_rtt_on_can_msg_received(struct rt_can_msg *msg)
{
    struct isotp_rtt_link *rtt_link, *next_rtt_link;

#if (DBG_LVL >= DBG_LOG)
    {
        char title_buf[32];
        rt_snprintf(title_buf, sizeof(title_buf), "[RX] ID: 0x%X", msg->id);
        print_hex_data(title_buf, msg->data, msg->len);
    }
#endif

    /* Iterate through all active links and dispatch the message to any link listening on this ID. */
    rt_list_for_each_entry_safe(rtt_link, next_rtt_link, &g_link_list_head, node)
    {
        if (rtt_link->recv_arbitration_id == msg->id)
        {
            isotp_on_can_message(&rtt_link->link, msg->data, msg->len);
            /* Do not break; multiple links might be listening to the same ID. */
        }
    }
}

/**
 * @brief  Creates and initializes a new ISO-TP link instance.
 * @param  can_dev The user-opened RT-Thread CAN device handle.
 * @param  send_arbitration_id The CAN ID to use for sending.
 * @param  recv_arbitration_id The CAN ID to listen for.
 * @param  send_ide CAN ID type (RT_CAN_STDID or RT_CAN_EXTID).
 * @param  send_rtr CAN frame type (RT_CAN_DTR or RT_CAN_RTR).
 * @param  send_buf User-provided buffer for outgoing PDUs.
 * @param  send_buf_size Size of the send buffer.
 * @param  recv_buf User-provided buffer for incoming PDUs.
 * @param  recv_buf_size Size of the receive buffer.
 * @return A handle to the new link, or RT_NULL on failure.
 */
isotp_rtt_link_t isotp_rtt_create(rt_device_t can_dev,
                                  uint32_t send_arbitration_id,
                                  uint32_t recv_arbitration_id,
                                  rt_uint8_t send_ide,
                                  rt_uint8_t send_rtr,
                                  uint8_t *send_buf,
                                  uint16_t send_buf_size,
                                  uint8_t *recv_buf,
                                  uint16_t recv_buf_size)
{
    if (!can_dev)
    {
        LOG_E("CAN device handle cannot be NULL.");
        return RT_NULL;
    }

    struct isotp_rtt_link *rtt_link = rt_malloc(sizeof(struct isotp_rtt_link));
    if (!rtt_link)
    {
        LOG_E("Failed to allocate memory for rtt_link.");
        return RT_NULL;
    }
    rt_memset(rtt_link, 0, sizeof(struct isotp_rtt_link));

    rtt_link->can_dev = can_dev;
    rtt_link->recv_arbitration_id = recv_arbitration_id;
    rtt_link->send_ide = send_ide;
    rtt_link->send_rtr = send_rtr;
    rtt_link->rx_buf_ptr = recv_buf;
    rtt_link->rx_buf_size = recv_buf_size;

    char event_name[RT_NAME_MAX];
    char mutex_name[RT_NAME_MAX];
    rt_snprintf(event_name, RT_NAME_MAX, "isotp_evt_%lx", recv_arbitration_id);
    rt_snprintf(mutex_name, RT_NAME_MAX, "isotp_tx_mtx_%lx", send_arbitration_id);

    rt_event_init(&rtt_link->event, event_name, RT_IPC_FLAG_FIFO);
    rtt_link->send_mutex = rt_mutex_create(mutex_name, RT_IPC_FLAG_FIFO);

    isotp_init_link(&rtt_link->link, send_arbitration_id, send_buf, send_buf_size, recv_buf, recv_buf_size);
    rtt_link->link.user_send_can_arg = rtt_link;

    isotp_set_tx_done_cb(&rtt_link->link, _isotp_rtt_tx_done_cb, rtt_link);
    isotp_set_rx_done_cb(&rtt_link->link, _isotp_rtt_rx_done_cb, rtt_link);

    rt_list_insert_after(&g_link_list_head, &rtt_link->node);

    LOG_I("ISO-TP link created for device:%s, SID:0x%X, RID:0x%X", can_dev->parent.name, send_arbitration_id, recv_arbitration_id);
    return rtt_link;
}

/**
 * @brief  Destroys an ISO-TP link and releases its resources.
 * @param  link The link handle to destroy.
 */
void isotp_rtt_destroy(isotp_rtt_link_t link)
{
    if (!link)
        return;
    rt_list_remove(&link->node);
    rt_event_detach(&link->event);
    rt_mutex_delete(link->send_mutex);
    rt_free(link);
    LOG_I("ISO-TP link destroyed.");
}

/**
 * @brief  Sends an ISO-TP message in a blocking manner.
 * @note   This function initiates a non-blocking send via `isotp_send` and then
 *         blocks the calling thread by waiting on an event. The event is posted
 *         by the `_isotp_rtt_tx_done_cb` callback upon successful transmission,
 *         or by an error condition.
 * @param  link The link handle.
 * @param  payload Pointer to the data to send.
 * @param  size Size of the data.
 * @param  timeout Timeout in system ticks.
 * @return RT_EOK on success, -RT_ETIMEOUT on timeout, -RT_ERROR on other failures.
 */
rt_err_t isotp_rtt_send(isotp_rtt_link_t link, const uint8_t *payload, uint16_t size, rt_int32_t timeout)
{
    if (!link)
        return -RT_EINVAL;

    rt_err_t result = RT_EOK;
    rt_uint32_t recved_evt;

    rt_mutex_take(link->send_mutex, RT_WAITING_FOREVER);

    /* Clear any stale events before starting a new operation. */
    rt_event_recv(&link->event, EVENT_FLAG_TX_DONE | EVENT_FLAG_ERROR | EVENT_FLAG_RX_DONE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recved_evt);

    int ret = isotp_send(&link->link, payload, size);
    if (ret != ISOTP_RET_OK)
    {
        LOG_E("isotp_send failed immediately with code: %d", ret);
        result = -RT_ERROR;
    }
    else
    {
        /* Block until the TX_DONE or ERROR event is received, or until timeout. */
        if (rt_event_recv(&link->event, EVENT_FLAG_TX_DONE | EVENT_FLAG_ERROR, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout, &recved_evt) != RT_EOK)
        {
            LOG_W("isotp_rtt_send timeout.");
            result = -RT_ETIMEOUT;
        }
        else if (recved_evt & EVENT_FLAG_ERROR)
        {
            LOG_E("isotp_rtt_send failed with an internal error event.");
            result = -RT_ERROR;
        }
    }

    rt_mutex_release(link->send_mutex);
    return result;
}

/**
 * @brief  Receives an ISO-TP message in a blocking manner.
 * @note   This function blocks the calling thread by waiting for the `EVENT_FLAG_RX_DONE` event,
 *         which is posted by the `_isotp_rtt_rx_done_cb` callback when a complete PDU
 *         has been assembled into the link's internal buffer. It then copies the data
 *         from the internal buffer to the user-provided `payload_buf`.
 * @param  link The link handle.
 * @param  payload_buf Buffer to store the received data.
 * @param  buf_size Size of the `payload_buf`.
 * @param  out_size Pointer to store the actual size of the received data.
 * @param  timeout Timeout in system ticks.
 * @return RT_EOK on success, -RT_EFULL if the PDU was truncated, -RT_ENOMEM if `payload_buf` is too small,
 *         -RT_ETIMEOUT on timeout, -RT_ERROR on other failures.
 */
rt_err_t isotp_rtt_receive(isotp_rtt_link_t link, uint8_t *payload_buf, uint16_t buf_size, uint16_t *out_size, rt_int32_t timeout)
{
    if (!link || !payload_buf || !out_size)
        return -RT_EINVAL;

    rt_uint32_t recved_evt;
    if (rt_event_recv(&link->event, EVENT_FLAG_RX_DONE | EVENT_FLAG_ERROR, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout, &recved_evt) != RT_EOK)
    {
        *out_size = 0;
        return -RT_ETIMEOUT;
    }

    if (recved_evt & EVENT_FLAG_RX_DONE)
    {
        uint16_t copy_size = link->rx_actual_size;

        if (copy_size > buf_size)
        {
            LOG_E("User receive buffer is too small! Required: %d, Provided: %d", copy_size, buf_size);
            *out_size = 0;
            return -RT_ENOMEM;
        }

        rt_memcpy(payload_buf, link->rx_buf_ptr, copy_size);
        *out_size = copy_size;
        return link->rx_truncated ? -RT_EFULL : RT_EOK;
    }
    else /* (recved_evt & EVENT_FLAG_ERROR) */
    {
        *out_size = 0;
        return -RT_ERROR;
    }
}
/** @} */
