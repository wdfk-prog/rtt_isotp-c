/**
 * @file isotp_rtt.c
 * @brief RT-Thread adapter implementation for the upstream isotp-c library.
 * @ingroup isotp_rtt_internal
 *
 * This file implements the RT-Thread-facing transport adapter, including callback
 * shims required by isotp-c, event-driven synchronization, link bookkeeping, and
 * the background polling thread used to drive protocol timers.
 */

/**
 * @defgroup isotp_rtt_internal Internal Implementation
 * @ingroup isotp_rtt_pkg
 * @brief Internal data structures and helper functions used by the adapter.
 * @details Items in this group are documented to support maintenance and Doxygen
 *          browsing, but they are not intended to be used as a stable application API.
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
 * @brief Internal representation of one adapter link instance.
 * @ingroup isotp_rtt_internal
 *
 * This structure wraps the upstream `IsoTpLink` object and augments it with the
 * RT-Thread synchronization primitives and metadata required by the adapter layer.
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
 * @brief Head of the global list of active adapter links.
 * @ingroup isotp_rtt_internal
 */
static struct rt_list_node g_link_list_head = RT_LIST_OBJECT_INIT(g_link_list_head);

/**
 * @brief Atomically print a short hex dump for debug logs.
 * @ingroup isotp_rtt_internal
 * @note The helper assembles the complete log line in a temporary buffer before a
 *       single `LOG_D()` call so that concurrent threads do not interleave partial output.
 * @param title Descriptive title shown before the dump.
 * @param data Pointer to the data buffer to print.
 * @param size Number of bytes to print.
 */
void print_hex_data(const char *title, const uint8_t *data, uint16_t size)
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
 * @brief Shim used by the upstream library to transmit one CAN frame.
 * @ingroup isotp_rtt_internal
 * @param arbitration_id CAN identifier used for transmission.
 * @param data Pointer to a payload up to 8 bytes.
 * @param size Payload size in bytes.
 * @param user_send_can_arg Adapter-owned user argument, expected to point to `struct isotp_rtt_link`.
 * @return `ISOTP_RET_OK` on success, otherwise `ISOTP_RET_ERROR`.
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
 * @brief Provide a microsecond timestamp to the upstream ISO-TP stack.
 * @ingroup isotp_rtt_internal
 * @note This timestamp drives protocol timing such as STmin and timeout handling.
 * @return Monotonic timestamp in microseconds, truncated to 32 bits.
 */
uint32_t isotp_user_get_us(void)
{
    //TODO: 使用更高精度的API
    return (uint32_t)((rt_uint64_t)rt_tick_get() * 1000000 / RT_TICK_PER_SECOND);
}

/**
 * @brief Logging sink for upstream debug output.
 * @ingroup isotp_rtt_internal
 * @param format `printf`-style format string.
 * @param ... Format arguments.
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
 * @brief Transmission-complete callback registered with the upstream stack.
 * @ingroup isotp_rtt_internal
 * @note The callback only signals the waiting RT-Thread event object.
 * @param link_ptr Pointer to the upstream `IsoTpLink`.
 * @param size Size of the transmitted PDU.
 * @param user_arg Adapter-owned callback context.
 */
static void _isotp_rtt_tx_done_cb(void *link_ptr, uint32_t size, void *user_arg)
{
    struct isotp_rtt_link *rtt_link = (struct isotp_rtt_link *)user_arg;
    rt_event_send(&rtt_link->event, EVENT_FLAG_TX_DONE);
}

/**
 * @brief Reception-complete callback registered with the upstream stack.
 * @ingroup isotp_rtt_internal
 * @note The callback records the final receive size and signals the waiting thread.
 * @param link_ptr Pointer to the upstream `IsoTpLink`.
 * @param data Pointer to the assembled payload.
 * @param size Payload size in bytes.
 * @param user_arg Adapter-owned callback context.
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
 * @brief Entry point of the background polling thread.
 * @ingroup isotp_rtt_internal
 * @note The thread periodically calls `isotp_poll()` for each active link so that
 *       protocol timers, separation time handling, and timeout processing continue to run.
 * @param parameter Unused thread argument.
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
 * @brief Auto-initialization hook that starts the polling thread.
 * @ingroup isotp_rtt_internal
 * @note Registered through `INIT_APP_EXPORT`, so the adapter starts its polling thread
 *       automatically when the application initializes.
 * @return `RT_EOK` on success, otherwise `-RT_ERROR`.
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
 * @brief Sends an ISO-TP message in a blocking manner.
 * @note  This function initiates a transmission and then blocks the calling thread
 *        until the entire message is successfully sent or an error/timeout occurs.
 * @param  link The link handle.
 * @param  payload Pointer to the data to send.
 * @param  size Size of the data.
 * @param  timeout Timeout in system ticks.
 * @return Returns ISOTP_RET_OK on success.
 * @retval ISOTP_RET_TIMEOUT_RTT on timeout.
 * @retval ISOTP_RET_INVAL_ARGS if the link handle is invalid.
 * @retval Other ISOTP_RET_* codes for protocol-level errors.
 */
int isotp_rtt_send(isotp_rtt_link_t link, const uint8_t *payload, uint16_t size, rt_int32_t timeout)
{
    int ret = ISOTP_RET_OK;
    if (!link)
        return ISOTP_RET_INVAL_ARGS;

    rt_uint32_t recved_evt;

    rt_mutex_take(link->send_mutex, RT_WAITING_FOREVER);

    /* Clear any stale events before starting a new operation. */
    rt_event_recv(&link->event, EVENT_FLAG_TX_DONE | EVENT_FLAG_ERROR | EVENT_FLAG_RX_DONE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recved_evt);

    ret = isotp_send(&link->link, payload, size);
    if (ret != ISOTP_RET_OK)
    {
        LOG_E("isotp_send failed immediately with code: %d", ret);
    }
    else
    {
        if (rt_event_recv(&link->event, EVENT_FLAG_TX_DONE | EVENT_FLAG_ERROR, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout, &recved_evt) != RT_EOK)
        {
            LOG_W("isotp_rtt_send timeout.");
            ret = ISOTP_RET_TIMEOUT_RTT;
        }
        else if (recved_evt & EVENT_FLAG_ERROR)
        {
            LOG_E("isotp_rtt_send failed with an internal error event.");
            ret = ISOTP_RET_ERROR_RTT;
        }
    }

    rt_mutex_release(link->send_mutex);
    return ret;
}

/**
 * @brief Sends an ISO-TP message in a non-blocking manner ("fire and forget").
 * @note  This function queues the message for transmission and returns immediately.
 *        It does not wait for the transmission to complete. The user cannot know
 *        the final status of the transmission when using this function.
 *        It is suitable for applications that send data periodically without needing
 *        an immediate acknowledgment of transmission completion.
 * @param  link The link handle.
 * @param  payload Pointer to the data to send.
 * @param  size Size of the data.
 * @return Returns ISOTP_RET_OK if the message was successfully queued for sending.
 * @retval ISOTP_RET_INVAL_ARGS if the link handle is invalid.
 * @retval Other ISOTP_RET_* codes if the message could not be queued (e.g., another send is already in progress).
 */
int isotp_rtt_send_nonblocking(isotp_rtt_link_t link, const uint8_t *payload, uint16_t size)
{
    int ret = ISOTP_RET_OK;
    if (!link)
        return ISOTP_RET_INVAL_ARGS;

    /*
     * To ensure non-blocking behavior, we use a non-blocking mutex take (`timeout = 0`).
     * This attempts to acquire the lock. If another send operation (blocking or non-blocking)
     * is already in progress, it will fail immediately instead of waiting.
     */
    if (rt_mutex_take(link->send_mutex, 0) != RT_EOK)
    {
        /*
         * Another send is already in progress on this link. Return a specific error
         * to indicate that the resource is busy.
         */
        return ISOTP_RET_INPROGRESS;
    }

    /* Clear stale events. */
    rt_uint32_t recved_evt;
    rt_event_recv(&link->event, EVENT_FLAG_TX_DONE | EVENT_FLAG_ERROR | EVENT_FLAG_RX_DONE, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, &recved_evt);
    
    /*
     * Call the core library's send function.
     */
    ret = isotp_send(&link->link, payload, size);
    
    /*
     * IMPORTANT: We immediately release the mutex. The actual CAN frames will be sent
     * in the background by the isotp_poll thread. This function returns now,
     * without waiting for the TX_DONE event. If isotp_send() fails immediately
     * (e.g., wrong length), that error code will be returned. Otherwise, it will
     * return ISOTP_RET_OK, meaning the message is *queued*.
     */
    rt_mutex_release(link->send_mutex);
    return ret;
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
