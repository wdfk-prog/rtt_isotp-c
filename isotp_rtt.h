/**
 * @file isotp_rtt.h
 * @brief Public API header for the RT-Thread adapter layer for the isotp-c library.
 *
 * This file defines the public interface for interacting with the ISO-TP adapter.
 * It provides functions to create, destroy, send, and receive ISO-TP messages
 * in a thread-safe and blocking manner suitable for an RTOS environment.
 * 
 * @author wdfk-prog ()
 * @version 1.1
 * @date 2025-11-08
 * 
 * @copyright Copyright (c) 2025  
 * 
 * @note :
 * @par 修改日志:
 * Date       Version Author      Description
 * 2025-11-08 1.0     wdfk-prog   first version
 * 2025-11-13 1.1     wdfk-prog   Implement non-blocking send functionality and optimize the blocking send interface
 */
#ifndef __ISOTP_RTT_H__
#define __ISOTP_RTT_H__

#include <rtthread.h>
#include <rtdevice.h>
#include "isotp-c/isotp.h"

/**
 * @name RT-Thread Adapter Specific Return Codes
 * @{
 * @brief These codes are returned by the adapter layer functions and supplement
 *        the standard ISOTP_RET_* codes from the core library.
 */
#define ISOTP_RET_INVAL_ARGS   -8  ///< Invalid arguments passed to the function (e.g., NULL link).
#define ISOTP_RET_TIMEOUT_RTT  -9  ///< Operation timed out (RT-Thread specific timeout).
#define ISOTP_RET_ERROR_RTT    -10 ///< An internal adapter-level error occurred.
/** @} */


/**
 * @brief Handle to an ISO-TP link instance.
 * @note  This is an opaque pointer; users should not attempt to access its members directly.
 */
typedef struct isotp_rtt_link* isotp_rtt_link_t;

/**
 * @brief Creates and initializes a new ISO-TP link instance.
 *
 * This function allocates resources for a new link, initializes the underlying isotp-c library,
 * and adds the link to the internal managed list.
 *
 * @warning This function is not thread-safe. The user must ensure that no other thread is
 *          iterating through the link list (by calling `isotp_rtt_on_can_msg_received`)
 *          at the same time this function is called. A mutex or scheduler lock should be
 *          used in the application layer if links can be created/destroyed dynamically
 *          while communication is active.
 *
 * @param can_dev           A handle to a previously opened RT-Thread CAN device.
 * @param send_arbitration_id  The CAN arbitration ID to use when transmitting frames for this link.
 * @param recv_arbitration_id  The CAN arbitration ID this link should listen to for incoming frames.
 * @param send_ide          The Identifier Extension type for outgoing frames (RT_CAN_STDID or RT_CAN_EXTID).
 * @param send_rtr          The Remote Transmission Request type for outgoing frames (RT_CAN_DTR or RT_CAN_RTR).
 * @param send_buf          A user-provided buffer for the protocol to use for formatting outgoing PDUs.
 * @param send_buf_size     The size of the send buffer in bytes.
 * @param recv_buf          A user-provided buffer for the protocol to use for assembling incoming PDUs.
 * @param recv_buf_size     The size of the receive buffer in bytes.
 *
 * @return A handle (`isotp_rtt_link_t`) to the newly created link on success, or RT_NULL on failure (e.g., memory allocation failed).
 */
isotp_rtt_link_t isotp_rtt_create(rt_device_t can_dev,
                                  uint32_t send_arbitration_id,
                                  uint32_t recv_arbitration_id,
                                  rt_uint8_t send_ide,
                                  rt_uint8_t send_rtr,
                                  uint8_t* send_buf,
                                  uint16_t send_buf_size,
                                  uint8_t* recv_buf,
                                  uint16_t recv_buf_size);

/**
 * @brief Destroys an ISO-TP link instance and releases all associated resources.
 *
 * This function removes the link from the managed list and frees all memory and
 * RTOS objects (events, mutexes) associated with it.
 *
 * @warning This function is not thread-safe. The user must ensure that no other thread is
 *          iterating through the link list (by calling `isotp_rtt_on_can_msg_received`)
 *          at the same time this function is called. A mutex or scheduler lock should be
 *          used in the application layer.
 *
 * @param link The handle of the link to be destroyed.
 */
void isotp_rtt_destroy(isotp_rtt_link_t link);

/**
 * @brief Processes a received CAN message and dispatches it to the appropriate link(s).
 *
 * This is the primary entry point for feeding raw CAN frames into the ISO-TP stack.
 * The function iterates through all active links and passes the message to any link
 * that is configured to listen to the message's arbitration ID.
 *
 * @warning This function MUST be called from a thread context (e.g., a dedicated consumer
 *          thread or a workqueue). It must NEVER be called directly from an Interrupt
 *          Service Routine (ISR). This is because the underlying protocol may need to
 *          transmit a response frame immediately (e.g., a Flow Control frame), which is
 *          a potentially blocking operation.
 *
 * @param msg A pointer to the `rt_can_msg` structure received from the CAN driver.
 */
void isotp_rtt_on_can_msg_received(struct rt_can_msg *msg);

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
int isotp_rtt_send(isotp_rtt_link_t link, const uint8_t *payload, uint16_t size, rt_int32_t timeout);

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
int isotp_rtt_send_nonblocking(isotp_rtt_link_t link, const uint8_t *payload, uint16_t size);

/**
 * @brief Receives a complete data payload (PDU) from an ISO-TP link in a blocking manner.
 *
 * This function blocks the calling thread until a complete message is received on the
 * specified link or a timeout occurs.
 *
 * @param link          The handle of the link to receive the message from.
 * @param payload_buf   A user-provided buffer to store the incoming data payload.
 * @param buf_size      The maximum size of the `payload_buf`.
 * @param out_size      A pointer to a variable where the actual size of the received data will be stored.
 * @param timeout       The maximum time to wait for a message to be received, in system ticks.
 *
 * @return Returns RT_EOK on success.
 * @retval -RT_EFULL if a complete message was received but was truncated because the link's internal
 *         receive buffer was too small. `out_size` will contain the amount of data that was copied.
 * @retval -RT_ENOMEM if the provided `payload_buf` is too small to hold the received data.
 * @retval -RT_ETIMEOUT if no message was received within the specified timeout.
 * @retval -RT_ERROR for other protocol-level errors.
 */
rt_err_t isotp_rtt_receive(isotp_rtt_link_t link, uint8_t* payload_buf, uint16_t buf_size, uint16_t* out_size, rt_int32_t timeout);

#endif // __ISOTP_RTT_H__