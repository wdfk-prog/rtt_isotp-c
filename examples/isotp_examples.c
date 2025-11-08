/**
 * @file isotp_examples.c
 * @brief An advanced example demonstrating the usage of the isotp-c RT-Thread adapter.
 *
 * This file provides a comprehensive test suite for the ISO-TP adapter layer,
 * showcasing a client-server model, multi-link handling, and robust resource management.
 * It is intended to be run as an MSH command (`isotp_example start`/`stop`).
 *
 * Key features demonstrated:
 * - Communication between two CAN buses (can1 and can2).
 * - A producer-consumer model for safe, non-blocking CAN message handling from ISRs.
 * - Multi-frame message transmission and reception.
 * - Automatic verification of received data.
 * - Handling of one CAN ID being monitored by multiple links (server and logger).
 * - Proper resource allocation, cleanup, and restoration of the CAN device's original state.
 * - Configuration of CAN baud rate and hardware filters.
 * @author wdfk-prog ()
 * @version 1.0
 * @date 2025-11-08
 * 
 * @copyright Copyright (c) 2025  
 * 
 * @note :
 * @par 修改日志:
 * Date       Version Author      Description
 * 2025-11-08 1.0     wdfk-prog   first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "isotp_rtt.h"

/* We set the debug level to WARNING to keep the console clean during normal operation. */
/* To see detailed TX/RX logs, set this to DBG_LOG in your local copy. */
#define DBG_TAG "isotp.example"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/* --- Example Configuration --- */
#define CAN1_DEV_NAME "can1" ///< CAN bus used by the client (tester).
#define CAN2_DEV_NAME "can2" ///< CAN bus used by the server (ECU) and logger.

#define CLIENT_SEND_ID 0x7E0  ///< CAN ID for client requests.
#define CLIENT_RECV_ID 0x7E8  ///< CAN ID for client to receive responses on.
#define SERVER_SEND_ID 0x7E8  ///< CAN ID for server responses.
#define SERVER_RECV_ID 0x7E0  ///< CAN ID for server to receive requests on.

/* --- RTOS Object Configuration --- */
#define CAN_RX_MQ_SIZE          32     ///< The message queue can buffer up to 32 incoming CAN frames.
#define RX_CONSUMER_THREAD_PRIO 15 ///< Priority for the thread that processes incoming CAN frames.

/* --- Global Resources --- */
/* Static buffers for ISO-TP links */
static uint8_t client_send_buf[256], client_recv_buf[256];
static uint8_t server_send_buf[256], server_recv_buf[256];
static uint8_t logger_recv_buf[256];

/* Handles for RTOS objects and devices */
static rt_bool_t is_running = RT_FALSE;
static rt_device_t can1_dev, can2_dev;
static rt_mq_t can_rx_mq;
static rt_thread_t rx_consumer_tid;
static rt_thread_t client_tid, server_tid, logger_tid;

/* --- Context Saving --- */
/** @brief Typedef for the CAN RX callback function pointer for readability. */
typedef rt_err_t (*can_rx_indicate_func_t)(rt_device_t dev, rt_size_t size);
/** @brief Stores the original RX callback of can1 to restore it on exit. */
static can_rx_indicate_func_t old_can1_rx_indicate = RT_NULL;
/** @brief Stores the original RX callback of can2 to restore it on exit. */
static can_rx_indicate_func_t old_can2_rx_indicate = RT_NULL;

/**
 * @brief Helper function to atomically print a title and hex data using ULOG.
 * @note  This function is only compiled in if DBG_LVL is set to DBG_LOG or lower.
 *        It constructs the entire log message in a local buffer before making a
 *        single call to LOG_D to prevent interleaving from other threads.
 */
void print_hex_data(const char *title, const uint8_t *data, uint16_t size)
{
#if (DBG_LVL <= DBG_LOG)
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
/** @name Producer-Consumer Model for CAN Reception
 *  @{
 *  @brief This is the recommended pattern for handling high-frequency hardware interrupts.
 *         The ISR (producer) does the minimal work of pushing data into a queue.
 *         A dedicated thread (consumer) then processes this data safely in the thread context.
 */
/*************************************************************************************************/

/**
 * @brief The consumer thread. It blocks indefinitely waiting for messages on a queue.
 * @param parameter Unused.
 */
static void can_rx_consumer_thread_entry(void *parameter)
{
    struct rt_can_msg msg;
    while (1)
    {
        /* Block until a CAN message is received from the message queue. */
        if (rt_mq_recv(can_rx_mq, &msg, sizeof(struct rt_can_msg), RT_WAITING_FOREVER) == sizeof(struct rt_can_msg))
        {
            /* Process the message in thread context. */
            isotp_rtt_on_can_msg_received(&msg);
        }
    }
}

/**
 * @brief The producer. This is the CAN RX callback function, executed in ISR context.
 * @note  This function must be extremely fast and non-blocking. Its only job is to
 *        read the CAN frame and post it to the message queue.
 * @param dev The device that triggered the interrupt.
 * @param size Unused.
 * @return RT_EOK.
 */
static rt_err_t can_rx_callback(rt_device_t dev, rt_size_t size)
{
    struct rt_can_msg msg;
    /* Initialize hdr_index to -1 to receive from any hardware filter bank. */
    msg.hdr_index = -1;
    if (rt_device_read(dev, 0, &msg, sizeof(msg)) == sizeof(msg))
    {
        /* Send the message to the queue. This is a non-blocking, ISR-safe operation. */
        if (rt_mq_send(can_rx_mq, &msg, sizeof(struct rt_can_msg)) == -RT_EFULL)
        {
            LOG_W("CAN RX message queue is full, message dropped.");
        }
    }
    return RT_EOK;
}
/** @} */


/*************************************************************************************************/
/** @name Example Application Threads
 *  @{
 *  @brief These threads simulate a real-world diagnostic scenario.
 */
/*************************************************************************************************/

/**
 * @brief Simulates an ECU (server) that receives commands and sends responses.
 */
static void server_thread_entry(void *param)
{
    LOG_I("[Server] ECU thread started on %s.", CAN2_DEV_NAME);
    isotp_rtt_link_t server_link = isotp_rtt_create(can2_dev, SERVER_SEND_ID, SERVER_RECV_ID, RT_CAN_STDID, RT_CAN_DTR, server_send_buf, sizeof(server_send_buf), server_recv_buf, sizeof(server_recv_buf));
    if (!server_link)
    {
        LOG_E("[Server] Failed to create link.");
        return;
    }

    uint8_t rx_payload[128];
    uint16_t received_size = 0;
    while (1)
    {
        rt_err_t result = isotp_rtt_receive(server_link, rx_payload, sizeof(rx_payload), &received_size, RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            /* Simulate a UDS positive response by adding 0x40 to the service ID. */
            rx_payload[0] += 0x40;
            isotp_rtt_send(server_link, rx_payload, received_size, RT_WAITING_FOREVER);
            print_hex_data("[Server] Sending Response", rx_payload, received_size);
        }
    }
}

/**
 * @brief Simulates a CAN bus logger that passively listens for and prints messages.
 * @note  This demonstrates that multiple links can listen to the same CAN ID.
 */
static void logger_thread_entry(void *param)
{
    LOG_I("[Logger] Logger thread started on %s.", CAN2_DEV_NAME);
    isotp_rtt_link_t logger_link = isotp_rtt_create(can2_dev, 0, SERVER_RECV_ID, RT_CAN_STDID, RT_CAN_DTR, RT_NULL, 0, logger_recv_buf, sizeof(logger_recv_buf));
    if (!logger_link)
    {
        LOG_E("[Logger] Failed to create link.");
        return;
    }

    uint16_t received_size = 0;
    while (1)
    {
        if (isotp_rtt_receive(logger_link, logger_recv_buf, sizeof(logger_recv_buf), &received_size, RT_WAITING_FOREVER) == RT_EOK)
        {
            print_hex_data("[Logger] Logged Command", logger_recv_buf, received_size);
        }
    }
}

/**
 * @brief Simulates a diagnostic tool (client) that sends commands and verifies responses.
 */
static void client_thread_entry(void *param)
{
    LOG_I("[Client] Tester thread started on %s.", CAN1_DEV_NAME);
    isotp_rtt_link_t client_link = isotp_rtt_create(can1_dev, CLIENT_SEND_ID, CLIENT_RECV_ID, RT_CAN_STDID, RT_CAN_DTR, client_send_buf, sizeof(client_send_buf), client_recv_buf, sizeof(client_recv_buf));
    if (!client_link)
    {
        LOG_E("[Client] Failed to create link.");
        return;
    }

    rt_thread_mdelay(1000); // Wait for other threads to initialize.
    rt_uint32_t count = 0;
    uint8_t request_payload[20];

    while (1)
    {
        LOG_I("------------------- Client Test Case %ld -------------------", count++);

        /* Prepare a new UDS request payload for each iteration. */
        request_payload[0] = 0x22; // UDS: Read Data By Identifier
        for (int i = 1; i < sizeof(request_payload); i++)
        {
            request_payload[i] = count + i;
        }

        print_hex_data("[Client] Sending Command", request_payload, sizeof(request_payload));

        /* Send the request and wait for the operation to complete. */
        if (isotp_rtt_send(client_link, request_payload, sizeof(request_payload), (200 * RT_TICK_PER_SECOND / 1000)) == RT_EOK)
        {
            uint8_t response_buf[128];
            uint16_t response_size = 0;

            /* Wait to receive the response from the server. */
            rt_err_t result = isotp_rtt_receive(client_link, response_buf, sizeof(response_buf), &response_size, (200 * RT_TICK_PER_SECOND / 1000));

            if (result == RT_EOK)
            {
                print_hex_data("[Client] Received Response", response_buf, response_size);

                /* The client's responsibility is to verify the response. */
                if (response_size == sizeof(request_payload) && response_buf[0] == (request_payload[0] + 0x40))
                {
                    if (rt_memcmp(&response_buf[1], &request_payload[1], response_size - 1) == 0)
                    {
                        LOG_I("[Client] VERIFICATION SUCCESS: Response is valid!");
                    }
                    else
                    {
                        LOG_E("[Client] VERIFICATION FAILED: Response data mismatch!");
                    }
                }
                else
                {
                    LOG_E("[Client] VERIFICATION FAILED: Response size or SID is incorrect!");
                }
            }
            else
            {
                LOG_E("[Client] Failed to receive response. Error code: %d", result);
            }
        }
        else
        {
            LOG_E("[Client] Command send failed.");
            return; /* Exit thread on send failure. */
        }
    }
}
/** @} */


/*************************************************************************************************/
/** @name MSH Command Implementation
 *  @{
 *  @brief Provides `start` and `stop` commands to manage the example's lifecycle.
 */
/*************************************************************************************************/

/**
 * @brief Starts the ISO-TP example.
 * @note  This function performs all necessary initialization: finding devices,
 *        creating RTOS objects, configuring CAN hardware, and starting threads.
 *        It carefully saves the original CAN device context before modifying it.
 */
static void isotp_example_start(void)
{
    if (is_running)
    {
        rt_kprintf("ISOTP example is already running.\n");
        return;
    }

    /* 1. Find CAN devices. */
    can1_dev = rt_device_find(CAN1_DEV_NAME);
    can2_dev = rt_device_find(CAN2_DEV_NAME);
    if (!can1_dev || !can2_dev)
    {
        LOG_E("Please ensure both '%s' and '%s' CAN devices are enabled.", CAN1_DEV_NAME, CAN2_DEV_NAME);
        return;
    }

    /* 
     * 2. Save original device context. This is crucial for being a good "citizen" in the system.
     *    We directly access the rx_indicate member.
     *    Note: This direct access assumes knowledge of the rt_device structure.
     */
    old_can1_rx_indicate = can1_dev->rx_indicate;
    old_can2_rx_indicate = can2_dev->rx_indicate;

    /* Some drivers might require the device to be closed before changing settings. */
    rt_device_close(can1_dev);
    rt_device_close(can2_dev);

    /* 3. Create IPC objects and the consumer thread. */
    can_rx_mq = rt_mq_create("can_rx_mq", sizeof(struct rt_can_msg), CAN_RX_MQ_SIZE, RT_IPC_FLAG_FIFO);
    if (!can_rx_mq)
    {
        LOG_E("Failed to create can_rx_mq.");
        return;
    }

    rx_consumer_tid = rt_thread_create("isotp_consumer", can_rx_consumer_thread_entry, RT_NULL, 2048, RX_CONSUMER_THREAD_PRIO, 10);
    if (rx_consumer_tid)
    {
        rt_thread_startup(rx_consumer_tid);
    }
    else
    {
        rt_mq_delete(can_rx_mq);
        LOG_E("Failed to create consumer thread.");
        return;
    }

    /* 4. Open devices and configure hardware. */
    rt_device_open(can1_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
    rt_device_open(can2_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);

    /* 4a. Configure hardware filters. */
#ifdef RT_CAN_USING_HDR
    struct rt_can_filter_item items[] = {
        {
            .id = 0,                    // 当掩码为0时, ID可以是任意值, 0最清晰
            .ide = RT_CAN_STDID,        // 对标准帧生效
            .rtr = RT_CAN_DTR,          // 对数据帧生效
            .mode = RT_CAN_MODE_MASK,   // 0: 掩码模式
            .mask = 0,                  // 核心: 掩码为0, 接收所有ID
            .hdr_bank = -1,             // 由驱动自动分配 filter bank
            .rxfifo = CAN_RX_FIFO0,     // 如果您的BSP定义了此宏, 可以指定FIFO
        },
    };

    struct rt_can_filter_config cfg = {
        .count = 1,     /* 过滤器条目数量 */
        .actived = 1,   /* 激活本配置 */
        .items = items  /* 指向我们的规则数组 */
    };

    rt_device_control(can1_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    rt_device_control(can2_dev, RT_CAN_CMD_SET_FILTER, &cfg);
#endif

    /* 4b. Configure baud rate and mode. */
    rt_device_control(can1_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    rt_device_control(can2_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    rt_device_control(can1_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    rt_device_control(can2_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);

    /* 5. Set our new RX callback. */
    LOG_I("Setting up new rx_indicate callbacks...");
    rt_device_set_rx_indicate(can1_dev, can_rx_callback);
    rt_device_set_rx_indicate(can2_dev, can_rx_callback);

    /* 6. Create and start application threads. */
    server_tid = rt_thread_create("isotp_server", server_thread_entry, RT_NULL, 2048, 22, 10);
    if (server_tid)
        rt_thread_startup(server_tid);

    logger_tid = rt_thread_create("isotp_logger", logger_thread_entry, RT_NULL, 2048, 23, 10);
    if (logger_tid) rt_thread_startup(logger_tid);

    client_tid = rt_thread_create("isotp_client", client_thread_entry, RT_NULL, 2048, 22, 10);
    if (client_tid)
        rt_thread_startup(client_tid);

    /* 7. Finalize and start CAN bus communication. */
    is_running = RT_TRUE;
    rt_device_control(can1_dev, RT_CAN_CMD_START, &is_running);
    rt_device_control(can2_dev, RT_CAN_CMD_START, &is_running);
    LOG_I("ISOTP example started successfully.");
}

/**
 * @brief Stops the ISO-TP example and cleans up all resources.
 * @note  This function is the counterpart to `isotp_example_start`. It deletes all
 *        threads and IPC objects, and most importantly, restores the CAN devices
 *        to their original state by restoring their RX callbacks.
 */
static void isotp_example_stop(void)
{
    if (!is_running)
    {
        rt_kprintf("ISOTP example is not running.\n");
        return;
    }

    /* 1. Delete all created threads. */
    if (client_tid)
        rt_thread_delete(client_tid);
    if (logger_tid)
        rt_thread_delete(logger_tid);
    if (server_tid)
        rt_thread_delete(server_tid);
    if (rx_consumer_tid)
        rt_thread_delete(rx_consumer_tid);
    client_tid = server_tid = logger_tid = rx_consumer_tid = RT_NULL;

    /* 2. Restore original device context and close devices. */
    LOG_I("Restoring original rx_indicate and closing devices...");
    if (can1_dev)
    {
        rt_device_set_rx_indicate(can1_dev, old_can1_rx_indicate);
        rt_device_close(can1_dev);
        old_can1_rx_indicate = RT_NULL;
    }
    if (can2_dev)
    {
        rt_device_set_rx_indicate(can2_dev, old_can2_rx_indicate);
        rt_device_close(can2_dev);
        old_can2_rx_indicate = RT_NULL;
    }

    /* 3. Delete the message queue. */
    if (can_rx_mq)
        rt_mq_delete(can_rx_mq);
    can_rx_mq = RT_NULL;

    is_running = RT_FALSE;
    LOG_I("ISOTP example stopped and resources cleaned up.");
}

/**
 * @brief MSH command entry point for the example.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return RT_EOK.
 */
static int isotp_example(int argc, char **argv)
{
    if (argc > 1)
    {
        if (!strcmp(argv[1], "start"))
        {
            isotp_example_start();
        }
        else if (!strcmp(argv[1], "stop"))
        {
            isotp_example_stop();
        }
        else
        {
            rt_kprintf("Usage: isotp_example [start|stop]\n");
        }
    }
    else
    {
        rt_kprintf("Usage: isotp_example [start|stop]\n");
    }
    return RT_EOK;
}
MSH_CMD_EXPORT(isotp_example, Run ISO - TP communication example);
