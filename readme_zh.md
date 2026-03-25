[English](./README.md)

# RT-Thread 的 rtt_isotp-c 软件包

`rtt_isotp-c` 是对上游 [isotp-c](https://github.com/SimonCahill/isotp-c) 的 RT-Thread 适配层软件包。

它为 ISO 15765-2（ISO-TP / CAN-TP）提供线程安全、事件驱动、支持多链路的 RT-Thread 集成方式，适用于 UDS 诊断、ECU 刷写以及任何需要在 CAN 总线上传输大于单帧的数据负载的场景。

在线站点文档 https://wdfk-prog.space/rtt_isotp-c/

## 说明

- 如果你只需要可直接使用的 UDS 协议栈，优先参考 [iso14229](https://github.com/wdfk-prog/iso14229)。
- 本仓库聚焦于 `isotp-c` 的 RT-Thread 适配层，更适合需要扩展传输层行为或将传输层集成进自定义应用的场景。
- `isotp-c/` 目录是上游子模块镜像，在本仓库中应视为 **只读**。
- 相关项目：
  - [iso14229 client demo](https://github.com/wdfk-prog/iso14229/tree/rtt/examples/rtt_server/client_demo)
  - [can_uds](https://github.com/wdfk-prog/can_uds)

## 特性

- 线程安全的阻塞式发送与接收接口。
- 可选的非阻塞发送接口。
- 基于 RT-Thread IPC 的事件驱动同步机制。
- 支持单总线或多总线下的多链路并发。
- 推荐使用生产者-消费者模型完成 ISR 安全的 CAN 接收集成。
- 自带示例，演示 client/server/logger 的完整拓扑。

## 仓库结构

```text
rtt_isotp-c/
├── isotp-c/                  # 上游 isotp-c 子模块镜像（本仓库中只读）
├── examples/
│   └── isotp_examples.c      # RT-Thread MSH 示例
├── figures/
│   └── config_menu.png       # 软件包启用截图
├── isotp_rtt.c               # RT-Thread 适配层实现
├── isotp_rtt.h               # 公开 API 头文件
├── LICENSE                   # 仓库许可证（LGPL-2.1）
├── readme.md                 # 英文 README（默认）
├── readme_zh.md              # 中文 README
└── SConscript                # RT-Thread 软件包构建脚本
```

## 许可证

本仓库根目录 `LICENSE` 为 **LGPL-2.1**。上游 `isotp-c` 子模块使用其自身许可证，见 `isotp-c/LICENSE`。

## 依赖

- RT-Thread >= 4.0.0
- RT-Thread CAN 设备框架（`RT_USING_CAN`）
- 若需运行示例命令，还需启用 FinSH/MSH（`RT_USING_FINSH`）

## 开启软件包

在 RT-Thread 软件包菜单中开启：

`RT-Thread online packages -> peripherals -> ISO-TP`

![config_menu.png](figures/config_menu.png)

开启后保存配置，并在工程根目录执行：

```sh
pkgs --update
```

## 集成模型

适配层的核心约束很简单：

> CAN 报文接收属于你的应用；ISO-TP 组包与状态机处理属于适配层。

推荐模型如下：

1. 在 CAN RX 回调中接收原始 CAN 报文。
2. 在 ISR 上下文中把报文投递到消息队列。
3. 在专用线程中消费这些报文。
4. 只在线程上下文中调用 `isotp_rtt_on_can_msg_received()`。

`isotp_rtt_on_can_msg_received()` **不能**在 ISR 中直接调用，因为协议层可能需要立即发送 Flow Control 报文。

## 快速开始

```c
#include "isotp_rtt.h"

static rt_device_t can_dev;
static rt_mq_t can_rx_mq;
static uint8_t send_buf[256];
static uint8_t recv_buf[256];

static rt_err_t my_can_rx_callback(rt_device_t dev, rt_size_t size)
{
    struct rt_can_msg msg;

    msg.hdr_index = -1;
    if (rt_device_read(dev, 0, &msg, sizeof(msg)) == sizeof(msg))
    {
        rt_mq_send(can_rx_mq, &msg, sizeof(struct rt_can_msg));
    }
    return RT_EOK;
}

static void can_consumer_thread_entry(void *parameter)
{
    struct rt_can_msg msg;

    while (rt_mq_recv(can_rx_mq, &msg, sizeof(msg), RT_WAITING_FOREVER) == sizeof(msg))
    {
        isotp_rtt_on_can_msg_received(&msg);
    }
}

void app_init(void)
{
    can_dev = rt_device_find("can1");
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);

    can_rx_mq = rt_mq_create("can_rx", sizeof(struct rt_can_msg), 32, RT_IPC_FLAG_FIFO);
    rt_device_set_rx_indicate(can_dev, my_can_rx_callback);

    isotp_rtt_link_t link = isotp_rtt_create(can_dev,
                                             0x7E0,
                                             0x7E8,
                                             RT_CAN_STDID,
                                             RT_CAN_DTR,
                                             send_buf,
                                             sizeof(send_buf),
                                             recv_buf,
                                             sizeof(recv_buf));

    uint8_t request[] = {0x22, 0xF1, 0x90};
    uint8_t response[256];
    uint16_t response_size = 0;

    isotp_rtt_send(link, request, sizeof(request), RT_WAITING_FOREVER);
    isotp_rtt_receive(link, response, sizeof(response), &response_size, RT_WAITING_FOREVER);
}
```

## 公开 API 概览

### `isotp_rtt_create()`
创建一个 ISO-TP 链路实例，并绑定到一个已经打开的 RT-Thread CAN 设备。

### `isotp_rtt_destroy()`
销毁链路并释放适配层分配的 RT-Thread 资源。

### `isotp_rtt_on_can_msg_received()`
向 ISO-TP 状态机喂入一帧原始 CAN 报文。只能在线程上下文调用。

### `isotp_rtt_send()`
阻塞式 ISO-TP 发送接口。

### `isotp_rtt_send_nonblocking()`
非阻塞发送接口，只负责排队发送请求并立即返回。

### `isotp_rtt_receive()`
阻塞式 ISO-TP 接收接口，返回一帧完整重组后的上层负载。

## 示例

`examples/isotp_examples.c` 演示了：

- 两个 CAN 设备（`can1`、`can2`）
- 一个客户端（tester）
- 一个服务端（ECU）
- 一个被动日志器（logger）
- 硬件过滤器配置、基于消息队列的接收处理，以及 MSH 下的 start/stop 生命周期管理

使能 `PKG_USING_ISOTP_C` 与 `PKG_ISOTP_C_EXAMPLES`，编译并下载固件后，运行：

```sh
msh />isotp_example start
```

## 配置说明

适配层会自动创建后台轮询线程 `isotp_poll`。你可以在 menuconfig 中配置：

- 轮询线程栈大小
- 轮询线程优先级
- 轮询间隔
- 示例开关

## Doxygen / GitHub Pages

本仓库可以通过 Doxygen 生成 API 文档，并用 GitHub Pages 发布。推荐配套文件为：

- `Doxyfile`
- `.github/workflows/pages-doxygen.yml`
- `docs/doxygen-mainpage.md`

这些文件已包含在本次生成的更新包中。

## 参考资料

1. <https://en.wikipedia.org/wiki/ISO_15765-2>
2. <https://docs.kernel.org/networking/iso15765-2.html>
3. <https://github.com/SimonCahill/isotp-c>
