#include "usart.h"

/*----------------------重定向串口打印（DMA + FIFO）--------------------------*/
_fff_declare(uint8_t, fifo_uart_tx, FIFO_UART_TX_SIZE); // 声明FIFO发送缓冲区
_fff_init(fifo_uart_tx);                                // 初始化FIFO

/* DMA发送临时缓冲区 */
static uint8_t           tx_dma_buf[TX_DMA_BUF_SIZE];
static volatile uint8_t  tx_dma_busy       = 0;
static volatile uint32_t tx_overflow_count = 0; // 溢出计数器

/* 启动DMA发送 */
static void usart2_start_dma_tx(void) {
    if (tx_dma_busy || _fff_is_empty(fifo_uart_tx)) {
        return; /* DMA忙或FIFO空，直接返回 */
    }

    /* 从FIFO读取数据到DMA缓冲区 */
    uint16_t len = 0;
    while (len < sizeof(tx_dma_buf) && !_fff_is_empty(fifo_uart_tx)) {
        tx_dma_buf[len++] = _fff_read(fifo_uart_tx);
    }

    if (len > 0) {
        tx_dma_busy = 1;
        HAL_UART_Transmit_DMA(&huart2, tx_dma_buf, len);
    }
}

/* DMA发送完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        tx_dma_busy = 0;

        /* 如果FIFO还有数据，继续发送 */
        usart2_start_dma_tx();
    }
}

int __io_putchar(int ch) {
    /* 检查FIFO是否满，触发DMA发送 */
    if (_fff_mem_level(fifo_uart_tx) >= (FIFO_UART_TX_SIZE)) {
        if (!tx_dma_busy) {
            usart2_start_dma_tx();
        }
    }

    /* FIFO满时阻塞等待，保证数据不截断 */
    while (_fff_is_full(fifo_uart_tx)) {
        if (!tx_dma_busy) {
            usart2_start_dma_tx(); /* 尝试腾出空间 */
        }
    }

    /* 写入FIFO */
    _fff_write(fifo_uart_tx, (uint8_t)ch);

    /* DMA空闲且有数据，立即启动发送 */
    if (!tx_dma_busy && !_fff_is_empty(fifo_uart_tx)) {
        usart2_start_dma_tx();
    }

    return ch;
}

int _write(int file, char *ptr, int len) {
    int i;
    for (i = 0; i < len; i++) {
        __io_putchar(*ptr++);
    }
    return len;
}
/*------------------------------------------------------------*/

/* 定义句柄 */
UART_HandleTypeDef huart2;         /* 声明UART2句柄，用于HAL库操作USART2 */
DMA_HandleTypeDef  hdma_usart2_rx; /* 声明USART2接收DMA句柄 */
DMA_HandleTypeDef  hdma_usart2_tx; /* 声明USART2发送DMA句柄 */

/* 接收缓冲区 */
uint8_t           rx_buf_temp[RX_BUF_TEMP_SIZE]; /* 临时接收缓冲区 */
volatile uint16_t rx_size = 0;                   /* 一次接收到的数据长度 */

static uint16_t last_dma_pos = 0; /* 上次DMA位置，用于计算增量 */

_fff_declare(uint8_t, fifo_uart_rx, FIFO_UART_RX_SIZE); // 声明FIFO接收缓冲区
_fff_init(fifo_uart_rx);                                // 初始化FIFO

void usart_init(void) {
    GPIO_InitTypeDef gpio_init_struct = {0}; /* GPIO初始化结构体 */

    /* 使能USART2、GPIOA、DMA1时钟 */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* 配置USART2 TX引脚PA2为复用推挽输出模式 */
    gpio_init_struct.Pin       = GPIO_PIN_2;
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull      = GPIO_NOPULL;
    gpio_init_struct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init_struct.Alternate = GPIO_AF7_USART2; /* 配置复用功能为USART2 */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    /* 配置USART2 RX引脚PA3为复用推挽输出模式 */
    gpio_init_struct.Pin       = GPIO_PIN_3;
    gpio_init_struct.Mode      = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull      = GPIO_NOPULL;
    gpio_init_struct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    /* 初始化UART参数 */
    huart2.Instance                    = USART2;                      /* 指定USART2外设 */
    huart2.Init.BaudRate               = 1152000;                     /* 波特率1152000 */
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;          /* 8位数据位 */
    huart2.Init.StopBits               = UART_STOPBITS_1;             /* 1位停止位 */
    huart2.Init.Parity                 = UART_PARITY_NONE;            /* 无校验 */
    huart2.Init.Mode                   = UART_MODE_TX_RX;             /* 发送+接收模式 */
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;         /* 无硬件流控 */
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;        /* 16倍过采样 */
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE; /* 1位采样 */
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;         /* 时钟分频器为1 */
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;     /* 无高级功能初始化 */
    HAL_UART_Init(&huart2);                                           /* 初始化UART2 */

    /* 配置DMA参数用于USART2 TX */
    hdma_usart2_tx.Instance                 = DMA1_Channel1;         /* 指定DMA1通道1 */
    hdma_usart2_tx.Init.Request             = DMA_REQUEST_USART2_TX; /* 关联USART2发送请求 */
    hdma_usart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;  /* 内存到外设 */
    hdma_usart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;      /* 外设地址不自增 */
    hdma_usart2_tx.Init.MemInc              = DMA_MINC_ENABLE;       /* 内存地址自增 */
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;   /* 外设字节对齐 */
    hdma_usart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;   /* 内存字节对齐 */
    hdma_usart2_tx.Init.Mode                = DMA_NORMAL;            /* 正常模式 */
    hdma_usart2_tx.Init.Priority            = DMA_PRIORITY_HIGH;     /* 高优先级 */
    HAL_DMA_Init(&hdma_usart2_tx);                                   /* 初始化DMA */

    /* 配置DMA参数用于USART2 RX */
    hdma_usart2_rx.Instance                 = DMA1_Channel2;         /* 指定DMA1通道2 */
    hdma_usart2_rx.Init.Request             = DMA_REQUEST_USART2_RX; /* 关联USART2接收请求 */
    hdma_usart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;  /* 外设到内存 */
    hdma_usart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;      /* 外设地址不自增 */
    hdma_usart2_rx.Init.MemInc              = DMA_MINC_ENABLE;       /* 内存地址自增 */
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;   /* 外设字节对齐 */
    hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;   /* 内存字节对齐 */
    hdma_usart2_rx.Init.Mode                = DMA_CIRCULAR;          /* 循环模式 */
    hdma_usart2_rx.Init.Priority            = DMA_PRIORITY_HIGH;     /* 高优先级 */
    HAL_DMA_Init(&hdma_usart2_rx);                                   /* 初始化DMA */

    /* 将DMA句柄与UART句柄进行关联 */
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    /* 配置DMA和USART2相关中断的优先级和使能 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0); /* 设置DMA1通道1(TX)中断优先级 */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);         /* 使能DMA1通道1(TX)中断 */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0); /* 设置DMA1通道2(RX)中断优先级 */
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);         /* 使能DMA1通道2(RX)中断 */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);        /* 设置USART2中断优先级 */
    HAL_NVIC_EnableIRQ(USART2_IRQn);                /* 使能USART2中断 */

    /* 启动DMA方式接收，准备接收128字节数据到rx_buf_temp */
    HAL_UART_Receive_DMA(&huart2, rx_buf_temp, RX_BUF_TEMP_SIZE);

    /* 使能UART空闲中断（IDLE），用于变长包分包 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

/* 发送数据函数 */
void usart_send_data(uint8_t *data, uint16_t size) {
    /* 等待上一次DMA发送完成，避免冲突 */
    while (huart2.gState != HAL_UART_STATE_READY)
        ;
    HAL_UART_Transmit_DMA(&huart2, data, size); /* DMA 方式发送数据 */
}

/* 从FIFO读取指定数量的数据 */
uint16_t usart_read_data(uint8_t *buf, uint16_t max_size) {
    uint16_t i;
    for (i = 0; i < max_size && !_fff_is_empty(fifo_uart_rx); i++) {
        buf[i] = _fff_read(fifo_uart_rx);
    }
    return i; /* 返回实际读取的字节数 */
}

/* 获取 fifo_uart_rx 中剩余空间 */
uint16_t usart_get_availableBuffer(void) {
    return _fff_mem_free(fifo_uart_rx);
}

/* 检查接收FIFO是否为空, 0表示非空，非0表示空 */
uint8_t usart_is_rxFifoEmpty(void) {
    return _fff_is_empty(fifo_uart_rx);
}

/* UART2中断服务函数 */
void USART2_IRQHandler(void) {
    /* 判断是否为IDLE中断（空闲线） */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2); /* 清除IDLE中断标志 */

        /* 计算当前DMA位置 */
        uint16_t curr_dma_pos = RX_BUF_TEMP_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

        if (curr_dma_pos != last_dma_pos) {
            if (curr_dma_pos > last_dma_pos) {
                /* 正常情况：新数据在 last_dma_pos 到 curr_dma_pos 之间 */
                rx_size = curr_dma_pos - last_dma_pos;
                _fff_write_multiple(fifo_uart_rx, &rx_buf_temp[last_dma_pos], rx_size);
            } else {
                /* DMA环绕：先写 last_dma_pos 到末尾，再写开头到 curr_dma_pos */
                rx_size = RX_BUF_TEMP_SIZE - last_dma_pos;
                _fff_write_multiple(fifo_uart_rx, &rx_buf_temp[last_dma_pos], rx_size);
                if (curr_dma_pos > 0) {
                    _fff_write_multiple(fifo_uart_rx, rx_buf_temp, curr_dma_pos);
                }
                rx_size += curr_dma_pos;
            }
            last_dma_pos = curr_dma_pos; /* 更新位置 */
        }
    }
    HAL_UART_IRQHandler(&huart2); /* 处理HAL库内部其他中断事件 */
}

/* DMA1通道1中断服务函数（USART2 TX DMA） */
void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_tx); /* 调用HAL库DMA TX中断处理函数 */
}

/* DMA1通道2中断服务函数（USART2 RX DMA） */
void DMA1_Channel2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart2_rx); /* 调用HAL库DMA RX中断处理函数 */
}