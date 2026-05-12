#include "main.h"

int main(void)
{
    // 外设初始化
    HAL_Init();
    clock_init();
    usart_init();

    gpio_init();
    encoder_init();
    tim_init();
    adc_init();
    gpio_m1_enable();

    // currentClosed_init(0.0f, 0.5f);
    speedClosed_init(2); // 速度闭环
    // positionClosed_init(0.0f); // 位置闭环
    // speedWeakClosed_init(1000);// 弱磁速度闭环
    // fluxObseverClosed_init(1000);// 无感速度闭环
    // as5600_test_init();  //
    // coggingCalibrationMode_init(); // 齿槽转矩标定

    while (1)
    {
        // currentClosedDebug_print_info();
        speedClosedDebug_print_info();
        // positionClosedDebug_print_info();
        // speedWeakClosedDebug_print_info();
        // fluxObseverClosedDebug_print_info();
        // as5600_test_poll();
        // coggingCalibrationModeDebug_print_info();
    }
}
