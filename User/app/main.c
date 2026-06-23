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
    // speedClosed_init(2);               // 速度闭环
    // positionClosed_init(0.0f);         // 位置闭环
    // speedWeakClosed_init(1000);        // 弱磁速度闭环
    // fluxObseverClosed_init(500);       // 无感速度闭环(Ortega)
    // mxlemmingObserverClosed_init(500); // 无感速度闭环(MXLEMMING)
    improvedFluxObserverClosed_init(100); // 无感速度闭环(改进非线性磁链观测器)
    // resistanceMeasureMode_init();      // 电阻辨识
    // inductanceMeasureMode_init();      // 电感辨识
    // coggingCalibrationMode_init();     // 齿槽转矩标定

    while (1)
    {
        // currentClosedDebug_print_info();
        // speedClosedDebug_print_info();
        // positionClosedDebug_print_info();
        // speedWeakClosedDebug_print_info();
        // fluxObseverClosedDebug_print_info();
        // mxlemmingObserverClosedDebug_print_info();
        // improvedFluxObserverClosedDebug_print_info();
        // resistanceMeasureModeDebug_print_info();
        // inductanceMeasureModeDebug_print_info();
        improvedFluxObserverClosedDebug_print_info();
        // coggingCalibrationModeDebug_print_info();
    }
}
