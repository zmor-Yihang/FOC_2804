#ifndef __COGGING_CALIBRATION_MODE_H__
#define __COGGING_CALIBRATION_MODE_H__

#include "../app/user_config.h"

#if (COGGING_CALIB_ENABLE != 0U)

#include "../foc/foc.h"
#include "../sensor/encoder.h"
#include "../sensor/current_sense.h"
#include "../utils/print.h"
#include "../utils/angle_utils.h"

/**
 * @brief 初始化齿槽转矩标定模式
 * @note 该模式会完成零点对齐，并进入单圈补偿表标定流程
 */
void coggingCalibrationMode_init(void);

/**
 * @brief 输出齿槽标定调试信息到上位机
 * @note 当前仅发送补偿值相关字段，便于抓取补偿表
 */
void coggingCalibrationModeDebug_print_info(void);

#endif /* COGGING_CALIB_ENABLE */

#endif /* __COGGING_CALIBRATION_MODE_H__ */