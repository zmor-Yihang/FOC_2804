#ifndef __MATH_UTILS_H__
#define __MATH_UTILS_H__

/**
 * @brief 浮点数限幅
 * @param value 输入值
 * @param min_val 下限
 * @param max_val 上限
 * @return 限幅后的值
 */
static inline float utils_clampf(float value, float min_val, float max_val) {
    return (value > max_val) ? max_val : ((value < min_val) ? min_val : value);
}

#endif /* __MATH_UTILS_H__ */
