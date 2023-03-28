#ifndef SRM_IC_2023_MODULES_SIMD_SIMD_H_
#define SRM_IC_2023_MODULES_SIMD_SIMD_H_

namespace simd {
void sin_cos_4f(const float x[4], float s[4], float c[4]);
void sin_4f(float x[4]);
float sin_f(float x);
void cos_4f(float x[4]);
float cos_f(float x);
void tan_4f(float x[4]);
void cot_4f(float x[4]);
void atan_4f(float x[4]);
void atan2_4f(const float y[4], const float x[4], float res[4]);
float atan2_f(float y, float x);
float sqrt_f(float x);
float rsqrt_f(float x);
}

#endif  // SRM_IC_2023_MODULES_SIMD_SIMD_H_
