#if defined(__x86_64__)
#include <emmintrin.h>
#endif
#if defined(__aarch64__)
#include "sse2neon/sse2neon.h"
#endif
#if defined(__x86_64__) | defined(__aarch64__)
#define USE_SSE2
#include "sse-math/sse-math.h"
#else
#include <cmath>
#endif
#include "simd.h"

void simd::sin_cos_4f(const float x[4], float s[4], float c[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, s_v4sf, c_v4sf;
  sincos_ps(x_v4sf, &s_v4sf, &c_v4sf);
  auto s_ptr = (float *) &s_v4sf, c_ptr = (float *) &c_v4sf;
  s[0] = *s_ptr++;
  c[0] = *c_ptr++;
  s[1] = *s_ptr++;
  c[1] = *c_ptr++;
  s[2] = *s_ptr++;
  c[2] = *c_ptr++;
  s[3] = *s_ptr;
  c[3] = *c_ptr;
#else
  for (auto i = 0; i < 4; i++) {
    s[i] = sinf(x[i]);
    c[i] = cosf(x[i]);
  }
#endif
}

void simd::sin_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, s_v4sf = sin_ps(x_v4sf);
  auto s_ptr = (float *) &s_v4sf;
  x[0] = *s_ptr++;
  x[1] = *s_ptr++;
  x[2] = *s_ptr++;
  x[3] = *s_ptr;
#else
  for (auto i = 0; i < 4; i++)
    x[i] = sinf(x[i]);
#endif
}

float simd::sin_f(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x}, s_v4sf = sin_ps(x_v4sf);
  return *(float *) &s_v4sf;
#else
  return sinf(x);
#endif
}

void simd::cos_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, c_v4sf = cos_ps(x_v4sf);
  auto c_ptr = (float *) &c_v4sf;
  x[0] = *c_ptr++;
  x[1] = *c_ptr++;
  x[2] = *c_ptr++;
  x[3] = *c_ptr;
#else
  for (auto i = 0; i < 4; i++)
    x[i] = cosf(x[i]);
#endif
}

float simd::cos_f(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x}, c_v4sf = cos_ps(x_v4sf);
  return *(float *) &c_v4sf;
#else
  return cosf(x);
#endif
}

void simd::tan_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = tan_ps(x_v4sf);
  auto t_ptr = (float *) &t_v4sf;
  x[0] = *t_ptr++;
  x[1] = *t_ptr++;
  x[2] = *t_ptr++;
  x[3] = *t_ptr;
#else
  for (auto i = 0; i < 4; i++)
    x[i] = tanf(x[i]);
#endif
}

void simd::cot_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = cot_ps(x_v4sf);
  auto t_ptr = (float *) &t_v4sf;
  x[0] = *t_ptr++;
  x[1] = *t_ptr++;
  x[2] = *t_ptr++;
  x[3] = *t_ptr;
#else
  for (auto i = 0; i < 4; i++)
    x[i] = 1.f / tanf(x[i]);
#endif
}

void simd::atan_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = atan_ps(x_v4sf);
  auto t_ptr = (float *) &t_v4sf;
  x[0] = *t_ptr++;
  x[1] = *t_ptr++;
  x[2] = *t_ptr++;
  x[3] = *t_ptr;
#else
  for (auto i = 0; i < 4; i++)
    x[i] = atanf(x[i]);
#endif
}

void simd::atan2_4f(const float y[4], const float x[4], float res[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, y_v4sf = {y[0], y[1], y[2], y[3]};
  v4sf res_v4sf = atan2_ps(y_v4sf, x_v4sf);
  auto res_ptr = (float *) &res_v4sf;
  res[0] = *res_ptr++;
  res[1] = *res_ptr++;
  res[2] = *res_ptr++;
  res[3] = *res_ptr;
#else
  for (auto i = 0; i < 4; i++)
    res[i] = atan2f(y[i], x[i]);
#endif
}

float simd::atan2_f(float y, float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  return atan2_ref(y, x);
#else
  return atan2f(y, x);
#endif
}

float simd::sqrt_f(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  return sqrt_ps(x);
#else
  return sqrtf(x);
#endif
}

float simd::rsqrt_f(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  return rsqrt_ps(x);
#else
  return 1.f / sqrtf(x);
#endif
}
