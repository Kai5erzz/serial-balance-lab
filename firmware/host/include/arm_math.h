#ifndef LAB_ARM_MATH_H
#define LAB_ARM_MATH_H
#include <stdint.h>
#include <math.h>
#include <string.h>
typedef float float32_t;
typedef double float64_t;
typedef int32_t q31_t;
typedef int16_t q15_t;
typedef int8_t q7_t;
typedef enum { ARM_MATH_SUCCESS=0, ARM_MATH_ARGUMENT_ERROR=-1, ARM_MATH_LENGTH_ERROR=-2,
               ARM_MATH_SIZE_MISMATCH=-3, ARM_MATH_NANINF=-4, ARM_MATH_SINGULAR=-5 } arm_status;
typedef struct { uint16_t numRows, numCols; float32_t *pData; } arm_matrix_instance_f32;
#define FAST_MATH_TABLE_SIZE 512
#ifndef PI
#define PI 3.14159265358979f
#endif
float32_t arm_sin_f32(float32_t x);
float32_t arm_cos_f32(float32_t x);
void arm_mat_init_f32(arm_matrix_instance_f32 *, uint16_t, uint16_t, float32_t *);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
static inline arm_status arm_sqrt_f32(float32_t x, float32_t *y) { *y = x >= 0 ? sqrtf(x) : 0; return x >= 0 ? ARM_MATH_SUCCESS : ARM_MATH_ARGUMENT_ERROR; }
#endif
