/*
 * lab1math.h
 *
 *  Created on: 2022年9月13日
 *      Author: ouwakariki
 */
#include "main.h"
#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_

void cMul(float *arrayA, float *arrayB, float *c_Y, uint32_t size);
extern void asmMul(float *array1, float *array2,uint32_t size ,float *asm_mul);

void cStdDev(float *array, uint32_t size, float *c_std_dev);
extern void asmStdDev(float *array, uint32_t size, float *asm_std_dev);


#endif /* INC_LAB1MATH_H_ */
