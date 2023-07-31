/*
 * senser_FFT.c
 *
 *  Created on: Mar 6, 2023
 *      Author: Arnold_Hsieh
 */
/* FFT settings */
#define SAMPLES					512 			 			/* 256 real party and 512  imaginary parts */
#define FFT_SIZE				(SAMPLES / 2)		/* FFT size is always the same size as we have samples, so 256 in our case */
#define ARM_MATH_CM4

#include "senser_FFT.h"
#include "main.h"
#include <stdio.h>
#include "arm_math.h"

arm_cfft_radix4_instance_f32 S; /* ARM CFFT module */

float32_t maxValue; /* Max FFT value is stored here */
uint32_t maxIndex; /* Index in Output array where max value is */
float32_t Input[SAMPLES];
float32_t Output[FFT_SIZE];

void FFT_(uint16_t *low_data, float32_t *max_value, uint32_t *max_index) {
	//uint8_t X;
	printf("FFT_start\r\n");
	//char buffer[32];
	for (int i = 0; i < FFT_SIZE; i++) {
		Input[i * 2] = (float32_t) ((float32_t) low_data[i] - 2048.0) * 1.65
				/ 2048.0; //
		Input[(uint16_t) (i * 2 + 1)] = 0;
	}

// Use CFFT module to process the data.

	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&S, Input);
	/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(Input, Output, FFT_SIZE);
	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(Output, FFT_SIZE, &maxValue, &maxIndex);

	*max_value = maxValue/ 128;
	*max_index = maxIndex* 4;
	//printf("%f, %d\r\n", maxValue / 128, (int) (maxIndex) * 4);
	for (int i = 0; i < FFT_SIZE / 2; i++) {
		Output[i] = Output[i] / 128;	//i=0 /256 is batter
		//	printf("%f\r\n", Output[i]);
//		HAL_Delay(10);
	}
}
