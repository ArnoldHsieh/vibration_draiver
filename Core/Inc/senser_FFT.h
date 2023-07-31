/*
 * senser_FFT.h
 *
 *  Created on: Mar 6, 2023
 *      Author: Arnold_Hsieh
 */

#ifndef INC_SENSER_FFT_H_
#define INC_SENSER_FFT_H_

#include "senser_FFT.h"
#include "main.h"
#include <stdio.h>
#include "arm_math.h"

void FFT_(uint16_t *low_data, float32_t *max_value, uint32_t *max_index);

#endif /* INC_SENSER_FFT_H_ */
