/*
 * HC_SR05.h
 */

#ifndef INC_HC_SR05_H_
#define INC_HC_SR05_H_

#include "stm32f4xx_hal.h"

void Delay(uint32_t time);
float HCSR05_GetDis (void);

#endif /* INC_HC_SR05_H_ */
