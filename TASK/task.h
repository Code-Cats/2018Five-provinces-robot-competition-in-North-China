#ifndef __TASK_H
#define __TASK_H

#include "sys.h"

void Steer1_set(u16 set1);
void Steer2_set(u16 set2);
void Steer3_set(u16 set3);
void Steer4_set(u16 set4);

void Control_Task(void);
void Remove_Task(void);

void RC_Calibration(void);
void Date_Init(void);


#endif
