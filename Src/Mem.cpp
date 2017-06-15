/*
 * mem.cpp
 *
 * Created: 03/11/2014 14:14:17
 *  Author: David
 */

#include "ecv.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "Mem.h"

//#include <new>
//void* operator new(size_t objsize, std::nothrow_t dummy) {
//	return malloc(objsize);
//}


void* operator new(size_t objsize)
{
	return pvPortMalloc(objsize);
}

void operator delete(void* obj) { vPortFree(obj); }

unsigned int getFreeMemory()
{
	return xPortGetFreeHeapSize();
}

// End
