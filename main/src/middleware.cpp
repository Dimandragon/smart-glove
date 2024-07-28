#include "middleware.hpp"

size_t MiddleWare::len = 0;
SmallMutex MiddleWare::mutex = SmallMutex();
SmallMutex MiddleWare::mutex2 = SmallMutex();
TaskHandle_t MiddleWare::task_handle = NULL;