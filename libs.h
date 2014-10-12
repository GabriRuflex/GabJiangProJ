#include <cstdio>
#include <pthread.h>
#include <unistd.h>
#include "miosix.h"
#include <miosix/kernel/scheduler/scheduler.h>

using namespace std;
using namespace miosix;

#define DEBUG
#define NVAL 5000

#include "adc.h"

#include "adc.cpp"
#include "timer.cpp"
