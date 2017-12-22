#ifndef __CLOCK_HPP__
#define __CLOCK_HPP__

#include <time.h>

class Clock
{
public:
	Clock(void)
	{
		tick();
	}
	
	void tick(void)
	{
        //clock_gettime(CLOCK_REALTIME,  &start);
        start = clock();
	}
		
	double tock(void)
	{
        end = clock();
        return (end-start)/(double)CLOCKS_PER_SEC;
        //clock_gettime(CLOCK_REALTIME,  &end);
        //return ((end.tv_sec * 10e9 + end.tv_nsec) - (start.tv_sec * 10e9 + start.tv_nsec)) * 1e-9;
	}	
		
private:

    //timespec start, end;
    clock_t start, end;
};

#endif


