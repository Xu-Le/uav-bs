//
// Copyright (C) 2017 Xu Le <xmutongxinXuLe@163.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef __TIMER_H__
#define __TIMER_H__

#define RUNNING_TIME_HIGH_PRECISION    1

#if defined(__GNUC__) && RUNNING_TIME_HIGH_PRECISION
#include <sys/time.h>
#else
#include <time.h>
#endif

#include <string.h>
#include <string>
#include "Log.h"

#if defined(__GNUC__) && RUNNING_TIME_HIGH_PRECISION
class Timer
{
public:
	explicit Timer(const char *s) : _note(s) { gettimeofday(&_start_time, NULL); }
	~Timer()
	{
		struct timeval _end_time;
		gettimeofday(&_end_time, NULL);
		printf("%ssecTime = %lds, usecTime = %ldus.\n", _note.c_str(), _end_time.tv_sec - _start_time.tv_sec, _end_time.tv_usec - _start_time.tv_usec);
	}
	
	double elapsed()
	{
		struct timeval _cur_time;
		gettimeofday(&_cur_time, NULL);
		double secTime = _cur_time.tv_sec - _start_time.tv_sec;
		double usecTime = _cur_time.tv_usec - _start_time.tv_usec;
		return secTime + usecTime/1000000;
	}

private:
	Timer(const Timer&);
	Timer& operator=(const Timer&);

private:
	std::string _note;
	struct timeval _start_time;
};
#else
class Timer
{
public:
	explicit Timer(const char *s) : _note(s) { _start_time = clock(); }
	~Timer() { printf("%s%fs.\n", _note.c_str(), elapsed()); }
	
	inline double elapsed() { return static_cast<double>(clock() - _start_time) / CLOCKS_PER_SEC; }

private:
	Timer(const Timer&);
	Timer& operator=(const Timer&);

private:
	std::string _note;
	clock_t _start_time;
};
#endif

#endif /* __TIMER_H__ */
