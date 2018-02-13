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

#ifndef __LOG_H__
#define __LOG_H__

#include <stdio.h>

#define LOG_NEED_MACRO_FILE_LINE    0

#ifdef __GNUC__
#define ERROR_LEVEL      1
#define WARNING_LEVEL    2
#define INFO_LEVEL       3
#define DEBUG_LEVEL      4

#if LOG_NEED_MACRO_FILE_LINE
#define uncond_log(...)     do { printf("%s: %d | ", __FILE__, __LINE__); printf(__VA_ARGS__); } while(0)
#define error_log(...)      do { if (log_level >= ERROR_LEVEL)   { printf("ERROR | %s: %d | ", __FILE__, __LINE__); printf(__VA_ARGS__); } } while(0)
#define warning_log(...)    do { if (log_level >= WARNING_LEVEL) { printf("WARN | %s: %d | ", __FILE__, __LINE__); printf(__VA_ARGS__); } } while(0)
#define info_log(...)       do { if (log_level >= INFO_LEVEL)    { printf("INFO | %s: %d | ", __FILE__, __LINE__); printf(__VA_ARGS__); } } while(0)
#define debug_log(...)      do { if (log_level >= DEBUG_LEVEL)   { printf("DEBUG | %s: %d | ", __FILE__, __LINE__); printf(__VA_ARGS__); } } while(0)
#else
#define uncond_log(...)     do { printf(__VA_ARGS__); } while(0)
#define error_log(...)      do { if (log_level >= ERROR_LEVEL)   { printf("ERROR | "); printf(__VA_ARGS__); } } while(0)
#define warning_log(...)    do { if (log_level >= WARNING_LEVEL) { printf("WARN | "); printf(__VA_ARGS__); } } while(0)
#define info_log(...)       do { if (log_level >= INFO_LEVEL)    { printf("INFO | "); printf(__VA_ARGS__); } } while(0)
#define debug_log(...)      do { if (log_level >= DEBUG_LEVEL)   { printf("DEBUG | "); printf(__VA_ARGS__); } } while(0)
#endif
const char* getLogLevel();
#else /* __GNUC__ */
#define uncond_log     printf
#define error_log      printf
#define warning_log    printf
#define info_log       printf
#define debug_log      printf
#endif

#endif /* __LOG_H__ */
