
/* 

  This file is provided under a dual BSD/GPLv2 license.  When using or 
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2005-2012 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify 
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but 
  WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  General Public License for more details.

  You should have received a copy of the GNU General Public License 
  along with this program; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution 
  in the file called LICENSE.GPL.
  
  Contact Information:
  Intel Corporation
  2200 Mission College Blvd.
  Santa Clara, CA  97052
  
  BSD LICENSE 

  Copyright(c) 2005-2012 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions 
  are met:

    * Redistributions of source code must retain the above copyright 
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in 
      the documentation and/or other materials provided with the 
      distribution.
    * Neither the name of Intel Corporation nor the names of its 
      contributors may be used to endorse or promote products derived 
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//Include files
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pal.h"
#include "thermal.h"

#define HOT2TEMP(x)	(0.0013985*x*x-1.010485*x+139.8188)
#define CAT2TEMP(x)	(0.0011807*x*x-1.06435*x+175.1366)

#define TEMP2CAT(x)	(int)( (1.06435-sqrt(1.06435*1.06435-4.0*0.0011807*(175.1366-x)))/(2.0*0.0011807) )

#define MIN_TEMP	60.0
#define MAX_TEMP	105.0

static const char *Intel_CEG_File_Info =
   "\r\n\r\n"
   "Intel_CEG_File_Info: " __FILE__ " compiled on " __DATE__ "\nat " __TIME__
#ifdef __GNUC__
   " with the GNU C compiler " __VERSION__
#endif
   "\r\n" "\r\n\0\0";

//#define DEBUG
#ifdef DEBUG
#define DEBUG_MSG(a...)		printf(a)
#else
#define DEBUG_MSG(a...)
#endif

#ifndef APP_NAME
#define APP_NAME "thermal"
#endif

#ifdef VER
static const char *version_string = "#@# " APP_NAME " " VER;
#endif

#define THERMAL_DEVICE	"/dev/thermal"

void
usage(char *const name)
{
   printf("Usage: %s <Options>\n", name);
   printf("Options:\n"
      "	-h, --help\n"
      "	      	Print this help.\n"
      "	-v, --version\n"
      "		Print application version\n"
      "	-r, --read\n"
      "		Read the CPU's temperature\n"
      "	-s value[60~105 degrees C]\n"
      "		Set temperature threshold for CPU shutdown condition\n"
      "		Only works for CE3100\n"
      "	-p, --print\n"
      "		Print current thermal status\n"
      "	-e, --enable\n"
      "		Enable threshold shutdown function\n"
      "	-d, --disable\n" 
      "		Disable threshold shutdown function\n");
}

void
read_version()
{
#ifdef VER
   printf("%s ", version_string);
#endif
   printf("Copyright (C) 2006-2012 Intel Corporation\n");
}

void
read_temperature()
{
   int fd = -1, re_value = EDL_FAILURE;
   _sensor_status status;

   memset(&status, 0, sizeof(status));
   if ((fd = open(THERMAL_DEVICE, O_RDONLY)) < 0) {
      printf("Open %s error!\n", THERMAL_DEVICE);
      return;
   }
   re_value = ioctl(fd, EDL_THERMAL_IOC_TRR, &status);

   if (re_value == EDL_SUCCESS) {
      if ( SOC_NAME_CE3100 == status.sensor_type ) {
          if ((status.trip_point > MIN_TRIP_POINT)
             && (status.trip_point < MAX_TRIP_POINT)) {
             	printf("Current CPU temperature is %f degrees C\n",
                HOT2TEMP(status.trip_point));
          } else if (MAX_TRIP_POINT == status.trip_point) {
             printf("CPU termperature is below the lowest detectable point for thermal sensor.\n");
          }
      } else if (SOC_NAME_CE5300 == status.sensor_type) {
         printf("Current CPU CORE 0 temperature is %d degrees C\n", status.DTS_value[0]);
         printf("Current CPU CORE 1 temperature is %d degrees C\n", status.DTS_value[1]);
      } else {
         printf("Current CPU temperature is %d degrees C\n", status.trip_point);
	}
   } else {
      printf("read temperature error!\n");
   }

   close(fd);
}

void
set_temperature_threshold(double temperature)
{
   int fd = -1;
   double read_trip_point = 0.0;
   int set_trip_point = 0;

   set_trip_point = TEMP2CAT(temperature);

   DEBUG_MSG("Input temperature is %f\n", temperature);
   DEBUG_MSG("Input trip point is %d\n", set_trip_point);

   if (set_trip_point <= MIN_TRIP_POINT || set_trip_point >= MAX_TRIP_POINT) {
      printf("Input value error or need calibration\n");
      return;
   }

   if ((fd = open(THERMAL_DEVICE, O_RDONLY)) < 0) {
      printf("Open %s error!\n", THERMAL_DEVICE);
      return;
   }

   read_trip_point = ioctl(fd, EDL_THERMAL_IOC_TPSTC, &set_trip_point);
   if(read_trip_point < 0) {
      close(fd);
      return;
   } 
   else  
      if (read_trip_point > MIN_TRIP_POINT && read_trip_point < MAX_TRIP_POINT) {
         printf("Current CPU termperature is %f degrees C\n",
         HOT2TEMP(read_trip_point));
      } 
      else {
         printf("CPU termperature is below the lowest detectable point for thermal sensor.\n");
      }

   printf("Set shutdown termperature is %f degrees C\n", temperature);

   close(fd);
}

void
set_threshold_action(int action)
{
   int fd = -1, re_value = EDL_FAILURE;
   _sensor_status status;

   memset(&status, 0, sizeof(status));

   if (SHUTDOWN_DISABLE != action && SHUTDOWN_ENABLE != action) {
      printf("set_threshold_action parameter error!\n");
      return;
   } else {
      status.enable_shutdown = action;
   }
   if ((fd = open(THERMAL_DEVICE, O_RDONLY)) < 0) {
      printf("Open %s error!\n", THERMAL_DEVICE);
      return;
   }

   re_value = ioctl(fd, EDL_THERMAL_IOC_TSC, &status);

   if (re_value == EDL_SUCCESS) {
      if (SHUTDOWN_ENABLE == status.enable_shutdown) {
//         printf("Enable shutdown function success\n");
//         printf("Default shutdown temperature is %f degrees C\n",
//            CAT2TEMP(DEFAULF_TRIP_POINT));
      } 
//      else
//         printf("Disable shutdown function success\n");
   } else {
      printf("Disable/Enable shutdown function fail!\n");
   }
   close(fd);
}

void
print_current_thermal_status()
{
   _sensor_status status;

   memset(&status, 0, sizeof(status));
   int fd = -1, re_value = EDL_FAILURE;

   if ((fd = open(THERMAL_DEVICE, O_RDONLY)) < 0) {
      printf("Open %s error!\n", THERMAL_DEVICE);
      return;
   }

   re_value = ioctl(fd, EDL_THERMAL_IOC_TSIS, &status);

   if (re_value == EDL_SUCCESS) {
      if (status.enable_shutdown == SHUTDOWN_ENABLE) {
         printf("Shutdown function: enable\n");
         if (status.trip_point > 0) {
            printf("Shutdown temperature: %d degree C\n",
               (int)CAT2TEMP(status.trip_point));
         }
      } else {
         printf("Shutdown function: disable\n");
      }
   }
   close(fd);
}

void
decode_options(int argc, char *const argv[])
{
   static const char *opt_string = "hvrs:edp";
   int optc, longind = 0;
   char set_value[5];
   double value = 0.0;

   static struct option const longopts[] = {
      {"help", no_argument, NULL, 'h'},
      {"version", no_argument, NULL, 'v'},
      {"read", no_argument, NULL, 'r'},
      {"set", required_argument, NULL, 's'},
      {"enable", no_argument, NULL, 'e'},
      {"disable", no_argument, NULL, 'd'},
      {"print", no_argument, NULL, 'p'},
      {NULL, 0, NULL, 0}
   };
   while ((optc =
         getopt_long(argc, argv, opt_string, longopts, &longind)) != -1) {
      switch (optc) {
      case 'h':
         usage(argv[0]);
         break;
      case 'v':
         read_version();
         break;
      case 'r':
         read_temperature();
         break;
      case 's':
         memset(set_value, 0, sizeof(set_value));
         strncpy(set_value, optarg, sizeof(set_value) - 1);
         value = atof(set_value);
         DEBUG_MSG("set temperature =%f\n", value);

         if (value < MIN_TEMP || value > MAX_TEMP) {
            usage(argv[0]);
            return;
         } else {
            set_temperature_threshold(value);
         }
         break;
      case 'd':
         set_threshold_action(SHUTDOWN_DISABLE);
         break;
      case 'e':
         set_threshold_action(SHUTDOWN_ENABLE);
         break;
      case 'p':
         print_current_thermal_status();
         break;
      default:
         usage(argv[0]);
         break;
      }
   }
}

int
main(int argc, char *const argv[])
{
   if (argc < 2 || (2 != strlen(argv[1]) && '-' != argv[1][1])) {
      usage(argv[0]);
      exit(0);
   }
   decode_options(argc, argv);
   return (0);                  /* stop the compiler from warning */
}
