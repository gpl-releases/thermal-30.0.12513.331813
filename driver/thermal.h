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


#ifndef _EDL_THERMAL_DRV_H_
#define _EDL_THERMAL_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#define SOC_MAX_CORE_COUNT	4

typedef struct{
	int enable_shutdown;	/*Shutdown function is enable/disable.*/
	unsigned int trip_point; /*The threshold trip point set for shutdown function*/
	unsigned int sensor_type; /*The sensor type of processor*/
	unsigned int DTS_value[SOC_MAX_CORE_COUNT]; /*Digital Thermal Sensor value of each core*/
}_sensor_status;



/** \def EDL_THERMAL_DEVICE_NAME
    \brief The driver's name
*/
#define EDL_THERMAL_DEVICE_NAME	"thermal"

/*
0x0 <= B2[6:0] <= 0x7F
*/
#define MIN_TRIP_POINT	(0x0)
#define MAX_TRIP_POINT	(0x7F)
#define DEFAULF_TRIP_POINT	(0x47)

#define SHUTDOWN_ENABLE		(0x1)
#define SHUTDOWN_DISABLE	(0x2)
/* IOCTLs */

/*
*/
#include <asm/ioctl.h>

typedef enum {
   EDL_SUCCESS                = 0x2000, /**< 0x2000 */ 
   EDL_FIRST_ERROR            = EDL_SUCCESS,
   EDL_INVALID_PARAM          = 0x2001, /**< 0x2001 */
   EDL_NOT_INITIALIZED        = 0x2002, /**< 0x2002 */
   EDL_ALREADY_INITIALIZED    = 0x2003, /**< 0x2003 */
   EDL_FAILURE                = 0x2004, /**< 0x2004 */
   EDL_MEM_ALLOC_FAIL         = 0x2005, /**< 0x2005 */
   EDL_MUTEX_NOT_INITIALIZED  = 0x2006, /**< 0x2006 */
   EDL_GETMUTEX_SEMOP_ERR     = 0x2007, /**< 0x2007 */
   EDL_MUTEX_CREATION_FAIL    = 0x2008, /**< 0x2008 */
   EDL_NO_MEDIA               = 0x2009, /**< 0x2009 */
   EDL_BUS_BUSY               = 0x200A, /**< 0x200A */
   EDL_NULL_POINTER           = 0x200B, /**< 0x200B */
   EDL_EEPROM_WRITE_FAIL      = 0x200C, /**< 0x200C */
   EDL_I2C_BUS_OPEN_FAIL      = 0x200D, /**< 0x200D */
   EDL_I2C_BUS_NOT_OPEN       = 0x200E, /**< 0x200E */
   EDL_I2C_BUS_CLOSE_FAIL     = 0x200F, /**< 0x200F */
   EDL_I2C_BUS_SPEED_SET_FAIL = 0x2010, /**< 0x2010 */
   EDL_I2C_READ_WRITE_FAIL    = 0x2011, /**< 0x2011 */
   EDL_I2C_INVALID_BUS_OR_DEVADDR  = 0x2012, /**< 0x2012 */
   EDL_UNKNOWN_ERROR            = 0x201B, /**< 0x201B */
   EDL_LAST_ERROR               = EDL_UNKNOWN_ERROR
} edl_result_t;


/** \def EDL_MEMORY_LAYOUT_IOC_MAGIC
    \brief The driver's Magic Number for IOCTLs

    No other device using a \b % for their Magic Number.
*/
#define EDL_THERMAL_IOC_MAGIC '*'

/** \def EDL_THERMAL_IOC_GET_INT
    \brief IOCTL number to Get The Integer Value
*/
#define EDL_THERMAL_IOC_TMC		_IOR(EDL_THERMAL_IOC_MAGIC, 1, char *)

/** \def EDL_THERMAL_IOC_GET_STR
    \brief IOCTL number to Get The String Value
*/
#define EDL_THERMAL_IOC_DTELT		_IOR(EDL_THERMAL_IOC_MAGIC, 2, char *)

/** \def EDL_THERMAL_IOC_SET_INT
    \brief IOCTL number to Set The Integer Value
*/
#define EDL_THERMAL_IOC_GTEL		_IOR(EDL_THERMAL_IOC_MAGIC, 3, char *)

/** \def EDL_THERMAL_IOC_SET_STR
    \brief IOCTL number to Set The String Value
*/
#define EDL_THERMAL_IOC_TSC		_IOR(EDL_THERMAL_IOC_MAGIC, 4, char *)

/** \def EDL_THERMAL_IOC_NODE_FIND
    \brief IOCTL number to Find The Specified Node
*/
#define EDL_THERMAL_IOC_TRR		_IOR(EDL_THERMAL_IOC_MAGIC, 5, char *)

/** \def EDL_THERMAL_IOC_NODE_FIRST_CHILD
    \brief IOCTL number to Find The First Child Node
*/
#define EDL_THERMAL_IOC_TPSTC	_IOR(EDL_THERMAL_IOC_MAGIC, 6, char *)

/** \def EDL_THERMAL_IOC_NODE_NEXT_SIBLING
    \brief IOCTL number to Find The Next Sibling Node
*/
#define EDL_THERMAL_IOC_TSIS 	_IOR(EDL_THERMAL_IOC_MAGIC, 7, char *)

/** \def EDL_THERMAL_IOC_NODE_GET_NAME
    \brief IOCTL number to Get The Name of Specified Node 
*/
#define EDL_THERMAL_IOC_TTB	_IOR(EDL_THERMAL_IOC_MAGIC, 8, char *)

/*@)*/

#ifdef __cplusplus
}
#endif

#endif /* _EDL_THERMAL_DRV_H_ */

