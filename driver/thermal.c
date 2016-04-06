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

/*Include files*/
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/msr.h>
#include <linux/proc_fs.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include "thermal.h"
#include "osal.h"
#include "pal.h"

extern struct pci_raw_ops pci_direct_conf1;
unsigned int pci_read_root_devid(void);

/*
 * Function for obtaining the device id of PCI root device.
 */
unsigned int pci_read_root_devid(void)
{
	unsigned int devid = 0;
	struct pci_bus * root_bus;

	root_bus = pci_find_bus(0,0);
	pci_bus_read_config_word(root_bus, PCI_DEVFN(0,0), PCI_DEVICE_ID, &devid);
	return devid;
}

static const char *Intel_CEG_File_Info =
   "\r\n\r\n"
   "Intel_CEG_File_Info: " __FILE__ " compiled on " __DATE__ "\nat " __TIME__
#ifdef __GNUC__
   " with the GNU C compiler " __VERSION__
#endif
   "\r\n" "\r\n\0\0";

#ifndef MOD_NAME
#define MOD_NAME "thermal.ko"
#endif

#ifdef VER
static const char *version_string = "#@# " MOD_NAME " " VER;
#endif

//#define DEBUG
#ifdef DEBUG
#define DEBUG_MSG(a...)               printk(a)
#else
#define DEBUG_MSG(a...)
#endif

#define TSC_E0		(1<<15)
#define TSC_AME 	(1<<3)
#define TSIS_HTI	(1<<20)
#define TTB_CSS		(1<<26)

#define 	Tjmax  110
#define 	BIT(a) (1 << a)
#define 	TEMP_MASK  ( BIT(22) | BIT(21) | BIT (20) | BIT (19) | BIT(18) | BIT(17) | BIT(16) )

#define DFX_MEM_BASE_ADDR           	0xDF8F0000
#define DFX_MEM_SIZE            			0x10000
#define DFX_BUS_NUM 1
#define DFX_DEVICE_NUM 11
#define DFX_FUNC_NUM 7

#define	TEMPERATURE_REF_DEFAULT	Tjmax 	/* default untrimmed */
#define	TEMPERATURE_REF_CE4100 	102 	/* PROCHOT trip point untrimmed */
#define	TEMPERATURE_REF_CE4200 	102 	/* CE4200 for now */
#define TEMPERATURE_REF_CE5300  100     /* CE5300 not determine yet */
#define	CE4100_DTS_TRIMMED_VER	10 	/* class test program revision with trimming */
#define	CE4200_DTS_TRIMMED_VER	3	/* CE4200 DTS trimming is enabled since rev3 H[227931]*/
#define CE5300_DTS_TRIMMED_VER  5     	/* CE5300 DTS trimming not determine yet */

#define CE5300_CORE_COUNT	2

/* These are predefined macros that specify different parameters
 * for our driver */
MODULE_AUTHOR("Intel Corporation, (C) 2005 - 2012 - All Rights Reserved");
MODULE_DESCRIPTION("Platform Configuration Device Driver");
MODULE_SUPPORTED_DEVICE("Intel Media Processors");

/* Notifies the kernel that our driver is not GPL. */
MODULE_LICENSE("Dual BSD/GPL");
//MODULE_LICENSE("GPL");

int major_number = -1;

static pal_soc_name_t soc = SOC_NAME_CE3100;
static pal_soc_stepping_t stepping = SOC_STEPPING_A0;
//Only for one application once
static int device_open = 0;

/*-------------------------------------------------------------------------------------------------------------------*/
/*				 device operation interfaces				     */
/*-------------------------------------------------------------------------------------------------------------------*/
static int edl_thermal_open(struct inode *inode, struct file *filp);
static int edl_thermal_release(struct inode *inode, struct file *filp);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 35)
static int edl_thermal_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int edl_thermal_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif

static int edl_thermal_init(void);
static void edl_thermal_exit(void);

static edl_result_t edl_thermal_sensor_control(_sensor_status *);
static int edl_thermal_trip_point_setting(unsigned int);
static edl_result_t edl_thermal_read_sensor_status(_sensor_status *);
static edl_result_t edl_thermal_read_temperature(_sensor_status *);

static void edl_thermal_write_register(unsigned char port,
   unsigned char address, unsigned char byte_enable, unsigned int MDR_data);
static int edl_thermal_read_register(unsigned char port,
   unsigned char address);

static edl_result_t edl_thermal_ce4100_read_temperature(_sensor_status *);
static edl_result_t edl_thermal_ce4100_auto_shutdown_enable(void);
static edl_result_t edl_thermal_ce4100_auto_shutdown_disable(void);
static edl_result_t edl_thermal_ce5300_auto_shutdown_enable(void);
static edl_result_t edl_thermal_ce5300_auto_shutdown_disable(void);
static unsigned int edl_thermal_get_base_addr(void);

static unsigned int edl_thermal_get_base_addr()
{
	unsigned int phyad =0;
#ifdef HARDCODE_BAR
	return DFX_MEM_BASE_ADDR;
#else
	os_pci_dev_t  pci_dev = NULL;
    //get the device  DFX
	if(os_pci_device_from_address(&pci_dev, DFX_BUS_NUM, DFX_DEVICE_NUM, DFX_FUNC_NUM) != OSAL_SUCCESS)
	{
			OS_INFO("Unable to access the PCI bus\n");
			return false;
	}
	//read the pci device base address from bar 0 
	os_pci_read_config_32(pci_dev, 0x10, &phyad);	
	OS_PCI_FREE_DEVICE(pci_dev);
	//printk("phyad = 0x%x\n", phyad);
	return phyad;
#endif

}

static edl_result_t
edl_thermal_sensor_control(_sensor_status * status)
{
   _sensor_status tmp;
   edl_result_t ret;

   if (NULL == status) {
      return EDL_FAILURE;
   }

   if (SOC_NAME_CE3100 == soc)
   {
      /*The hot mode should be triped first. 
         If not, setting action will shutdown immediately
         even if the actually temp is very low. */
      edl_thermal_read_temperature(&tmp);
      edl_thermal_write_register(0x04, 0xB0, 0x0, (TSC_E0 | TSC_AME));
 
      if (SHUTDOWN_ENABLE == status->enable_shutdown) {
         edl_thermal_write_register(0x04, 0xB2, 0x0, DEFAULF_TRIP_POINT);
         edl_thermal_write_register(0x04, 0xB6, 0x0, (TTB_CSS));
      } else {
         edl_thermal_write_register(0x04, 0xB2, 0x0, 0x7F7F0000);
         edl_thermal_write_register(0x04, 0xB2, 0x0, 0x0);
         edl_thermal_write_register(0x04, 0xB6, 0x0, 0x0);
      }
   }
   else {
      if (SHUTDOWN_ENABLE == status->enable_shutdown) {
	 if (SOC_NAME_CE5300 == soc) 
         	ret = edl_thermal_ce5300_auto_shutdown_enable();
	 else
         	ret = edl_thermal_ce4100_auto_shutdown_enable();

	  if( ret != EDL_SUCCESS ){
         	printk("For Intel CE4x00/CE5300 Media Processor, please make sure it is a DTS fuse enabled part, or DTS has been overrided!\n");	
         	return ret;
         }
      } else {
	 if (SOC_NAME_CE5300 == soc) 
         	ret = edl_thermal_ce5300_auto_shutdown_disable();
	 else 
         	ret = edl_thermal_ce4100_auto_shutdown_disable();

	  if( ret != EDL_SUCCESS ){
          	printk("For Intel CE4x00/CE5300 Media Processor, please make sure it is a DTS fuse enabled part, or DTS has been overrided!\n");	
            	return ret;
         }
      } 
   }
   return EDL_SUCCESS;
}

static void
edl_thermal_write_register(unsigned char port, unsigned char address,
   unsigned char byte_enable, unsigned int MDR_data)
{
   unsigned int MCR_data;

   MCR_data = 0xE0000000 + (port << 16) + (address << 8) + (byte_enable << 4);

   outl(0x800000D4, 0xCF8);
   outl(MDR_data, 0xCFC);
   outl(0x800000D0, 0xCF8);
   outl(MCR_data, 0xCFC);
   msleep(1);
}

static int
edl_thermal_read_register(unsigned char port, unsigned char address)
{
   unsigned int MDR_data, temp;

   MDR_data = 0xD0000000 + (port << 16) + (address << 8);

   outl(0x800000D0, 0xCF8);
   outl(MDR_data, 0xCFC);
   outl(0x800000D4, 0xCF8);
   temp = inl(0xCFC);

   return temp;
}

static edl_result_t
edl_thermal_read_temperature(_sensor_status * status)
{
   int data = 0;
   int read_trip_point = 0;
   int set_trip_point = 0;

   if (NULL == status) {
      return EDL_FAILURE;
   }

	status->sensor_type = soc;

   //Query shutdown function is enable or not
   data = edl_thermal_read_register(0x04, 0xB6);
   if (data & TTB_CSS) {
      status->enable_shutdown = SHUTDOWN_ENABLE;
	  set_trip_point = (0x7F & edl_thermal_read_register(0x04, 0xB2));
	  edl_thermal_write_register(0x04, 0xB6, 0x0, 0x0);
   } else {
      status->enable_shutdown = SHUTDOWN_DISABLE;
   }

   //Write B0. Opcode=0xE0, Port=0x04, Address=0xB0, ByteEnable=0x0
   //B0[15]=1,enable sensor 0;B0[3]=1, enable analog mode
   edl_thermal_write_register(0x04, 0xB0, 0x0, (TSC_E0 | TSC_AME));

   //Clear the regsiter B5 status
   edl_thermal_write_register(0x04, 0xB2, 0x0, 0x7F7F0000);
   //Enable it
   edl_thermal_write_register(0x04, 0xB2, 0x0, 0x0);

   for (read_trip_point = 0; read_trip_point < 0x7F; read_trip_point++) {

      edl_thermal_write_register(0x04, 0xB2, 0x0, read_trip_point);

      data = edl_thermal_read_register(0x04, 0xB5);

      DEBUG_MSG("data=0x%x, i=%d\n", data, read_trip_point);

      if (data & (TSIS_HTI))    // B5[20], Hot Trip Indicator
      {
         DEBUG_MSG("line:%d, data=0x%x, i=%d!!!\n", __LINE__, data,
            read_trip_point);

         status->trip_point = read_trip_point;
		 if(SHUTDOWN_ENABLE == status->enable_shutdown)
		 {
		 	edl_thermal_write_register(0x04, 0xB2, 0x0, set_trip_point);
			msleep(1);
			edl_thermal_write_register(0x04, 0xB6, 0x0, TTB_CSS);
		 }
         return EDL_SUCCESS;
      }

   }
   status->trip_point = read_trip_point;
   if(SHUTDOWN_ENABLE == status->enable_shutdown)
   {
		edl_thermal_write_register(0x04, 0xB2, 0x0, set_trip_point);
		msleep(1);
		edl_thermal_write_register(0x04, 0xB6, 0x0, TTB_CSS);
   }
   return EDL_SUCCESS;
}

static int
edl_thermal_trip_point_setting(unsigned int set_trip_point)
{
   int read_trip_point = 0;
   _sensor_status status;

   /*The hot mode should be triped first. 
      If not, setting action will shutdown immediately
      even if the actually temp is very low. */
   edl_thermal_read_temperature(&status);
   read_trip_point = status.trip_point;

   edl_thermal_write_register(0x04, 0xB0, 0x0, (TSC_E0 | TSC_AME));
   DEBUG_MSG("[KERNEL]:trip_point = 0x%x[%d]\n", set_trip_point,
      set_trip_point);
   edl_thermal_write_register(0x04, 0xB2, 0x0, set_trip_point);
   edl_thermal_write_register(0x04, 0xB6, 0x0, TTB_CSS);
 
   return read_trip_point;
}

static edl_result_t
edl_thermal_read_sensor_status(_sensor_status * status)
{
   unsigned int re_value = 0;

   if (NULL == status) {
      return EDL_FAILURE;
   }
   re_value = edl_thermal_read_register(0x04, 0xB2);
   DEBUG_MSG("B2 register = 0x%x\n", re_value);
   status->trip_point = (re_value & 0x7F);

   re_value = edl_thermal_read_register(0x04, 0xB6);
   DEBUG_MSG("B6 register = 0x%x\n", re_value);
   if (re_value & TTB_CSS) {
      //Shutdown function is enable
      status->enable_shutdown = SHUTDOWN_ENABLE;
   } else {
      //Shutdown function is disable
      status->enable_shutdown = SHUTDOWN_DISABLE;
   }

   return EDL_SUCCESS;
}

static edl_result_t
edl_thermal_read_msr_status(_sensor_status * status, int core_count)
{
   unsigned int msr_lo[CE5300_CORE_COUNT], msr_high, i;

   if (NULL == status || core_count > CE5300_CORE_COUNT) {
      return EDL_FAILURE;
   }
 
   for (i = 0; i< core_count; i++) {
	rdmsr_on_cpu(i, MSR_IA32_MISC_ENABLE, &msr_lo[i], &msr_high);	
   	DEBUG_MSG("CPU %d, MSR register IA32_MISC_ENABLE,  msr_lo = 0x%x, msr_high = 0x%x\n", i*2, msr_lo[i], msr_high);

   	if ( (msr_lo[i] & BIT(3)) && !(msr_lo[i] & BIT(13))) {
      		//Shutdown function is enable
      		status->enable_shutdown = SHUTDOWN_ENABLE;
   	} else {
      		//Shutdown function is disable
      		status->enable_shutdown = SHUTDOWN_DISABLE;
   	}

   }
   return EDL_SUCCESS;
}

static edl_result_t 
edl_thermal_ce5300_read_temperature(_sensor_status * status)
{
	int therm_status_digital_readout;
	unsigned int temperature_ref;
	unsigned int fuse_ult_revid;
	unsigned int base_addr, m_dfx_base_addr = 0, size = DFX_MEM_SIZE;
	uint32_t ts_low, ts_high;  
	int i;

	base_addr = edl_thermal_get_base_addr();

	/* map the gpio register space to something we can actually access */

	m_dfx_base_addr = (uint32_t *)OS_MAP_IO_TO_MEM_NOCACHE(base_addr, size);
	DEBUG_MSG("m_dfx_base_addr = 0x%0x\n", m_dfx_base_addr);

	/* did it work? */
	if (m_dfx_base_addr == NULL) {
		// no
		//printk("Unable to map to: 0x%8.8X; size = 0x%x\n", base_addr, size);
		return EDL_FAILURE;
	}
	
	// Default reference temperature is Tjmax(110C)
	temperature_ref = TEMPERATURE_REF_DEFAULT;

	// Check the value of fuse_ult_revid[0:3]
        fuse_ult_revid = *(volatile unsigned *)(m_dfx_base_addr + 0xC) & 0xF;

	DEBUG_MSG("fuse_ult_revid = 0x%x\n", fuse_ult_revid);
	
	/* Release memory */
	if (m_dfx_base_addr != NULL) {
		//printk("Unmap : 0x%8.8X; size = 0x%x\n", base_addr, size);
		OS_UNMAP_IO_FROM_MEM(m_dfx_base_addr, size);
		m_dfx_base_addr = NULL;
	}

	status->sensor_type = soc;

        if ((soc == SOC_NAME_CE5300) && (fuse_ult_revid >= CE5300_DTS_TRIMMED_VER) ) {
                temperature_ref = TEMPERATURE_REF_CE5300;
        } 

	status->trip_point =  temperature_ref;


	/* 
	 * Get digital readout value
	 * CE5300 have 4 logical processors.
	 * Core 0 id is 0, core 1 id is 1 in terms of the current setting.
	 */
	for (i = 0; i < CE5300_CORE_COUNT; i++) {
#ifdef DEBUG
		rdmsr_on_cpu((3 - i), MSR_IA32_THERM_STATUS, &ts_low, &ts_high);	
		DEBUG_MSG("logic cpu  %d, ts_low=%x\n", 3 - i,  ts_low);
		therm_status_digital_readout = (ts_low & (TEMP_MASK)) >> 16;
	        DEBUG_MSG("read out=%x\n", therm_status_digital_readout);
#endif


		rdmsr_on_cpu(i, MSR_IA32_THERM_STATUS, &ts_low, &ts_high);	

	        DEBUG_MSG("cpu core %d, ts_low=%x\n", i, ts_low);
		therm_status_digital_readout = (ts_low & (TEMP_MASK)) >> 16;
	        DEBUG_MSG("read out=%x\n", therm_status_digital_readout);

		/*
		 * report status
		 */
		if (ts_low & BIT(31)) { 
			if(temperature_ref > therm_status_digital_readout)
				status->DTS_value[i] = temperature_ref - therm_status_digital_readout;
			else
				status->DTS_value[i] = therm_status_digital_readout - temperature_ref;
		}
		else {
         		printk("For Intel CE5300 Media Processor, please make sure it is a DTS fuse enabled part, or DTS has been overrided!\n");
   			return EDL_FAILURE;
		}
		
		DEBUG_MSG("DTS_value[%d] = %x\n", i, status->DTS_value[i]);

		/* 
		 * check for PROCHOT:
		 */
		if (ts_low & BIT(0)) 
		 	printk(" PROCHOT !\n"); 

		/* 
		 * check for OUT OF SPEC:
		 */
		if (ts_low & BIT(4)) 
			printk(" OUT_OF_SPEC! \n"); 

	}
   return EDL_SUCCESS;
}

static edl_result_t 
edl_thermal_ce4100_read_temperature(_sensor_status * status)
{
	int therm_status_digital_readout;
	unsigned int msr_lo, msr_high;
	unsigned int temperature_ref;
	unsigned int fuse_ult_revid;
	unsigned int base_addr, m_dfx_base_addr = 0, size = DFX_MEM_SIZE;

	base_addr = edl_thermal_get_base_addr();

	/* map the gpio register space to something we can actually access */

	m_dfx_base_addr = (uint32_t *)OS_MAP_IO_TO_MEM_NOCACHE(base_addr, size);
	//printk("m_dfx_base_addr = 0x%0x\n", m_dfx_base_addr);

	/* did it work? */
	if (m_dfx_base_addr == NULL) {
		// no
		//printk("Unable to map to: 0x%8.8X; size = 0x%x\n", base_addr, size);
		return EDL_FAILURE;
	}
	
	// Default reference temperature is Tjmax(110C)
	temperature_ref = TEMPERATURE_REF_DEFAULT;

	// Check the value of fuse_ult_revid[7:4]
	 fuse_ult_revid = *(volatile unsigned *)(m_dfx_base_addr + 0x0) >> 4 & 0xF;

	 //printk("fuse_ult_revid = 0x%x\n", fuse_ult_revid);
	
	/* Release memory */
	if (m_dfx_base_addr != NULL) {
		//printk("Unmap : 0x%8.8X; size = 0x%x\n", base_addr, size);
		OS_UNMAP_IO_FROM_MEM(m_dfx_base_addr, size);
		m_dfx_base_addr = NULL;
	}

	status->sensor_type = soc;

	if ((soc == SOC_NAME_CE4100) && (fuse_ult_revid >= CE4100_DTS_TRIMMED_VER) ) {
    		temperature_ref = TEMPERATURE_REF_CE4100;
	}
	else if ((soc == SOC_NAME_CE4200) && ((stepping >= SOC_STEPPING_C0) || (CE4200_DTS_TRIMMED_VER == fuse_ult_revid) )) {
		temperature_ref = TEMPERATURE_REF_CE4200;
	}

	status->trip_point  = temperature_ref; 

	// read IA32_THERM_STATUS
	rdmsr(0x19C, msr_lo, msr_high);

	// get digital readout value
	therm_status_digital_readout = (msr_lo&(TEMP_MASK))>>16;

	// report status
	if (msr_lo & BIT(31)){ 
		if(temperature_ref > therm_status_digital_readout)
			status->trip_point = temperature_ref -therm_status_digital_readout;
		else
			status->trip_point = therm_status_digital_readout - temperature_ref;
	}
	else  {
   		return EDL_FAILURE;
	}
		

	// check for PROCHOT:
	if (msr_lo & BIT(0)) 
		 printk(" PROCHOT !\n"); 

	// check for OUT OF SPEC:
	if (msr_lo & BIT(4)) 
		printk(" OUT_OF_SPEC! \n"); 

   return EDL_SUCCESS;
}

#define ASM __asm__ __volatile__ (

#define EASM );

/****************************************************************************/
uint32_t readMsrLow(uint32_t msr)
{
         uint32_t returnVal = 0;

         ASM
                   "rdmsr;mov %%eax, %0"
                   : "=g" (returnVal)
                   : "c" (msr)
                   : "%eax", "%edx"
         EASM

         return returnVal;
}

/****************************************************************************/
uint32_t readMsrHigh(uint32_t msr)
{
         uint32_t returnVal = 0;

         ASM
                   "rdmsr;mov %%edx, %0"
                   : "=g" (returnVal)
                   : "c" (msr)
                   : "%eax", "%edx"
         EASM

         return returnVal;
}

/****************************************************************************/
void modifyMsrLow(uint32_t msr, uint32_t mask, uint32_t value)
{
         ASM
                   "rdmsr;notl %1;andl %1, %%eax;orl %2, %%eax;wrmsr"
                   : /* no outputs */
                   : "c" (msr), "m" (mask), "m" (value)
                   : "%eax", "%edx"
         EASM

         return;
}

/****************************************************************************/
void modifyMsrHigh(uint32_t msr, uint32_t mask, uint32_t value)
{
         ASM
                   "rdmsr;notl %1;andl %1, %%edx;orl %2, %%edx;wrmsr"
                   : /* no outputs */
                   : "c" (msr), "m" (mask), "m" (value)
                   : "%eax", "%edx"

         EASM

         return;
}

static edl_result_t
edl_thermal_ce5300_auto_shutdown_enable(void)
{
	uint32_t l = 0, h = 0;
        int i;
	/*
	 * CE5300 only supports TM1
	 */
        for (i = 0; i < CE5300_CORE_COUNT; i++) {	
	 	rdmsr_on_cpu(i, MSR_IA32_MISC_ENABLE, &l, &h);
	        l |= (0x1 << 3);	
		l &=  ~(0x1 << 13);
		l |= (0x0 << 13);
	 	wrmsr_on_cpu(i, MSR_IA32_MISC_ENABLE, l, h);

		l = 0;
	 	rdmsr_on_cpu(i, MSR_IA32_MISC_ENABLE, &l, &h);

		if (!(l & 0x8)) { 
		   	printk("enable core %d auto shutdown failed\n", i);
			return EDL_FAILURE;
		} 
	} 
	return EDL_SUCCESS; 
} 

static edl_result_t 
edl_thermal_ce5300_auto_shutdown_disable(void)
{
	uint32_t l = 0, h = 0;
        int i;
	/*
	 * CE5300 only supports TM1
	 */
        for (i = 0; i < CE5300_CORE_COUNT; i++) {	
	 	rdmsr_on_cpu(i, MSR_IA32_MISC_ENABLE, &l, &h);
		DEBUG_MSG("l0 = %x\n", l);
	        l &= ~(0x1 << 3);	
		l |= (0x0 << 3);
		l &= ~(0x1 << 13);
		l |= (0x0 << 13);
	 	wrmsr_on_cpu(i, MSR_IA32_MISC_ENABLE, l, h);

		l = 0;
	 	rdmsr_on_cpu(i, MSR_IA32_MISC_ENABLE, &l, &h);

		DEBUG_MSG("l1 = %x\n", l);
		if (l & 0x8) { 
		   	printk("disable core %d auto shutdown failed\n", i);
			return EDL_FAILURE;
		}
	}
	return EDL_SUCCESS;
}


static edl_result_t
edl_thermal_ce4100_auto_shutdown_enable(void)
{
	uint32_t val =0;

    	modifyMsrLow(0x1A0, 0x00000008, 0x00000008);
    	modifyMsrLow(0x1A0, 0x00002000, 0x00000000);

 	edl_thermal_write_register(0x02, 0x30, 0x4000000, 0x4000000);
 	edl_thermal_write_register(0x02, 0x30, 0x8000000, 0);

	val = readMsrLow(0x1A0);
	if (!(val & 0x8) || (val & 0x00002000)) 
		return EDL_FAILURE;

	return EDL_SUCCESS;
}

static edl_result_t 
edl_thermal_ce4100_auto_shutdown_disable(void)
{
	uint32_t val =0;

    	modifyMsrLow(0x1A0, 0x00000008, 0x00000000);
    	modifyMsrLow(0x1A0, 0x00002000, 0x00002000);

 	edl_thermal_write_register(0x02, 0x30, 0x4000000, 0x4000000);
 	edl_thermal_write_register(0x02, 0x30, 0x8000000, 0);

	val = readMsrLow(0x1A0);
	if ((val & 0x8) || !(val & 0x00002000)) 
		return EDL_FAILURE;

	return EDL_SUCCESS;
}

edl_result_t
edl_thermal_file_info(char *p)
{
//  char *s, *d;
   char *d;
   const char *s;

   /* check for NULL pointer */
   if ((char *)NULL == p) {
      return (EDL_NULL_POINTER);
   }

   d = p;
   s = Intel_CEG_File_Info;

   while (*s) {
      *d = *s;
      d++;
      s++;
   }

   *d = '\0';

   return (EDL_SUCCESS);
}

#ifdef VER
edl_result_t
edl_thermal_version_string(char *p)
{
   char *s, *d;

   /* check for NULL pointer */
   if ((char *)NULL == p) {
      return (EDL_NULL_POINTER);
   }

   d = p;
   s = (char *)version_string;

   while (*s) {
      *d = *s;
      d++;
      s++;
   }

   *d = '\0';

   return (EDL_SUCCESS);
}
#endif

static int
edl_thermal_open(struct inode *inode, struct file *filp)
{
   if (device_open)
      return -EBUSY;
   device_open++;

   DEBUG_MSG(KERN_INFO "%s:%4i: %s (pid %d) opened EDL thermal drv.\n",
      __FILE__, __LINE__, current->comm, current->pid);

   return (0);
}

static int
edl_thermal_release(struct inode *inode, struct file *filp)
{
   device_open--;

   DEBUG_MSG(KERN_INFO "%s:%4i: %s (pid %d) released EDL thermal drv.\n",
      __FILE__, __LINE__, current->comm, current->pid);

   return (0);
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 35)
static int edl_thermal_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,  unsigned long arg)
#else
static int edl_thermal_ioctl(struct file *filp, unsigned int cmd,  unsigned long arg)
#endif
{
   unsigned int read_trip_point = 0;
   unsigned int set_trip_point = 0;
   _sensor_status sensor_status;
   edl_result_t thermal_status = EDL_SUCCESS;

   memset(&sensor_status, 0x0, sizeof(sensor_status));
   /* Make sure that we haven't erroneously received the ioctl call */
   if (_IOC_TYPE(cmd) != EDL_THERMAL_IOC_MAGIC) {
      printk(KERN_ERR
         "%s:%4i: ioctl command %x does not belong to this driver\n", __FILE__,
         __LINE__, cmd);
      return -ENOTTY;
   }

   DEBUG_MSG(KERN_INFO "%s:%4i: Driver received ioctl command: %x\n", __FILE__,
      __LINE__, cmd);

   switch (cmd) {

   case EDL_THERMAL_IOC_TSC:
      /* make sure we have a valid pointer to our parameters */
      if (!arg)
         return -EINVAL;

      /* read the parameters from user */
      if (copy_from_user(&sensor_status, (void *)arg, sizeof(sensor_status))) {
         printk(KERN_ERR "%s:%4i: could not copy from user\n",
            __FILE__, __LINE__);
         return EDL_FAILURE;
      }
      DEBUG_MSG("[KERNEL]: Action = %d\n", sensor_status.enable_shutdown);
      thermal_status = edl_thermal_sensor_control(&sensor_status);
      if (EDL_SUCCESS != thermal_status) {
         thermal_status = -EINVAL;
      }
      break;

   case EDL_THERMAL_IOC_TRR:
	  if (SOC_NAME_CE3100 == soc)
      		thermal_status = edl_thermal_read_temperature(&sensor_status);
	  else if (SOC_NAME_CE5300 == soc)
	  	thermal_status = edl_thermal_ce5300_read_temperature(&sensor_status);
	  else
	  	thermal_status = edl_thermal_ce4100_read_temperature(&sensor_status);

      if (sensor_status.trip_point > MAX_TRIP_POINT) {
         thermal_status = -EINVAL;
      } else {
         /* read the parameters from user */
         if (copy_to_user((void *)arg, &sensor_status, sizeof(sensor_status))) {
            printk(KERN_ERR "%s:%4i: could not copy to user\n",
               __FILE__, __LINE__);
            return EDL_FAILURE;
         }
      }
      break;

   case EDL_THERMAL_IOC_TPSTC:
      /* make sure we have a valid pointer to our parameters */
      if (SOC_NAME_CE3100 != soc){
         printk("Setting threshold is only support for Intel CE 3100 Media Processor!\n");
         return -EINVAL;
	  }

      if (!arg)
         return -EINVAL;

      /* read the parameters from user */
      if (copy_from_user(&set_trip_point, (void *)arg, sizeof(set_trip_point))) {
         printk(KERN_ERR "%s:%4i: could not copy from user\n",
            __FILE__, __LINE__);
         return EDL_FAILURE;
      }

      read_trip_point = edl_thermal_trip_point_setting(set_trip_point);
      if ( read_trip_point > MAX_TRIP_POINT) {
         thermal_status = -EINVAL;
      } else {
         return read_trip_point;
      }
      break;

   case EDL_THERMAL_IOC_TSIS:
      /* make sure we have a valid pointer to our parameters */
      if (!arg)
         return -EINVAL;

	  if (SOC_NAME_CE3100 == soc)
	      thermal_status = edl_thermal_read_sensor_status(&sensor_status);
    	  else if (SOC_NAME_CE5300 ==  soc)
	      thermal_status = edl_thermal_read_msr_status(&sensor_status, CE5300_CORE_COUNT);
	  else 
	      thermal_status = edl_thermal_read_msr_status(&sensor_status, 1);

      /* read the parameters from user */
      if (copy_to_user((void *)arg, &sensor_status, sizeof(sensor_status))) {
         printk(KERN_ERR "%s:%4i: could not copy to user\n",
            __FILE__, __LINE__);
         return EDL_FAILURE;
      }

      if (EDL_SUCCESS != thermal_status) {
         thermal_status = -EINVAL;
      }
      break;
   default:
      thermal_status = -ENOTTY;
   }

   return thermal_status;
}

int edl_thermal_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    _sensor_status sensor_status;
    int len;
    char msg[100];
    memset(msg, 0x0, 100);
    if (SOC_NAME_CE3100 == soc)
        edl_thermal_read_sensor_status(&sensor_status);
    else if (SOC_NAME_CE5300 ==  soc)
        edl_thermal_read_msr_status(&sensor_status, CE5300_CORE_COUNT);
    else
        edl_thermal_read_msr_status(&sensor_status, 1);

    if (SHUTDOWN_ENABLE == sensor_status.enable_shutdown)
        strncpy(msg, "Shutdown function: enable\n", sizeof(msg)-1);
    else
        strncpy(msg, "Shutdown function: disable\n", sizeof(msg)-1);

    len = strlen(msg);

    if (off >= len)
        return 0;

    if (count > len - off)
        count = len - off;

    memcpy(page + off, msg + off, count);
    return off + count;
}

static struct file_operations edl_thermal_fops = {
   .owner = THIS_MODULE,
   .open = edl_thermal_open,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 35)
   .ioctl = edl_thermal_ioctl,
#else
   .unlocked_ioctl = edl_thermal_ioctl,
#endif
   .release = edl_thermal_release,
};

static int
edl_thermal_init(void)
{
   struct proc_dir_entry *proc;
   unsigned int root_device_id;
    pal_soc_info_t pal_info;

	if (PAL_SUCCESS != pal_get_soc_info(&pal_info)) 
	{
		OS_ERROR("Cann't the SOC info!\n");
		return EDL_FAILURE;
	} 
	
	soc = pal_info.name;
	stepping = pal_info.stepping;

/* we should not remove the proc at init function. */
//   remove_proc_entry(EDL_THERMAL_DEVICE_NAME, NULL);

   proc = create_proc_entry(EDL_THERMAL_DEVICE_NAME, S_IRUSR | S_IWUSR, NULL);
   if (NULL != proc) {
      memcpy(&edl_thermal_fops, proc->proc_fops, sizeof(edl_thermal_fops));
      edl_thermal_fops.open = edl_thermal_open;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 35)
      edl_thermal_fops.ioctl = edl_thermal_ioctl;
#else
      edl_thermal_fops.unlocked_ioctl = edl_thermal_ioctl;
#endif
      edl_thermal_fops.release = edl_thermal_release;
      proc->proc_fops = &edl_thermal_fops;
      proc->read_proc = edl_thermal_proc_read;
   } else {
      printk("create_proc_entry(\"%s\") failed\n", EDL_THERMAL_DEVICE_NAME);
      return -EIO;
   }

   /* we need to register our device with the OS */
   /* We'll dynamically ask the OS to assign us an available major */
   /* number. */
   if ((major_number =
         register_chrdev(0, EDL_THERMAL_DEVICE_NAME, &edl_thermal_fops)) < 0) {
      /* Error occured with device registration */
      printk(KERN_ERR
         "%s:%4i: EDL thermal device registration failed - code %d\n",
         __FILE__, __LINE__, major_number);
   }

   printk(EDL_THERMAL_DEVICE_NAME " init complete! (build " __DATE__ ")\n");
   printk("major number =%d\n", major_number);

   //edl_thermal_trip_point_setting(MAX_TRIP_POINT >> 1);

   return 0;
}

static void
edl_thermal_exit(void)
{
   unregister_chrdev(major_number, EDL_THERMAL_DEVICE_NAME);
   remove_proc_entry(EDL_THERMAL_DEVICE_NAME, NULL);
   printk(KERN_NOTICE EDL_THERMAL_DEVICE_NAME " unloaded\n");
}

module_init(edl_thermal_init);
module_exit(edl_thermal_exit);
