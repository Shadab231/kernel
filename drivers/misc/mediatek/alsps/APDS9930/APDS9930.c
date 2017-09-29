/* drivers/hwmon/mt6516/amit/APDS9930.c - APDS9930 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "APDS9930.h"
#include <linux/sched.h>
//LINE<JIRA_ID><DATE20131219><add PS Calibration>zenghaihui
static int g_ps_cali_flag = 0;
static int g_ps_base_value = 0;
static int g_tp_tpye_checked = 0;
extern char * synaptics_get_vendor_info(void);
static void ap3220_ps_cali_tp_check(void);
static void ap3220_ps_cali_set_threshold(void);
static void ap3220_ps_cali_start(void);
static int set_psensor_threshold(struct i2c_client *client);
/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define APDS9930_DEV_NAME     "APDS9930"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_ERR APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args) 

#define I2C_FLAG_WRITE	0
#define I2C_FLAG_READ	1

int APDS9930_CMM_PPCOUNT_VALUE = 0x08;   //Pulse Number
int APDS9930_CMM_CONTROL_VALUE = 0x2A;   //PS 4X gain，ALS 16X gain
int ZOOM_TIME = 0x01;


/******************************************************************************
 * extern functions
*******************************************************************************/
#ifdef CUST_EINT_ALS_TYPE
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);
#else
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_client *APDS9930_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id APDS9930_i2c_id[] = {{APDS9930_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_APDS9930={ I2C_BOARD_INFO("APDS9930", 0x39)};

/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int APDS9930_i2c_remove(struct i2c_client *client);
static int APDS9930_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int APDS9930_i2c_resume(struct i2c_client *client);

static DEFINE_MUTEX(APDS9930_mutex);


static struct APDS9930_priv *g_APDS9930_ptr = NULL;

 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
static unsigned long long int_top_time = 0;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct APDS9930_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct APDS9930_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct APDS9930_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver APDS9930_i2c_driver = {	
	.probe      = APDS9930_i2c_probe,
	.remove     = APDS9930_i2c_remove,
	.detect     = APDS9930_i2c_detect,
	.suspend    = APDS9930_i2c_suspend,
	.resume     = APDS9930_i2c_resume,
	.id_table   = APDS9930_i2c_id,
	.driver = {
		.name           = APDS9930_DEV_NAME,
	},
};

static struct APDS9930_priv *APDS9930_obj = NULL;
static struct platform_driver APDS9930_alsps_driver;
/*------------------------i2c function for 89-------------------------------------*/
int APDS9930_i2c_master_operate(struct i2c_client *client, const char *buf, int count, int i2c_flag)
{
	int res = 0;
	mutex_lock(&APDS9930_mutex);
	switch(i2c_flag){	
	case I2C_FLAG_WRITE:
	client->addr &=I2C_MASK_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	
	case I2C_FLAG_READ:
	client->addr &=I2C_MASK_FLAG;
	client->addr |=I2C_WR_FLAG;
	client->addr |=I2C_RS_FLAG;
	res = i2c_master_send(client, buf, count);
	client->addr &=I2C_MASK_FLAG;
	break;
	default:
	APS_LOG("APDS9930_i2c_master_operate i2c_flag command not support!\n");
	break;
	}
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&APDS9930_mutex);
	return res;
	EXIT_ERR:
	mutex_unlock(&APDS9930_mutex);
	APS_ERR("APDS9930_i2c_transfer fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
int APDS9930_get_addr(struct alsps_hw *hw, struct APDS9930_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void APDS9930_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "APDS9930")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "APDS9930")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static long APDS9930_enable_als(struct i2c_client *client, int enable)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];	  
	long res = 0;

	databuf[0]= APDS9930_CMM_ENABLE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_LOG("APDS9930_CMM_ENABLE als value = %x\n",databuf[0]);
	
	if(enable)
		{
			databuf[1] = databuf[0]|0x03;
			databuf[0] = APDS9930_CMM_ENABLE;
			//APS_LOG("APDS9930_CMM_ENABLE enable als value = %x\n",databuf[1]);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		}
	else {
		if(test_bit(CMC_BIT_PS, &obj->enable))
			databuf[1] = databuf[0]&0xFD;
		else
			databuf[1] = databuf[0]&0xF8;
		
			databuf[0] = APDS9930_CMM_ENABLE;
			//APS_LOG("APDS9930_CMM_ENABLE disable als value = %x\n",databuf[1]);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
	return 0;
		
EXIT_ERR:
	APS_ERR("APDS9930_enable_als fail\n");
	return res;
}

//static int value_count=0;
static int value_flag=0;
static int value_first=1024;


/*----------------------------------------------------------------------------*/
static long APDS9930_enable_ps(struct i2c_client *client, int enable)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	long res = 0;

	// value_count=0;
	 value_flag=0;
	 value_first=1024;

	
	databuf[0]= APDS9930_CMM_ENABLE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	//APS_LOG("APDS9930_CMM_ENABLE ps value = %x\n",databuf[0]);
	
	if(enable)
		{
			databuf[1] = databuf[0]|0x05;
			databuf[0] = APDS9930_CMM_ENABLE;
			//APS_LOG("APDS9930_CMM_ENABLE enable ps value = %x\n",databuf[1]);	
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		}
	else{
		if(test_bit(CMC_BIT_ALS, &obj->enable))
			databuf[1] = databuf[0]&0xFB;
		else
			databuf[1] = databuf[0]&0xF8;
		
			databuf[0] = APDS9930_CMM_ENABLE;
			//APS_LOG("APDS9930_CMM_ENABLE disable ps value = %x\n",databuf[1]);	
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*fix bug*/
			databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
        databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
        databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;	
        databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*fix bug*/
		}
	return 0;
	
EXIT_ERR:
	APS_ERR("APDS9930_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int APDS9930_check_and_clear_intr(struct i2c_client *client) 
{
	int res,intp,intl;
	u8 buffer[2];

	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	    return 0;

	buffer[0] = APDS9930_CMM_STATUS;
	res = APDS9930_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	res = 0;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 1;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 1;
		intl = 1;		
	}

	if(1 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}

		res = APDS9930_i2c_master_operate(client, buffer, 0x1, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("APDS9930_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int APDS9930_check_intr(struct i2c_client *client) 
{
	int res,intp,intl;
	u8 buffer[2];

	if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	return 0;

	buffer[0] = APDS9930_CMM_STATUS;
	res = APDS9930_i2c_master_operate(client, buffer, 0x101, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = 0;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("APDS9930_check_intr fail\n");
	return 1;
}

static int APDS9930_clear_intr(struct i2c_client *client) 
{
	int res;
	u8 buffer[2];
	
	buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
	res = APDS9930_i2c_master_operate(client, buffer, 0x1, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}
	return res;

EXIT_ERR:
	APS_ERR("APDS9930_check_and_clear_intr fail\n");
	return 1;
}


/*-----------------------------------------------------------------------------*/
void APDS9930_eint_func(void)
{
	struct APDS9930_priv *obj = g_APDS9930_ptr;
	if(!obj)
	{
		return;
	}
	int_top_time = sched_clock();
	schedule_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int APDS9930_setup_eint(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);        

	g_APDS9930_ptr = obj;
	
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#ifdef CUST_EINT_ALS_TYPE
    mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, APDS9930_eint_func, 0);
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, APDS9930_eint_func, 0);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/

static int APDS9930_init_client(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;

	databuf[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x00);
	res = APDS9930_i2c_master_operate(client, databuf, 0x1, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	databuf[0] = APDS9930_CMM_ENABLE;
	if(obj->hw->polling_mode_ps == 1)
	databuf[1] = 0x08;
	if(obj->hw->polling_mode_ps == 0)
	databuf[1] = 0x28;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	databuf[0] = APDS9930_CMM_ATIME;    
    databuf[1] = 0xDB;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	databuf[0] = APDS9930_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	databuf[0] = APDS9930_CMM_WTIME;    
    databuf[1] = 0xFF;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{
		if(1 == ps_cali.valid)
		{
			databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(ps_cali.close & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;	
			databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		else
		{
			databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
            databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
            databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
            databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;	
            databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);;
			res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}

		}

		databuf[0] = APDS9930_CMM_Persistence;
        databuf[1] = 0x00;
		res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

	}

	databuf[0] = APDS9930_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

       /*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = APDS9930_CMM_PPCOUNT;    
	databuf[1] = APDS9930_CMM_PPCOUNT_VALUE;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

        /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = APDS9930_CMM_CONTROL;    
	databuf[1] = 0x20;//APDS9930_CMM_CONTROL_VALUE;//<BUG><lower ps gain><20140812>yinhuiyong
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if((res = APDS9930_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if((res = APDS9930_check_and_clear_intr(client)))
	{
		APS_ERR("check/clear intr: %d\n", res);
	    return res;
	}
	
	return APDS9930_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}


/*#ifdef APDS9930_WHITE_PANEL
#define APDS9930_GA     309
#define APDS9930_COE_B      175
#define APDS9930_COE_C      64
#define APDS9930_COE_D      108
#define APDS9930_DF     52
#endif
#ifdef APDS9930_BLACK_PANEL
#define APDS9930_GA     405
#define APDS9930_COE_B      178
#define APDS9930_COE_C      45
#define APDS9930_COE_D      76
#define APDS9930_DF     52
#endif*/

int APDS9930_GA, APDS9930_COE_B, APDS9930_COE_C, APDS9930_COE_D, APDS9930_DF;

#define CMD_WORD        0xA0
static int tp_color_val = 1;   //default black panel

static int LuxCalculation(struct i2c_client *client, int ch0data, int ch1data)
{
    int luxValue=0;

    int IAC1=0;
    int IAC2=0;
    int IAC=0;

    if(tp_color_val ==2)   //white panel
    {
        APDS9930_GA=309;
        APDS9930_COE_B=175;
        APDS9930_COE_C  =64;
        APDS9930_COE_D=108;
        APDS9930_DF=52;
    }
    else
    {
        APDS9930_GA=405;
        APDS9930_COE_B=178;
        APDS9930_COE_C  =45;
        APDS9930_COE_D=76;
        APDS9930_DF=52;
    }

    IAC1 = (ch0data - (APDS9930_COE_B*ch1data)/100);            // re-adjust COE_B to avoid 2 decimal point
    IAC2 = ((APDS9930_COE_C*ch0data)/100 - (APDS9930_COE_D*ch1data)/100);   // re-adjust COE_C and COE_D to void 2 decimal point

    if (IAC1 > IAC2)
        IAC = IAC1;
    else if (IAC1 <= IAC2)
        IAC = IAC2;
    else
        IAC = 0;

    if (IAC1<0 && IAC2<0)
    {
        IAC = 0;    // cdata and irdata saturated
        return -1;  // don't report first, change gain may help
    }

    APS_LOG("IAC1=%d, IAC2=%d, IAC=%d ---CYY\n", IAC1, IAC2, IAC);

    luxValue = ((IAC*APDS9930_GA*APDS9930_DF)/100)*4/((10064/100)*16);


    return luxValue;
}


static ssize_t APDS9930_show_tp_color(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    len = snprintf(buf, PAGE_SIZE, "%d\n", tp_color_val);
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t APDS9930_store_tp_color(struct device_driver *ddri, char *buf, size_t count)
{
    int color;

    APS_LOG("Entry APDS9930_store_tp_color buf string=%s\n", buf);
    if(1 == sscanf(buf, "%d", &color))
    {
        APS_LOG("color=%d\n", color);
        if((color < 1) || (color > 2))
            tp_color_val = 1;
        else
            tp_color_val = color;
    }
    else
    {
        APS_LOG("invalid content: '%s', length = %d\n", buf, count);
    }
    return count;
}


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(tp_color,   0664, APDS9930_show_tp_color, APDS9930_store_tp_color);

/*----------------------------------------------------------------------------*/
static struct device_attribute *apds9930_attr_list[] =
{
    &driver_attr_tp_color,
};
/*----------------------------------------------------------------------------*/
static int APDS9930_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(apds9930_attr_list)/sizeof(apds9930_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, apds9930_attr_list[idx]))
        {
            APS_ERR("driver_create_file (%s) = %d\n", apds9930_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(apds9930_attr_list)/sizeof(apds9930_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, apds9930_attr_list[idx]);
    }

    return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int APDS9930_read_als(struct i2c_client *client, u16 *data)
{	 
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	u8 buffer[2];
	u16 atio;
	int res = 0;
    int ch0data, ch1data;;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

    /*if(tp_color_val == 2)
        APS_LOG("White Panel ---CYY\n");
    else
        APS_LOG("BLack Panel ---CYY\n");
    */

    ch0data = i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CMM_C0DATA_L);
    ch1data = i2c_smbus_read_word_data(client, CMD_WORD|APDS9930_CMM_C1DATA_L);



#if 0
	buffer[0]=APDS9930_CMM_C0DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	c0_value = buffer[0] | (buffer[1]<<8);
	c0_nf = obj->als_modulus*c0_value;
    APS_LOG("c0_value=%d, c0_nf=%d, als_modulus=%d ---CYY\n", c0_value, c0_nf, obj->als_modulus);

	buffer[0]=APDS9930_CMM_C1DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	c1_value = buffer[0] | (buffer[1]<<8);
	c1_nf = obj->als_modulus*c1_value;	
    APS_LOG("c1_value=%d, c1_nf=%d, als_modulus=%d ---CYY\n", c1_value, c1_nf, obj->als_modulus);
	
	if((c0_value > c1_value) &&(c0_value < 50000))
	{  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;

        APS_LOG("atio = %d ---CYY\n", atio);
	if(atio<30)
	{
		*data = (13*c0_nf - 24*c1_nf)/10000;
	}
	else if(atio>= 30 && atio<38) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (16*c0_nf - 35*c1_nf)/10000;
	}
	else if(atio>= 38 && atio<45)  /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (9*c0_nf - 17*c1_nf)/10000;
	}
	else if(atio>= 45 && atio<54) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (6*c0_nf - 10*c1_nf)/10000;
	}
	else
		*data = 0;
	/*Lenovo-sw chenlj2 add 2011-06-03,add }*/
    }
	else if (c0_value > 50000)
	{
		*data = 65535;
	}
	else if(c0_value == 0)
        {
                *data = 0;
        }
        else
	{
		APS_DBG("APDS9930_read_als als_value is invalid!!\n");
		return -1;
	}	
#endif

    APS_LOG("ch0data=%d, ch1data=%d ---CYY\n", ch0data, ch1data);
    *data = LuxCalculation(client, ch0data, ch1data);
    APS_LOG("APDS9930_read_als als_value_lux = %d  ---CYY\n", *data);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}
int APDS9930_read_als_ch0(struct i2c_client *client, u16 *data)
{	 
	//struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u16 c0_value;	 
	u8 buffer[2];
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

//get adc channel 0 value
	buffer[0]=APDS9930_CMM_C0DATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	c0_value = buffer[0] | (buffer[1]<<8);
	*data = c0_value;
	//APS_LOG("c0_value=%d\n", c0_value);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int APDS9930_get_als_value(struct APDS9930_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("APDS9930_get_als_value exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
#if defined(MTK_AAL_SUPPORT)
        int level_high = obj->hw->als_level[idx];
    	int level_low = (idx > 0) ? obj->hw->als_level[idx-1] : 0;
        int level_diff = level_high - level_low;
		int value_high = obj->hw->als_value[idx];
        int value_low = (idx > 0) ? obj->hw->als_value[idx-1] : 0;
        int value_diff = value_high - value_low;
        int value = 0;
        
        if ((level_low >= level_high) || (value_low >= value_high))
            value = value_low;
        else
            value = (level_diff * value_low + (als - level_low) * value_diff + ((level_diff + 1) >> 1)) / level_diff;

		APS_DBG("ALS: %d [%d, %d] => %d [%d, %d] \n", als, level_low, level_high, value, value_low, value_high);
		return value;
#endif			        
        APS_ERR("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		return obj->hw->als_value[idx];
	}
	else
	{
        APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
long APDS9930_read_ps(struct i2c_client *client, u16 *data)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);	
	u8 buffer[2];
	u16 temp_data;
	long res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=APDS9930_CMM_PDATA_L;
	res = APDS9930_i2c_master_operate(client, buffer, 0x201, I2C_FLAG_READ);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	temp_data = buffer[0] | (buffer[1]<<8);
	//APS_LOG("yucong APDS9930_read_ps ps_data=%d, low:%d  high:%d", *data, buffer[0], buffer[1]);
	if(temp_data < obj->ps_cali)
		*data = 0;
	else
		*data = temp_data - obj->ps_cali;
	return 0;	
	return 0;    

EXIT_ERR:
	APS_ERR("APDS9930_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/


static int APDS9930_check_start( u16 low_ps,u16 ps )
{
APS_FUN(f) ;
if(!value_flag)
{
			
	if((ps < low_ps))
        {
		value_flag=1;
		value_first=ps;		
        }
	else if (value_first>ps)
        {
			value_first=ps;
        }

}

APS_LOG("APDS9930_check_start low_ps=%d, ps  = %d,value_first=%d,value_flag=%d\n",low_ps,ps,value_first,value_flag);

  return 0;
		
}

static int APDS9930_get_ps_value(struct APDS9930_priv *obj, u16 ps)
{
	int val;// mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=1;

	int res=0;
	int ps_value_low=0;


	if(ps_cali.valid == 1)
	{
		ps_value_low= ps_cali.far_away;
		res=APDS9930_check_start(ps_value_low,ps);
		if(value_flag==1)
		{
			if((ps >ps_cali.close))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			
			else if((ps < ps_cali.far_away))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
				val = val_temp;
		}
		else
		{
			if((ps >(value_first+300)))
		        {
		            val = 0;  /*close*/
		            val_temp = 0;
		            intr_flag_value = 1;
		        }

		        else if((ps < (value_first+180)))
		        {
		            val = 1;  /*far away*/
		            val_temp = 1;
		            intr_flag_value = 0;
		        }
		        else
		            val = val_temp;			
		}

			APS_LOG("APDS9930_get_ps_value val  = %d",val);
	}
	else
	{
		ps_value_low= atomic_read(&obj->ps_thd_val_low);
		res=APDS9930_check_start(ps_value_low,ps);

		if(value_flag==1)
		{
			if((ps  > atomic_read(&obj->ps_thd_val_high)))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			else if((ps  < atomic_read(&obj->ps_thd_val_low)))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
			       val = val_temp;	
			}	
		else
		{
			if((ps >(value_first+300)))
		        {
		            val = 0;  /*close*/
		            val_temp = 0;
		            intr_flag_value = 1;
		        }

		        else if((ps < (value_first+180)))
		        {
		            val = 1;  /*far away*/
		            val_temp = 1;
		            intr_flag_value = 0;
		        }
		        else
		            val = val_temp;			
		}
	}
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}


/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
//#define DEBUG_APDS9930
static void APDS9930_eint_work(struct work_struct *work)
{
	struct APDS9930_priv *obj = (struct APDS9930_priv *)container_of(work, struct APDS9930_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
	u8 databuf[3];
	int res = 0;

	if((err = APDS9930_check_intr(obj->client)))
	{
		APS_ERR("APDS9930_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		APDS9930_read_ps(obj->client, &obj->ps);
		APDS9930_read_als_ch0(obj->client, &obj->als);
		APS_LOG("APDS9930_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		APS_LOG("APDS9930 int top half time = %lld\n", int_top_time);

		if(obj->als > 40000)
			{
			APS_LOG("APDS9930_eint_work ALS too large may under lighting als_ch0=%d!\n",obj->als);
			return;
			}
		sensor_data.values[0] = APDS9930_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	

#ifdef DEBUG_APDS9930
		databuf[0]= APDS9930_CMM_ENABLE;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x101, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_ENABLE ps value = %x\n",databuf[0]);
		
		databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_INT_LOW_THD_LOW before databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);

		databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_INT_HIGH_THD_LOW before databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);
#endif
#if 0
/*singal interrupt function add*/
		if(intr_flag_value){
						databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
						databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
						databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
						databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
						databuf[1] = (u8)(0x00FF);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
						databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH; 
						databuf[1] = (u8)((0xFF00) >> 8);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
				}
				else{	
						databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
						databuf[1] = (u8)(0 & 0x00FF);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
						databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH;	
						databuf[1] = (u8)((0 & 0xFF00) >> 8);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
						
						databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;	
						databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
					
						databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH; 
						databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);
						res = APDS9930_i2c_master_operate(obj->client, databuf, 0x2, I2C_FLAG_WRITE);
						res = i2c_master_send(obj->client, databuf, 0x2);
						if(res <= 0)
						{
							goto EXIT_ERR;
						}
				}
#endif
		//let up layer to know
		#ifdef DEBUG_APDS9930
		databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_INT_LOW_THD_LOW after databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);

		databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW;
		res = APDS9930_i2c_master_operate(obj->client, databuf, 0x201, I2C_FLAG_READ);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("APDS9930_eint_work APDS9930_CMM_INT_HIGH_THD_LOW after databuf[0]=%d databuf[1]=%d!\n",databuf[0],databuf[1]);
		#endif
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	
	APDS9930_clear_intr(obj->client);
#ifdef CUST_EINT_ALS_TYPE
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
    return;
EXIT_ERR:
    APDS9930_clear_intr(obj->client);
#ifdef CUST_EINT_ALS_TYPE
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
	APS_ERR("i2c_transfer error = %d\n", res);
	return;
}


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int APDS9930_open(struct inode *inode, struct file *file)
{
	file->private_data = APDS9930_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int APDS9930_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

//LINE<JIRA_ID><DATE20131219><add PS Calibration>zenghaihui
static void ap3220_ps_cali_tp_check(void)
{
#if 0
    u8 data, data2, data3;
    int res =0;
    char *product_id = NULL;

    if(g_tp_tpye_checked)
    {
        APS_LOG("ap3220_ps_tp_check tp already checked \n");
        return;
    }
    
    product_id = synaptics_get_vendor_info();
    
    APS_LOG("ap3220_ps_tp_check product_id = %s \n", product_id);
    
    if( 0 == memcmp(product_id, "JTOUCH", 6))
    {
        //PS LED Control
        data = (AP3220_PS_SETTING_LED_PULSE_2<< AP3220_PS_LED_PULSE_SHIFT) & AP3220_PS_LED_PULSE_MASK;
        data2 = (AP3220_PS_SETTING_LED_RATIO_16 << AP3220_PS_LED_RATIO_SHIFT) & AP3220_PS_LED_RATIO_MASK;
        data |= data2;
        res = ap3220_i2c_write_reg(AP3220_REG_PS_LED,data);
        
        if(res < 0)
        {
            APS_LOG("i2c_master_send function err in ap3220_jtouch_white_tp_threshold_reset\n");
        }
        
        mdelay(50);
    }

    g_tp_tpye_checked = 1;
#endif    
}

static void ap3220_ps_cali_set_threshold(void)
{
	u8 data, data2, data3;
	u16 value_high,value_low;
	int res =0;

    struct alsps_hw *hw = get_cust_alsps_hw();

    APS_LOG("ap3220_ps_cali_set_threshold:g_ps_base_value=%x, hw->ps_threshold_high=%x,hw->ps_threshold_low=%x \n",
        g_ps_base_value, hw->ps_threshold_high, hw->ps_threshold_low);
    
    value_high= g_ps_base_value + hw->ps_threshold_high;
    value_low= g_ps_base_value + hw->ps_threshold_low;

    
    if( value_high > 0x3f0)
    {
        value_high= 0x3f0;
        value_low= 0x3e0;
        APS_LOG("ap3220_ps_cali_set_threshold: set value_high=0x3f0,value_low=0x3e0, please check the phone \n");
    }
    

        
	atomic_set(&APDS9930_obj->ps_thd_val_high, value_high);
	atomic_set(&APDS9930_obj->ps_thd_val_low,  value_low);

	set_psensor_threshold(APDS9930_obj->client);
	/*
	res = ap3220_i2c_write_reg(0x2A,value_low&0x0003);
	res = ap3220_i2c_write_reg(0x2B,(value_low>>2)&0x00ff);
	res = ap3220_i2c_write_reg(0x2C,value_high&0x0003);
	res = ap3220_i2c_write_reg(0x2D,(value_high>>2)&0x00ff);
	*/
	APS_LOG("ap3220_ps_cali_set_threshold:value_high=%x,value_low=%x! \n",value_high,value_low);
}

static void ap3220_ps_cali_start(void)
{
    long err = 0;
    u16 			vl_read_ps = 0;
    u16 			vl_ps_count = 0;
    u16 			vl_ps_sun = 0;
    u16 			vl_index = 0;
    
    

    APS_LOG("entry ap3220_ps_cali_start \n");
    
    if(NULL == APDS9930_obj->client)
    {
        APS_ERR("APDS9930_obj->client == NULL\n"); 
        return;
    }

    
    //ap3220_ps_cali_tp_check();

    // enable ps and backup reg data
    /*
    if((err = ap3220_enable_ps(ltr501_obj->client, 1)))
    {
        APS_ERR("enable ps fail: %ld\n", err); 
        goto exit_handle;
    }
    mdelay(50);
    */


    // read ps
    for(vl_index = 0; vl_index < 4; vl_index++)
    {
        if((err = APDS9930_read_ps(APDS9930_obj->client, &vl_read_ps)))
        {
            APS_ERR("enable ps fail: %ld\n", err); 
            goto exit_handle;
        }
        
        APS_LOG("vl_index=%d, vl_read_ps = %d \n",vl_index, vl_read_ps);

        if(vl_index >=2)
        {
            vl_ps_sun += vl_read_ps;
            
            vl_ps_count ++;
        }
        
        vl_read_ps = 0;
        
        mdelay(30);
    }

    g_ps_base_value = (vl_ps_sun/vl_ps_count);
    g_ps_cali_flag = 1;
    
    APS_LOG("ap3220_ps_cali_start:g_ps_base_value=%x \n",g_ps_base_value);
    
    
exit_handle:
	APS_LOG("tag: exit_handle\n");
    /*
    if((err = ap3220_enable_ps(ltr501_obj->client, 0)))
    {
        APS_ERR("disable ps fail: %d\n", err); 
    }
    */
    
}

/*----------------------------------------------------------------------------*/
static int set_psensor_threshold(struct i2c_client *client)
{
	struct APDS9930_priv *obj = i2c_get_clientdata(client);
	u8 databuf[3];    
	int res = 0;
	APS_ERR("set_psensor_threshold function high: 0x%x, low:0x%x\n",atomic_read(&obj->ps_thd_val_high),atomic_read(&obj->ps_thd_val_low));

	databuf[0] = APDS9930_CMM_INT_LOW_THD_LOW;	
	databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_low) & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		return -1;
	}
	databuf[0] = APDS9930_CMM_INT_LOW_THD_HIGH; 
	databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low) & 0xFF00) >> 8);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		return -1;
	}
	databuf[0] = APDS9930_CMM_INT_HIGH_THD_LOW; 
	databuf[1] = (u8)(atomic_read(&obj->ps_thd_val_high) & 0x00FF);
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		return -1;
	}
	databuf[0] = APDS9930_CMM_INT_HIGH_THD_HIGH;	
	databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high) & 0xFF00) >> 8);;
	res = APDS9930_i2c_master_operate(client, databuf, 0x2, I2C_FLAG_WRITE);
	if(res <= 0)
	{
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static long APDS9930_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct APDS9930_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	int ps_result;
	int ps_cali;
	static int factory_status = 0; //LINE<20130718><for ftm>wangyanhui
       //LINE<JIRA_ID><DATE20131219><add PS Calibration>zenghaihui        
       int ps_cali_data[2] = {0x00};
	int threshold[2];

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = APDS9930_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = APDS9930_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = APDS9930_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = APDS9930_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = APDS9930_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
                            //BEGIN<20130718><for ftm>wangyanhui
                            //LINE<JIRA_ID><DATE20130115><for ftm>zenghaihui
                            if(obj->ps > atomic_read(&obj->ps_thd_val_high))
                            {
                                //dat = 0;  /*close*/
                                factory_status = 0;
                            }
                            else if(obj->ps < atomic_read(&obj->ps_thd_val_low))
                            {
                                //dat = 1;  /*far*/
                                factory_status =1;
                            }
                            dat = factory_status;
                            //END<20130718><for ftm>wangyanhui	
                            
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = APDS9930_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = APDS9930_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = APDS9930_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = APDS9930_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = APDS9930_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
			if((err = APDS9930_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			if(obj->ps > atomic_read(&obj->ps_thd_val_high))
				{
					ps_result = 0;
				}
			else	ps_result = 1;
				
			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}			   
			break;

			case ALSPS_IOCTL_CLR_CALI:
				if(copy_from_user(&dat, ptr, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(dat == 0)
					obj->ps_cali = 0;
				break;

			case ALSPS_IOCTL_GET_CALI:
				ps_cali = obj->ps_cali ;
				if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_IOCTL_SET_CALI:
				if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
				{
					err = -EFAULT;
					goto err_out;
				}

				obj->ps_cali = ps_cali;
				break;

			case ALSPS_SET_PS_THRESHOLD:
				if(copy_from_user(threshold, ptr, sizeof(threshold)))
				{
					err = -EFAULT;
					goto err_out;
				}
				APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]); 
				atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
				atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm

				set_psensor_threshold(obj->client);
				
				break;
				
			case ALSPS_GET_PS_THRESHOLD_HIGH:
				threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
				APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]); 
				if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
				
			case ALSPS_GET_PS_THRESHOLD_LOW:
				threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
				APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]); 
				if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
           //LINE<JIRA_ID><DATE20131218><add PS Calibration>zenghaihui
            case ALSPS_IOCTL_PS_CALI_START:
                APS_LOG("case ALSPS_IOCTL_PS_CALI_START: \n");
                
                ap3220_ps_cali_start();
                
                if (ptr == NULL) {
                    APS_ERR("%s ptr == NULL", __FUNCTION__);
                    err = -EINVAL;
                    break;
                }
                
                ps_cali_data[0] = g_ps_cali_flag;
                ps_cali_data[1] = g_ps_base_value;
                
                APS_LOG("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);
                
                if (copy_to_user(ptr, ps_cali_data, sizeof(ps_cali_data))) {
                    APS_ERR("%s copy_from_user error", __FUNCTION__);
                    err = -EFAULT;
                    break;
                }
                break;

            case ALSPS_IOCTL_PS_SET_CALI:
                APS_LOG("case ALSPS_IOCTL_PS_SET_CALI: \n");
                
                if (ptr == NULL) {
                    APS_ERR("%s ptr == NULL", __FUNCTION__);
                    err = -EINVAL;
                    break;
                }
                
                if (copy_from_user(&ps_cali_data, ptr, sizeof(ps_cali_data))) {
                    APS_ERR("%s copy_from_user error", __FUNCTION__);
                    err = -EFAULT;
                    break;
                }

                g_ps_cali_flag = ps_cali_data[0];
                g_ps_base_value = ps_cali_data[1];

                if(!g_ps_cali_flag)
                {
                    g_ps_base_value = 0x0F; // set default base value
                    APS_LOG("not calibration!!! set g_ps_base_value = 0x80 \n");
                }
                
                APS_LOG("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);

                ap3220_ps_cali_set_threshold();
                
                break;

            case ALSPS_IOCTL_PS_GET_CALI:
                APS_LOG("case ALSPS_IOCTL_PS_GET_CALI: \n");
                
                if (ptr == NULL) {
                    APS_ERR("%s ptr == NULL", __FUNCTION__);
                    err = -EINVAL;
                    break;
                }
                
                ps_cali_data[0] = g_ps_cali_flag;
                ps_cali_data[1] = g_ps_base_value;
                
                APS_LOG("g_ps_cali_flag = %x, g_ps_base_value = %x \n", g_ps_cali_flag, g_ps_base_value);
                
                if (copy_to_user(ptr, ps_cali_data, sizeof(ps_cali_data))) {
                    APS_ERR("%s copy_to_user error", __FUNCTION__);
                    err = -EFAULT;
                    break;
                }
                break;

            case ALSPS_IOCTL_PS_CLR_CALI:
                APS_LOG("case ALSPS_IOCTL_PS_CLR_CALI: \n");
                g_ps_cali_flag = 0;
                g_ps_base_value = 0;
                ap3220_ps_cali_set_threshold();
                break;
                
            case ALSPS_IOCTL_PS_CALI_RAW_DATA:    
                if((err = APDS9930_read_ps(obj->client, &obj->ps)))
                {
                    goto err_out;
                }
                
                dat = obj->ps;
                            
                if(copy_to_user(ptr, &dat, sizeof(dat)))
                {
                    err = -EFAULT;
                    goto err_out;
                }  
                break;        
			/*------------------------------------------------------------------------------------------*/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations APDS9930_fops = {
	.owner = THIS_MODULE,
	.open = APDS9930_open,
	.release = APDS9930_release,
	.unlocked_ioctl = APDS9930_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice APDS9930_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &APDS9930_fops,
};
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
//	struct APDS9930_priv *obj = i2c_get_clientdata(client);    
//	int err;
	APS_FUN();    
#if 0
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if(err = APDS9930_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = APDS9930_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		APDS9930_power(obj->hw, 0);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_resume(struct i2c_client *client)
{
//	struct APDS9930_priv *obj = i2c_get_clientdata(client);        
//	int err;
	APS_FUN();
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	APDS9930_power(obj->hw, 1);
	if(err = APDS9930_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = APDS9930_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = APDS9930_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void APDS9930_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct APDS9930_priv *obj = container_of(h, struct APDS9930_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	#if 1
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = APDS9930_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/
static void APDS9930_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct APDS9930_priv *obj = container_of(h, struct APDS9930_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

        #if 1
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = APDS9930_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/
static int temp_als = 0;
static int ALS_FLAG = 0;

int APDS9930_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int value;
	int err = 0;
	
	hwm_sensor_data* sensor_data;
	struct APDS9930_priv *obj = (struct APDS9930_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{	
				value = *(int *)buff_in;
				if(value)
				{
					if((err = APDS9930_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					#if 1
					if(!test_bit(CMC_BIT_ALS, &obj->enable))
					{
						ALS_FLAG = 1;
						if((err = APDS9930_enable_als(obj->client, 1)))
						{
							APS_ERR("enable als fail: %d\n", err); 
							return -1;
						}
					}
					#endif
				}
				else
				{
					if((err = APDS9930_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					#if 1
					if(ALS_FLAG == 1)
					{
						if((err = APDS9930_enable_als(obj->client, 0)))
						{
							APS_ERR("disable als fail: %d\n", err); 
							return -1;
						}
						ALS_FLAG = 0;
					}
					#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				APDS9930_read_ps(obj->client, &obj->ps);
				APDS9930_read_als_ch0(obj->client, &obj->als);
				APS_ERR("APDS9930_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = APDS9930_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


int APDS9930_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct APDS9930_priv *obj = (struct APDS9930_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if((err = APDS9930_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = APDS9930_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing known issue*/
				APDS9930_read_als(obj->client, &obj->als);
				if(obj->als == 0)
				{
					sensor_data->values[0] = temp_als;				
				}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
					APDS9930_read_als(obj->client, &obj->als);
					b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = APDS9930_get_als_value(obj, obj->als);
					temp_als = sensor_data->values[0];
				}
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, APDS9930_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct APDS9930_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	APDS9930_obj = obj;
	obj->hw = get_cust_alsps_hw();
	APDS9930_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	INIT_WORK(&obj->eint_work, APDS9930_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
	//LINE <Jira ID (KeyCode)> <DATE20140625> <BUG INFO> limi.zhang
	#if 0
	obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
	#endif									//(400)/16*2.72 here is amplify *100 //16
	//LINE <Jira ID (KeyCode)> <DATE20140625> <BUG INFO> limi.zhang								
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	obj->ps_cali = 0;
	
	APDS9930_i2c_client = client;
	
	if(1 == obj->hw->polling_mode_ps)
		//if (1)
		{
			obj_ps.polling = 1;
		}
		else
		{
			obj_ps.polling = 0;
		}
	
	if((err = APDS9930_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("APDS9930_init_client() OK!\n");

	if((err = misc_register(&APDS9930_device)))
	{
		APS_ERR("APDS9930_device register failed\n");
		goto exit_misc_device_register_failed;
	}

    if(err = APDS9930_create_attr(&APDS9930_alsps_driver.driver))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

	obj_ps.self = APDS9930_obj;
	
	obj_ps.sensor_operate = APDS9930_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = APDS9930_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = APDS9930_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = APDS9930_early_suspend,
	obj->early_drv.resume   = APDS9930_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&APDS9930_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	//exit_kfree:
	kfree(obj);
	exit:
	APDS9930_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_i2c_remove(struct i2c_client *client)
{
	int err;	

    if(err = APDS9930_delete_attr(&APDS9930_alsps_driver.driver))
    {
        APS_ERR("APDS9930_delete_attr fail: %d\n", err);
    }

	if((err = misc_deregister(&APDS9930_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	APDS9930_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();

	APDS9930_power(hw, 1);    
	//APDS9930_force[0] = hw->i2c_num;
	//APDS9930_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",APDS9930_force[0],APDS9930_force[1]);
	if(i2c_add_driver(&APDS9930_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int APDS9930_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	APDS9930_power(hw, 0);    
	i2c_del_driver(&APDS9930_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver APDS9930_alsps_driver = {
	.probe      = APDS9930_probe,
	.remove     = APDS9930_remove,    
	.driver     = {
		.name  = "als_ps",
//		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init APDS9930_init(void)
{
	//APS_FUN();
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_APDS9930, 1);
	if(platform_driver_register(&APDS9930_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit APDS9930_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&APDS9930_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(APDS9930_init);
module_exit(APDS9930_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("APDS9930 driver");
MODULE_LICENSE("GPL");

