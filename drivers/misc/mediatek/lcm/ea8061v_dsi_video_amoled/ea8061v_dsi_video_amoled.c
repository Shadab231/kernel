/*[JSTINNO_SRC xiaoyan.yu,  add for supporting AMOLED, DATE20140903-01 START*/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#define LCM_PRINT printf
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#define LCM_PRINT printk
#endif



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("[ea8061v_dsi_vdo_lcm_drv_amoled] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             		0XFE
#define REGFLAG_END_OF_TABLE      	0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define   TRUE     1
#endif

#ifndef FALSE
    #define   FALSE    0
#endif

//#define HS_INIT
#ifdef HS_INIT
//extern void dsi_enter_hs(bool enter);  // Modified by zhangxian
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
#define GPIO_LCM_RST GPIO112
static LCM_UTIL_FUNCS lcm_util = {0};
#define GPIO_LCM_3V3_EN GPIO45
#define GPIO_LCM_1V8_EN GPIO119
#define SET_RESET_PIN(v)   	\
	mt_set_gpio_mode(GPIO_LCM_RST,GPIO_MODE_00);	\
	mt_set_gpio_dir(GPIO_LCM_RST,GPIO_DIR_OUT);		\
	if(v)											\
		mt_set_gpio_out(GPIO_LCM_RST,GPIO_OUT_ONE);	\
	else											\
		mt_set_gpio_out(GPIO_LCM_RST,GPIO_OUT_ZERO);

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)       
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};



// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

        memset(params, 0, sizeof(LCM_PARAMS));

        params->type   = LCM_TYPE_DSI;

        params->width  = FRAME_WIDTH;
        params->height = FRAME_HEIGHT;

        // enable tearing-free
        //params->dbi.te_mode 		= LCM_DBI_TE_MODE_VSYNC_ONLY;
        //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
        params->dsi.mode   = CMD_MODE;
#else
        params->dsi.mode   =BURST_VDO_MODE;// SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;
#endif

        // DSI
        /* Command mode setting */
        //1 Three lane or Four lane
        params->dsi.LANE_NUM				= LCM_FOUR_LANE;
        //The following defined the fomat for data coming from LCD engine.
        params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
        params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
        params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
        params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

        // Highly depends on LCD driver capability.
        // Not support in MT6573
        params->dsi.packet_size=256;

        // Video mode setting		
        params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

        params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
        params->dsi.word_count=720*3;	

        params->dsi.vertical_sync_active				= 2;//2;	//2
        params->dsi.vertical_backporch				= 8;//60;//14;	//7
        params->dsi.vertical_frontporch				= 14;//16;	//14
        params->dsi.vertical_active_line				= FRAME_HEIGHT; 

        params->dsi.horizontal_sync_active			= 30;//4;	//2;
        params->dsi.horizontal_backporch				= 40;//42;	//60;	//42; 200
        params->dsi.horizontal_frontporch				= 55;//100;//44;	//60;	//44; 200
        params->dsi.horizontal_active_pixel			= FRAME_WIDTH;

	 params->dsi.cont_clock = 1 ; //ADD BY PETER
	 params->dsi.ssc_disable = 1; // ADD BY PETER
	
        params->dsi.PLL_CLOCK = 212; //215; //	

}
static void lcm_setbacklight(unsigned int level)                                                                             
{
	unsigned char mapped_level = 0;
	//unsigned int data_array[16];
	
	LCM_DBG("level =%d",level);
	//for LGE backlight IC mapping table
	if(level > 255) 
		mapped_level = 255;
	if(level > 0) 
		mapped_level = (unsigned char)level;
	else
		mapped_level=0;

	//data_array[0] = (0x00511500 | (mapped_level<<24));		// Display On
	//dsi_set_cmdq(&data_array, 1, 1);
	dsi_set_cmdq_V2(0x51, 1, &mapped_level, 1); 	///enable TE
		
}


static void lcm_change_backligth_to_dim_mode(void)
{
 //  unsigned char dim_mode = 0x28;
 //  LCM_DBG("lcm_change_backligth_to_dim_mode");

  // dsi_set_cmdq_V2(0x53, 1, &dim_mode, 1); 
}

static void init_lcm_registers(void) 
{
        unsigned int data_arry[10];

        LCM_DBG("zx:init_lcm_registers");	   

        // {0x39, 0xF0,  2 ,{0x5A,0x5A}},	//Level2 Key Unlock
        data_arry[0] = 0x00033902;
        data_arry[1] = 0x005a5af0;
        dsi_set_cmdq(&data_arry,2,1);

        //{0x39, 0xB8,  2 ,{0x00,0x10}},	//MPSCTL
        data_arry[0] = 0x00033902;
        data_arry[1] = 0x001000b8;
        dsi_set_cmdq(&data_arry,2,1);

        //{0x15, 0xBA,	1 ,{0x56}}, 	  //SOURCE_CTL
        data_arry[0] = 0x56ba1500;
        dsi_set_cmdq(&data_arry,1,1);

        // {0x39, 0xF0,  2 ,{0xa5,0xa5}},	//Level2 Key  lock
        data_arry[0] = 0x00033902;
        data_arry[1] = 0x00a5a5f0;
        dsi_set_cmdq(&data_arry,2,1);

        /* dimming mode (Brightness control) */

        //{0x15, 0x53,  1 ,{0x20}},		//WRDISBV
        //data_arry[0] = 0x28531500;    // on dim must
        data_arry[0] = 0x20531500;   // off dim
        dsi_set_cmdq(&data_arry,1,1);


        data_arry[0] = 0x00511500;
        dsi_set_cmdq(&data_arry,1,1);
	 
#ifdef BUILD_LK
        data_arry[0] = 0xFF511500;
        dsi_set_cmdq(&data_arry,1,1);
#endif

        data_arry[0] = 0x00110500;
        dsi_set_cmdq(&data_arry,1,1);
        MDELAY(130);

        data_arry[0] = 0x00290500; // Sleep Out
        dsi_set_cmdq(data_arry, 1, 1);
       
}

static void lcm_init_power(void)
{
        mt_set_gpio_mode(GPIO_LCM_1V8_EN | 0x80000000,GPIO_MODE_00);	  
        mt_set_gpio_dir(GPIO_LCM_1V8_EN | 0x80000000,GPIO_DIR_OUT);	  
        mt_set_gpio_out(GPIO_LCM_1V8_EN | 0x80000000,GPIO_OUT_ZERO);

        mt_set_gpio_mode(GPIO_LCM_3V3_EN | 0x80000000,GPIO_MODE_00);	  
        mt_set_gpio_dir(GPIO_LCM_3V3_EN | 0x80000000,GPIO_DIR_OUT);	  
        mt_set_gpio_out(GPIO_LCM_3V3_EN | 0x80000000,GPIO_OUT_ZERO);
        MDELAY(15); // min 1
        mt_set_gpio_out(GPIO_LCM_1V8_EN | 0x80000000,GPIO_OUT_ONE);
        mt_set_gpio_out(GPIO_LCM_3V3_EN | 0x80000000,GPIO_OUT_ONE);
#if 0
#if defined(TINNO_ANDROID_L5560AE)
     #ifdef BUILD_LK
     //dprintf(0, "vgp3 on\n");
     mt6325_upmu_set_rg_vgp3_vosel(3);
     mt6325_upmu_set_rg_vgp3_en(0);
     #else
     printk("vgp3 on\n");
     hwPowerDown(MT6325_POWER_LDO_VGP3, "LCD");
     #endif
     //MDELAY(5); // min 1
     mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
     mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
     mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);

     #ifdef BUILD_LK
     dprintf(0, "vgp3 on\n");
     mt6325_upmu_set_rg_vgp3_vosel(3);
     mt6325_upmu_set_rg_vgp3_en(1);
     #else
     printk("vgp3 on\n");
     hwPowerOn(MT6325_POWER_LDO_VGP3, VOL_1800, "LCD");
     #endif
     //MDELAY(5); // min 1
     mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
     mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
     mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
	 
     MDELAY(15); // min 1
#elif defined(TINNO_ANDROID_L8420AE)
   mt_set_gpio_mode(GPIO118 | 0x80000000,GPIO_MODE_00);	  
   mt_set_gpio_dir(GPIO118 | 0x80000000,GPIO_DIR_OUT);	  
   mt_set_gpio_out(GPIO118 | 0x80000000,GPIO_OUT_ZERO);

   mt_set_gpio_mode(GPIO114 | 0x80000000,GPIO_MODE_00);	  
   mt_set_gpio_dir(GPIO114 | 0x80000000,GPIO_DIR_OUT);	  
   mt_set_gpio_out(GPIO114 | 0x80000000,GPIO_OUT_ZERO);

   mt_set_gpio_mode(GPIO118 | 0x80000000,GPIO_MODE_00);	  
   mt_set_gpio_dir(GPIO118 | 0x80000000,GPIO_DIR_OUT);	  
   mt_set_gpio_out(GPIO118 | 0x80000000,GPIO_OUT_ONE);

   mt_set_gpio_mode(GPIO114 | 0x80000000,GPIO_MODE_00);	  
   mt_set_gpio_dir(GPIO114 | 0x80000000,GPIO_DIR_OUT);	  
   mt_set_gpio_out(GPIO114 | 0x80000000,GPIO_OUT_ONE);

   MDELAY(15); // min 1

#endif
#endif //#if 0
}

static void lcm_init(void)
{

        LCM_DBG("zx:lcm_init");
        //SET_RESET_PIN(1);
        SET_RESET_PIN(0);
        MDELAY(5);
        SET_RESET_PIN(1);
        MDELAY(25);
        lcm_init_power();
        init_lcm_registers();
}

static void lcm_suspend(void)
{
        LCM_DBG("lcm_suspend ----power off");
#ifdef HS_INIT
        //      dsi_enter_hs(0);
#endif
        SET_RESET_PIN(0);
        MDELAY(10);
        mt_set_gpio_out(GPIO_LCM_1V8_EN | 0x80000000,GPIO_OUT_ZERO);
        mt_set_gpio_out(GPIO_LCM_3V3_EN | 0x80000000,GPIO_OUT_ZERO);  
}


static void lcm_resume(void)
{
        LCM_DBG("zx:lcm_resume");
        lcm_init_power();
        lcm_init();
}



// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static unsigned int lcm_esd_check(void)
{
	unsigned int ret=FALSE;
#ifndef BUILD_LK	
	char  buffer[6];
	int   array[4];
	char esd1;

	array[0] = 0x00013708;
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0x0A, buffer, 1);
	esd1=buffer[0];

	if(esd1==0x9c)
	{
		ret=FALSE;
	}
	else
	{			 
		ret=TRUE;
	}
	LCM_DBG("lcm_esd_check esd1=%x",esd1);

#endif

	return ret;
}


static unsigned int lcm_esd_recover(void)
{
    LCM_DBG("lcm_esd_recover");
    lcm_init();
    return TRUE;
}



#ifndef BUILD_LK
//#define ESD_DEBUG
#endif

static lcm_goto_suspend(void)
{
      unsigned char para;
      
      LCM_DBG("lcm_goto_suspend");
      
      //mipi_1data(0x28,0X00); 
      dsi_set_cmdq_V2(0x28, 0, &para, 1); 	
      MDELAY(40);
      
      //mipi_1data(0x10,0X00); 
      dsi_set_cmdq_V2(0x10, 0, &para, 1);	   
      MDELAY(130);
}



LCM_DRIVER ea8061v_dsi_vdo_lcm_drv_amoled= 
{
    .name           = "ea8061v_dsi_vdo_lcm_drv_amoled",
    .set_util_funcs = lcm_set_util_funcs,
    .set_backlight	= lcm_setbacklight,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
   // .esd_check   = lcm_esd_check,
  //  .esd_recover   = lcm_esd_recover,
#ifndef BUILD_LK
    .goto_suspend = lcm_goto_suspend,
    .change_backligth_to_dim_mode = lcm_change_backligth_to_dim_mode,
#endif

};

/*JSTINNO_SRC xiaoyan.yu, DATE20140903-01 END]*/

