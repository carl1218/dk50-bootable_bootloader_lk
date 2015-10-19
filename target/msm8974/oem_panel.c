/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <debug.h>
#include <err.h>
#include <smem.h>
#include <msm_panel.h>
#include <board.h>
#include <mipi_dsi.h>

#include <i2c_qup.h>
#include <blsp_qup.h>
#include <pm8x41.h>
#include <platform/gpio.h>

#include "include/panel.h"
#include "panel_display.h"

/*---------------------------------------------------------------------------*/
/* GCDB Panel Database                                                       */
/*---------------------------------------------------------------------------*/
#include "include/panel_toshiba_720p_video.h"
#include "include/panel_sharp_qhd_video.h"
#include "include/panel_jdi_1080p_video.h"
#include "include/panel_generic_720p_cmd.h"
#include "include/panel_jdi_qhd_dualdsi_video.h"
#include "include/panel_jdi_qhd_dualdsi_cmd.h"
#include "include/panel_hx8379a_wvga_video.h"
#include "include/panel_hx8379c_wvga_video.h"

#define DISPLAY_MAX_PANEL_DETECTION 8

/*---------------------------------------------------------------------------*/
/* static panel selection variable                                           */
/*---------------------------------------------------------------------------*/
enum {
JDI_1080P_VIDEO_PANEL,
TOSHIBA_720P_VIDEO_PANEL,
SHARP_QHD_VIDEO_PANEL,
GENERIC_720P_CMD_PANEL,
JDI_QHD_DUALDSI_VIDEO_PANEL,
JDI_QHD_DUALDSI_CMD_PANEL,
HX8379A_WVGA_VIDEO_PANEL,
HX8379C_WVGA_VIDEO_PANEL,
UNKNOWN_PANEL
};

#define DISPLAY_POWER_UP_PIN (13)

static void dsi2lvds_bridge_init();
static int display_power_up(uint8_t status);
static void per_display_init(int blsp_id, int qup_id,
    unsigned char led_driver_addr, int hx7816_addr, int hx7316_addr);

static uint32_t panel_id;

int oem_panel_rotation()
{
	/* OEM can keep there panel spefic on instructions in this
	function */
	return NO_ERROR;
}


int oem_panel_on()
{
	/* OEM can keep there panel spefic on instructions in this
	function */
	return NO_ERROR;
}

int oem_panel_off()
{
	/* OEM can keep there panel spefic off instructions in this
	function */
	return NO_ERROR;
}

static void init_panel_data(struct panel_struct *panelstruct,
			struct msm_panel_info *pinfo,
			struct mdss_dsi_phy_ctrl *phy_db)
{
	switch (panel_id) {
	case TOSHIBA_720P_VIDEO_PANEL:
		panelstruct->paneldata    = &toshiba_720p_video_panel_data;
		panelstruct->panelres     = &toshiba_720p_video_panel_res;
		panelstruct->color        = &toshiba_720p_video_color;
		panelstruct->videopanel   = &toshiba_720p_video_video_panel;
		panelstruct->commandpanel = &toshiba_720p_video_command_panel;
		panelstruct->state        = &toshiba_720p_video_state;
		panelstruct->laneconfig   = &toshiba_720p_video_lane_config;
		panelstruct->paneltiminginfo
					 = &toshiba_720p_video_timing_info;
		panelstruct->panelresetseq
					 = &toshiba_720p_video_panel_reset_seq;
		panelstruct->backlightinfo = &toshiba_720p_video_backlight;
		pinfo->mipi.panel_cmds
					= toshiba_720p_video_on_command;
		pinfo->mipi.num_of_panel_cmds
					= TOSHIBA_720P_VIDEO_ON_COMMAND;
		memcpy(phy_db->timing,
			toshiba_720p_video_timings, TIMING_SIZE);
		pinfo->mipi.signature 	= TOSHIBA_720P_VIDEO_SIGNATURE;
		break;
	case SHARP_QHD_VIDEO_PANEL:
	
		panelstruct->paneldata    = &sharp_qhd_video_panel_data;
		panelstruct->panelres     = &sharp_qhd_video_panel_res;
		panelstruct->color        = &sharp_qhd_video_color;
		panelstruct->videopanel   = &sharp_qhd_video_video_panel;
		panelstruct->commandpanel = &sharp_qhd_video_command_panel;
		panelstruct->state        = &sharp_qhd_video_state;
		panelstruct->laneconfig   = &sharp_qhd_video_lane_config;
		panelstruct->paneltiminginfo
					 = &sharp_qhd_video_timing_info;
		panelstruct->panelresetseq
					 = &sharp_qhd_video_panel_reset_seq;
		panelstruct->backlightinfo = &sharp_qhd_video_backlight;
		pinfo->mipi.panel_cmds
					= sharp_qhd_video_on_command;
		pinfo->mipi.num_of_panel_cmds
					= SHARP_QHD_VIDEO_ON_COMMAND;
		memcpy(phy_db->timing,
				sharp_qhd_video_timings, TIMING_SIZE);
		break;
	case JDI_1080P_VIDEO_PANEL:
		panelstruct->paneldata    = &jdi_1080p_video_panel_data;
		panelstruct->panelres     = &jdi_1080p_video_panel_res;
		panelstruct->color        = &jdi_1080p_video_color;
		panelstruct->videopanel   = &jdi_1080p_video_video_panel;
		panelstruct->commandpanel = &jdi_1080p_video_command_panel;
		panelstruct->state        = &jdi_1080p_video_state;
		panelstruct->laneconfig   = &jdi_1080p_video_lane_config;
		panelstruct->paneltiminginfo
			= &jdi_1080p_video_timing_info;
		panelstruct->panelresetseq
					 = &jdi_1080p_video_panel_reset_seq;
		panelstruct->backlightinfo = &jdi_1080p_video_backlight;
		pinfo->mipi.panel_cmds
			= jdi_1080p_video_on_command;
		pinfo->mipi.num_of_panel_cmds
			= JDI_1080P_VIDEO_ON_COMMAND;
		memcpy(phy_db->timing,
			jdi_1080p_video_timings, TIMING_SIZE);
		pinfo->mipi.signature = JDI_1080P_VIDEO_SIGNATURE;
		break;
	case GENERIC_720P_CMD_PANEL:
		panelstruct->paneldata    = &generic_720p_cmd_panel_data;
		panelstruct->panelres     = &generic_720p_cmd_panel_res;
		panelstruct->color        = &generic_720p_cmd_color;
		panelstruct->videopanel   = &generic_720p_cmd_video_panel;
		panelstruct->commandpanel = &generic_720p_cmd_command_panel;
		panelstruct->state        = &generic_720p_cmd_state;
		panelstruct->laneconfig   = &generic_720p_cmd_lane_config;
		panelstruct->paneltiminginfo
			= &generic_720p_cmd_timing_info;
		panelstruct->panelresetseq
					 = &generic_720p_cmd_reset_seq;
		panelstruct->backlightinfo = &generic_720p_cmd_backlight;
		pinfo->mipi.panel_cmds
			= generic_720p_cmd_on_command;
		pinfo->mipi.num_of_panel_cmds
			= GENERIC_720P_CMD_ON_COMMAND;
		memcpy(phy_db->timing,
			generic_720p_cmd_timings, TIMING_SIZE);
		pinfo->mipi.signature = GENERIC_720P_CMD_SIGNATURE;
		break;
	case JDI_QHD_DUALDSI_VIDEO_PANEL:
		panelstruct->paneldata    = &jdi_qhd_dualdsi_video_panel_data;
		panelstruct->panelres     = &jdi_qhd_dualdsi_video_panel_res;
		panelstruct->color        = &jdi_qhd_dualdsi_video_color;
		panelstruct->videopanel   = &jdi_qhd_dualdsi_video_video_panel;
		panelstruct->commandpanel = &jdi_qhd_dualdsi_video_command_panel;
		panelstruct->state        = &jdi_qhd_dualdsi_video_state;
		panelstruct->laneconfig   = &jdi_qhd_dualdsi_video_lane_config;
		panelstruct->paneltiminginfo
			= &jdi_qhd_dualdsi_video_timing_info;
		panelstruct->panelresetseq
					 = &jdi_qhd_dualdsi_video_reset_seq;
		panelstruct->backlightinfo = &jdi_qhd_dualdsi_video_backlight;
		pinfo->mipi.panel_cmds
			= jdi_qhd_dualdsi_video_on_command;
		pinfo->mipi.num_of_panel_cmds
			= JDI_QHD_DUALDSI_VIDEO_ON_COMMAND;
		memcpy(phy_db->timing,
			jdi_qhd_dualdsi_video_timings, TIMING_SIZE);
		break;
	case JDI_QHD_DUALDSI_CMD_PANEL:
		panelstruct->paneldata    = &jdi_qhd_dualdsi_cmd_panel_data;
		panelstruct->panelres     = &jdi_qhd_dualdsi_cmd_panel_res;
		panelstruct->color        = &jdi_qhd_dualdsi_cmd_color;
		panelstruct->videopanel   = &jdi_qhd_dualdsi_cmd_video_panel;
		panelstruct->commandpanel = &jdi_qhd_dualdsi_cmd_command_panel;
		panelstruct->state        = &jdi_qhd_dualdsi_cmd_state;
		panelstruct->laneconfig   = &jdi_qhd_dualdsi_cmd_lane_config;
		panelstruct->paneltiminginfo
			= &jdi_qhd_dualdsi_cmd_timing_info;
		panelstruct->panelresetseq
					 = &jdi_qhd_dualdsi_cmd_reset_seq;
		panelstruct->backlightinfo = &jdi_qhd_dualdsi_cmd_backlight;
		pinfo->mipi.panel_cmds
			= jdi_qhd_dualdsi_cmd_on_command;
		pinfo->mipi.num_of_panel_cmds
			= JDI_QHD_DUALDSI_CMD_ON_COMMAND;
		memcpy(phy_db->timing,
			jdi_qhd_dualdsi_cmd_timings, TIMING_SIZE);
		break;
	case HX8379A_WVGA_VIDEO_PANEL:
		panelstruct->paneldata        = &hx8379a_wvga_video_panel_data;
		panelstruct->panelres         = &hx8379a_wvga_video_panel_res;
		panelstruct->color            = &hx8379a_wvga_video_color;
		panelstruct->videopanel       = &hx8379a_wvga_video_video_panel;
		panelstruct->commandpanel     = &hx8379a_wvga_video_command_panel;
		panelstruct->state            = &hx8379a_wvga_video_state;
		panelstruct->laneconfig       = &hx8379a_wvga_video_lane_config;
		panelstruct->paneltiminginfo  = &hx8379a_wvga_video_timing_info;
		panelstruct->panelresetseq    = &hx8379a_wvga_video_reset_seq;
		panelstruct->backlightinfo    = &hx8379a_wvga_video_backlight;
		pinfo->mipi.panel_cmds	      = hx8379a_wvga_video_on_command;
		pinfo->mipi.num_of_panel_cmds = HX8379A_WVGA_VIDEO_ON_COMMAND;
		memcpy(phy_db->timing, hx8379a_wvga_video_timings, TIMING_SIZE);
		dprintf(CRITICAL, "Display Init for HX8379A_WVGA_VIDEO_PANEL\n");
		break;

	case HX8379C_WVGA_VIDEO_PANEL:
		panelstruct->paneldata        = &hx8379c_wvga_video_panel_data;
		panelstruct->panelres         = &hx8379c_wvga_video_panel_res;
		panelstruct->color            = &hx8379c_wvga_video_color;
		panelstruct->videopanel       = &hx8379c_wvga_video_video_panel;
		panelstruct->commandpanel     = &hx8379c_wvga_video_command_panel;
		panelstruct->state            = &hx8379c_wvga_video_state;
		panelstruct->laneconfig       = &hx8379c_wvga_video_lane_config;
		panelstruct->paneltiminginfo  = &hx8379c_wvga_video_timing_info;
		panelstruct->panelresetseq    = &hx8379c_wvga_video_reset_seq;
		panelstruct->backlightinfo    = &hx8379c_wvga_video_backlight;
		pinfo->mipi.panel_cmds	      = hx8379c_wvga_video_on_command;
		pinfo->mipi.num_of_panel_cmds = HX8379C_WVGA_VIDEO_ON_COMMAND;
		
		memcpy(phy_db->timing, hx8379c_wvga_video_timings, TIMING_SIZE);
		dprintf(CRITICAL, "Display Init for HX8379C_WVGA_VIDEO_PANEL\n");
		break;
		
	case UNKNOWN_PANEL:
		memset(panelstruct, 0, sizeof(struct panel_struct));
		memset(pinfo->mipi.panel_cmds, 0, sizeof(struct mipi_dsi_cmd));
		pinfo->mipi.num_of_panel_cmds = 0;
		memset(phy_db->timing, 0, TIMING_SIZE);
		pinfo->mipi.signature = 0;
		break;
	}
}

uint32_t oem_panel_max_auto_detect_panels()
{
	return target_panel_auto_detect_enabled() ?
			DISPLAY_MAX_PANEL_DETECTION : 0;
}

static uint32_t auto_pan_loop = 0;

bool oem_panel_select(struct panel_struct *panelstruct,
			struct msm_panel_info *pinfo,
			struct mdss_dsi_phy_ctrl *phy_db)
{
	uint32_t hw_id = board_hardware_id();
	uint32_t target_id = board_target_id();
	bool ret = true;

	switch (hw_id) {
	case HW_PLATFORM_MTP:
	case HW_PLATFORM_FLUID:
	case HW_PLATFORM_SURF:
		switch (auto_pan_loop) {
		case 0:
			panel_id = JDI_1080P_VIDEO_PANEL;
			break;
		case 1:
			panel_id = TOSHIBA_720P_VIDEO_PANEL;
			break;
		case 2:
			panel_id = GENERIC_720P_CMD_PANEL;
			break;
		default:
			panel_id = UNKNOWN_PANEL;
			ret = false;
			break;
		}
		auto_pan_loop++;
		break;
	case HW_PLATFORM_DRAGON:
		panel_id = SHARP_QHD_VIDEO_PANEL;
		break;
    case HW_PLATFORM_VARSOM:
        dsi2lvds_bridge_init();
        display_power_up(1);
        thread_sleep(50);
        /* display 1 */
        per_display_init(BLSP_ID_1, QUP_ID_2, 0x28, 0x3c, 0x49);
        /* display 2 */
        per_display_init(BLSP_ID_1, QUP_ID_0, 0x28, 0x3c, 0x49);
        panel_id = SHARP_QHD_VIDEO_PANEL;
        break;
	default:
		dprintf(CRITICAL, "Display not enabled for %d HW type\n"
					, hw_id);
		return false;
	}

	init_panel_data(panelstruct, pinfo, phy_db);

	return ret;
}

void dsi2lvds_bridge_init(void)
{
	unsigned char addresses[]={	0x09, 0x0A, 0x0B, 0x0D, 0x10, 
					0x11, 0x12, 0x13, 0x18, 0x19, 
					0x1A, 0x1B, 0x20, 0x21, 0x22,
					0x23, 0x24, 0x25, 0x26, 0x27,
					0x28, 0x29, 0x2A, 0x2B, 0x2C,
					0x2D, 0x2E, 0x2F, 0x30, 0x31,
					0x32, 0x33, 0x34, 0x35, 0x36,
					0x37, 0x38, 0x39, 0x3A, 0x3B,
					0x3C, 0x3D, 0x3E, 0x0D};
#ifdef DSI2LVDS_BRIDGE_CONFIG_6BIT
	unsigned char datas[]={		0x01, 0x01, 0x58, 0x00, 0x5e,
					0x00, 0x46, 0x46, 0x6F, 0x00,
					0x03, 0x00, 0x56, 0x05, 0x56,
					0x05, 0x00, 0x03, 0x00, 0x03,
					0x21, 0x00, 0x21, 0x00, 0x30,
					0x00, 0x30, 0x00, 0x03, 0x00,
					0x03, 0x00, 0x28, 0x28, 0x1d,
					0x1d, 0x28, 0x28, 0x0d, 0x0d,
					0x00, 0x00, 0x00, 0x01};
#else /* DSI2LVDS_BRIDGE_CONFIG_6BIT */
	unsigned char datas[]={		0x01, 0x01, 0x58, 0x00, 0x5e,
					0x00, 0x46, 0x46, 0x6C, 0x00,
					0x03, 0x00, 0x56, 0x05, 0x56,
					0x05, 0x00, 0x03, 0x00, 0x03,
					0x21, 0x00, 0x21, 0x00, 0x30,
					0x00, 0x30, 0x00, 0x03, 0x00,
					0x03, 0x00, 0x28, 0x28, 0x1d,
					0x1d, 0x28, 0x28, 0x0d, 0x0d,
					0x00, 0x00, 0x00, 0x01};
#endif /* DSI2LVDS_BRIDGE_CONFIG_6BIT */

	struct qup_i2c_dev *dev;
	char buf[2];
	struct i2c_msg msg;
	int ret,i;
	int soc_ver = board_soc_version(); //Get the CHIP version

	gpio_tlmm_config(85, /*func*/0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA, 1);

	gpio_set(85, 2); /* Clear LVDS bridge reset */
	thread_sleep(10);
	/*
	1 arg: BLSP ID can be BLSP_ID_1 or BLSP_ID_2
	2 arg: QUP ID can be QUP_ID_0:QUP_ID_5
	3 arg: I2C CLK. should be 100KHZ, or 400KHz
	4 arg: Source clock, should be set @ 19.2 MHz for V1 and 50MHz for V2
	or Higher Rev
	*/
	if( soc_ver >= BOARD_SOC_VERSION2 )
	{
		dev = qup_blsp_i2c_init(BLSP_ID_2, QUP_ID_1, 100000, 50000000);
	}
	else
	{
		dev = qup_blsp_i2c_init(BLSP_ID_2, QUP_ID_1, 100000, 19200000);
	}
	if(!dev)
	{
		printf("DSI2LVDS Bridge failed to initialize\n");
		return;
	}

	//Test Transfer
	msg.addr = 0x2c;
	msg.flags = I2C_M_WR;
	msg.len = 2;
	msg.buf = buf;
	for(i=0;i<sizeof(addresses);i++)
	{
		msg.buf[0]=addresses[i];
		msg.buf[1]=datas[i];
		ret = qup_i2c_xfer(dev, &msg, 1);
	}
	qup_i2c_deinit(dev);
}

static int display_power_up(uint8_t enable)
{
    uint8_t status;
    struct pm8x41_gpio gpio = {
        .direction = PM_GPIO_DIR_OUT,
        .output_buffer = PM_GPIO_OUT_CMOS,
        .out_strength = PM_GPIO_OUT_DRIVE_MED,
    };

/* Configure the GPIO 
        gpio.direction = PM_GPIO_DIR_OUT;
        gpio.function  = PM_GPIO_FUNC_2;
        gpio.pull      = PM_GPIO_PULL_UP_30;
        gpio.vin_sel   = 2;
        gpio.output_buffer = PM_GPIO_OUT_CMOS,
        gpio.out_strength = PM_GPIO_OUT_DRIVE_HIGH;
*/
        pm8x41_gpio_config(DISPLAY_POWER_UP_PIN, &gpio);
        /* Wait for the pmic gpio config to take effect */
        mdelay(1);
        if (enable) {
            pm8x41_gpio_set(DISPLAY_POWER_UP_PIN, PM_GPIO_FUNC_HIGH);
        } else {
            gpio.out_strength = PM_GPIO_OUT_DRIVE_LOW;
            pm8x41_gpio_config(DISPLAY_POWER_UP_PIN, &gpio);
            pm8x41_gpio_set(DISPLAY_POWER_UP_PIN, PM_GPIO_FUNC_LOW);
        }
        mdelay(50);
        pm8x41_gpio_get(DISPLAY_POWER_UP_PIN, &status);
        dprintf(CRITICAL, "%s:: gpio %d state: %d (set to %d)\n",
            __FUNCTION__, DISPLAY_POWER_UP_PIN, status, enable);

        return status; /* active high */
}

// lcos module

static const char hx7318_init_first_stage_reg_sequence_addr[] = {
    0x00, 0x00, 0x13, 0x14, 0x17, 0x5D, 0x5E, 0x5F,
    0x61, 0x82, 0x19, 0x1A, 0x21, 0x22, 0x23, 0x24,
    0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C,
    0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34,
    0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C,
    0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C,
    0x4D, 0x4E, 0x4F, 0x50
};

static const char hx7318_init_first_stage_reg_sequence_data[] = {
    0x00, 0xE1, 0x45, 0x80, 0x08, 0x9E, 0x99, 0x3E,
    0x28, 0xEB, 0x08, 0xDD, 0x03, 0x03, 0x03, 0xFC,
    0xFC, 0xFC, 0x5F, 0x5F, 0x5F, 0xA0, 0xA0, 0xA0,
    0x73, 0x73, 0x73, 0x8C, 0x8C, 0x8C, 0x83, 0x83,
    0x83, 0x7C, 0x7C, 0x7C, 0x8B, 0x8B, 0x8B, 0x74,
    0x74, 0x74, 0x97, 0x97, 0x97, 0x68, 0x68, 0x68,
    0xA3, 0xA3, 0xA3, 0x5C, 0x5C, 0x5C, 0xDF, 0xDF,
    0xDF, 0x20, 0x20, 0x20
};

static const char hx7318_init_second_stage_reg_sequence_addr[] = {
    0x00, 0x00
};

static const char hx7318_init_second_stage_reg_sequence_data[] = {
    0xF1, 0xE1
};

static const char hx7816_init_reg_sequence_addr[] = {
    0xFE, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
    0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x30, 0x3A,
    0x3B, 0x3C, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D,
    0x4E, 0x4F, 0x54, 0x66, 0x67, 0x68, 0x69, 0x6A,
    0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x70, 0x73,
    0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7C,
    0x7D, 0x7C, 0x7D, 0x7C, 0x7D, 0x7C, 0x7D, 0x7C,
    0x7D, 0x7C, 0x7D, 0x7C, 0x7D, 0x7B, 0x7C, 0x7D,
    0x7C, 0x7D, 0x7C, 0x7D, 0x7C, 0x7D, 0x7C, 0x7D,
    0x7B, 0x7C, 0x7D, 0x7C, 0x7D, 0x7E, 0x90, 0xA0,
    0xA2, 0xA3, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5,
    0xC4, 0xC5, 0xC4, 0xC5, 0xC4, 0xC5, 0xC4, 0xC5,
    0xC4, 0xC5, 0xC4, 0xC5, 0xC4, 0xC5, 0xC4, 0xC5,
    0xC4, 0xC5, 0xC4, 0xC5, 0xC4, 0xC5, 0xC4, 0xC5,
    0xC4, 0xC5, 0xC4, 0xC5, 0xC4, 0xC5, 0xC9, 0xCC,
    0xCD, 0xCD, 0xCD, 0xCD, 0xCD, 0xCD, 0xCD, 0xCD,
    0xCD, 0xCD, 0xCD, 0xCD, 0xCD, 0xCD, 0xCD, 0xCD,
    0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7,
    0xD8, 0xD9, 0xDA, 0xDC, 0xDD, 0xDE, 0xED, 0xEE,
    0xEF, 0xF1, 0xF2, 0xF3, 0x98, 0xF7, 0xF0
};

static const char hx7816_init_reg_sequence_data[] = {
    0x33, 0x07, 0x58, 0x00, 0x40, 0x06, 0x56, 0x05,
    0x1D, 0x03, 0x34, 0x03, 0x00, 0x03, 0x67, 0xFF,
    0xFF, 0x7F, 0x7A, 0x06, 0x10, 0x00, 0x54, 0x00,
    0x56, 0x05, 0x34, 0x03, 0x03, 0x00, 0x1C, 0x00,
    0x00, 0x03, 0x03, 0x00, 0xFF, 0x0F, 0x00, 0x10,
    0x01, 0xFF, 0x0F, 0x00, 0x10, 0x30, 0x01, 0x03,
    0x08, 0x50, 0x5A, 0x00, 0x00, 0x40, 0x03, 0x89,
    0x03, 0x89, 0x03, 0x40, 0x03, 0x40, 0x03, 0x89,
    0x03, 0x89, 0x03, 0x40, 0x03, 0x0A, 0xB8, 0x06,
    0x54, 0x00, 0x60, 0x05, 0x30, 0x00, 0x08, 0x03,
    0x0F, 0x05, 0x05, 0x04, 0x04, 0x10, 0x11, 0x03,
    0x1B, 0x00, 0x50, 0x5A, 0x00, 0x00, 0x22, 0x04,
    0x50, 0x0A, 0x22, 0x04, 0x50, 0x0A, 0x22, 0x0B,
    0xD0, 0x10, 0x22, 0x0B, 0xD0, 0x10, 0xB4, 0x11,
    0xE2, 0x17, 0xB4, 0x11, 0xE2, 0x17, 0xB2, 0x18,
    0x3E, 0x03, 0xB2, 0x18, 0x3E, 0x03, 0x44, 0x00,
    0x04, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
    0x03, 0x05, 0x07, 0x09, 0x0B, 0x00, 0x00, 0x00,
    0x02, 0x0A, 0x02, 0x00, 0x10, 0x06, 0x00, 0x00,
    0x90, 0x05, 0x02, 0xAB, 0x02, 0x05, 0x01, 0x01,
    0x01, 0x0A, 0x08, 0x01, 0x43, 0x01, 0x11
};

static int hx7318_init_first_stage(struct qup_i2c_dev* dev,
    unsigned char addr) {
    int ret = 0;
	struct i2c_msg msg;
	char buf[2];
	char* addresses = hx7318_init_first_stage_reg_sequence_addr;
	char* datas = hx7318_init_first_stage_reg_sequence_data;
	size_t size = sizeof(hx7318_init_first_stage_reg_sequence_addr);
	int i;

	if (!dev) {
		dprintf(CRITICAL, "%s:: unable to initialize hx7318 (first stage)\n",
		    __FUNCTION__);
		return -1;
	}
	/* set default address if not provided by user */
	if (!addr) {
	    addr = 0x49;
	}

	msg.addr = addr;
	msg.flags = I2C_M_WR;
	msg.len = 2;
	msg.buf = buf;

	for (i = 0; i < size; i++) {
		msg.buf[0]=addresses[i];
		msg.buf[1]=datas[i];
		ret = qup_i2c_xfer(dev, &msg, 1);
	}
	
    return ret;
}

static int hx7318_init_second_stage(struct qup_i2c_dev* dev,
    unsigned char addr) {
    struct i2c_msg msg;
	char buf[2];
    char* addresses = hx7318_init_second_stage_reg_sequence_addr;
	char* datas = hx7318_init_second_stage_reg_sequence_data;
	size_t size = sizeof(hx7318_init_second_stage_reg_sequence_addr);
	int i;
	int ret;

	if (!dev) {
		dprintf(CRITICAL, "%s:: unable to initialize hx7318 (second stage)\n",
		    __FUNCTION__);
		return -1;
	}
	/* set default address if not provided by user */
	if (!addr) {
	    addr = 0x49;
	}

	msg.addr = addr;
	msg.flags = I2C_M_WR;
	msg.len = 2;
	msg.buf = buf;

	for (i = 0; i < size; i++) {
		msg.buf[0]=addresses[i];
		msg.buf[1]=datas[i];
		ret = qup_i2c_xfer(dev, &msg, 1);
	}
	
    return ret;
}

// asic module
static int hx7816_shutdown(struct qup_i2c_dev* dev, unsigned char addr) {
	struct i2c_msg msg;
	char buf[2];
	int ret;

	if (!dev) {
		dprintf(CRITICAL, "%s:: unable to shutdown hx7816\n", __FUNCTION__);
		return -1;
	}
	/* set default address if not provided by user */
	if (!addr) {
	    addr = 0x3C;
	}

	msg.addr = addr;
	msg.flags = I2C_M_WR;
	msg.len = 2;
	msg.buf = buf;

	msg.buf[0] = 0xf0;
	msg.buf[1] = 0x00;

    ret = qup_i2c_xfer(dev, &msg, 1);
    dprintf(CRITICAL, "%s:: hx7816 shutdown done\n", __FUNCTION__);
    return ret;
}

static int hx7816_init(struct qup_i2c_dev* dev, unsigned char addr) {
    int ret = 0;
	struct i2c_msg msg;
	char buf[2];
	char* addresses = hx7816_init_reg_sequence_addr;
	char* datas = hx7816_init_reg_sequence_data;
	size_t size = sizeof(hx7816_init_reg_sequence_addr);
	int i;

	if (!dev) {
		dprintf(CRITICAL, "%s:: unable to initialize hx7816\n", __FUNCTION__);
		return -1;
	}
	/* set default address if not provided by user */
	if (!addr) {
	    addr = 0x3C;
	}

	msg.addr = addr;
	msg.flags = I2C_M_WR;
	msg.len = 2;
	msg.buf = buf;

	for (i=0; i < size; i++) {
		msg.buf[0]=addresses[i];
		msg.buf[1]=datas[i];
		ret = qup_i2c_xfer(dev, &msg, 1);
	}

	dprintf(CRITICAL, "%s:: hx7816 init done\n", __FUNCTION__);
	
    return ret;
}

static int led_driver_init(struct qup_i2c_dev* dev, unsigned char addr,
    unsigned char level) {
    char addresses[] = { 0x01, 0x02, 0x03 };
    int ret = 0;
	struct i2c_msg msg;
	char buf[2];
	int i;

	if (!dev) {
		dprintf(CRITICAL, "%s:: unable to initialize led driver\n", __FUNCTION__);
		return -1;
	}
	/* set default address if not provided by user */
	if (!addr) {
	    addr = 0x28;
	}

    msg.flags = I2C_M_WR;
	msg.addr = addr;
	msg.len = 2;
	msg.buf = buf;

	for (i = 0; i < sizeof(addresses)/sizeof(addresses[0]); i++) {
	    struct i2c_msg v_msg[2];
	    char v_msg_data[2];
	    char retry = 0;
error_retry:
		msg.buf[0] = addresses[i];
		msg.buf[1] = level;
		ret = qup_i2c_xfer(dev, &msg, 1);

		if (ret != 1 && retry) {
		    dprintf(CRITICAL, "no response from led driver\n");
		    break;
		}
	    v_msg[0].addr = msg.addr;
	    v_msg[0].flags = I2C_M_WR;
	    v_msg[0].len = 1;
	    v_msg[0].buf = &v_msg_data[0];
	    v_msg[1].addr = msg.addr;
	    v_msg[1].flags = I2C_M_RD;
	    v_msg[1].len = 1;
   	    v_msg[1].buf = &v_msg_data[1];

   	    v_msg[0].buf[0] = addresses[i];
   	    ret = qup_i2c_xfer(dev, v_msg, 2);
   	    if (v_msg_data[1] != level) {
   	        if (!retry++) {
   	            dprintf(CRITICAL, "%s:: first error on reg %d, retry\n",
   	                __FUNCTION__, addresses[i]);
   	            goto error_retry;
   	        }
   	        dprintf(CRITICAL, "led driver reg %d failed to init\n",
   	            addresses[i]);
   	        break;
   	    }
   	    dprintf(CRITICAL, "led driver init successfully\n");
	}
	
    return 0;
}

// complete sequence per port
static void per_display_init(int blsp_id, int qup_id,
    unsigned char led_driver_addr, int hx7816_addr, int hx7316_addr) {

    struct qup_i2c_dev* dev;
    // open port @ 100Khz (source clock 19.2Mhz)
    dev = qup_blsp_i2c_init(blsp_id, qup_id, 100000, 19200000);
    if (!dev)
	{
		dprintf(CRITICAL, "%s:: i2c failed to init\n");
		return;
	}

    led_driver_init(dev, led_driver_addr, 0x30);
    thread_sleep(2);
    hx7816_shutdown(dev, hx7816_addr);
    thread_sleep(2);
    hx7318_init_first_stage(dev, hx7316_addr);
    thread_sleep(2);
    hx7816_init(dev, hx7816_addr);
    thread_sleep(2);
    hx7318_init_second_stage(dev, hx7316_addr);
    qup_i2c_deinit(dev);

    dprintf(CRITICAL, "%s:: init done for blsp %d qup %d\n",
        __FUNCTION__, blsp_id, qup_id);
}

