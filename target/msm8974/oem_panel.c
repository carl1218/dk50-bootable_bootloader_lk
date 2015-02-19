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

void dsi2lvds_bridge_init();

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
	unsigned char datas[]={		0x01, 0x01, 0x58, 0x00, 0x5e,
					0x00, 0x46, 0x46, 0x6F, 0x00,
					0x03, 0x00, 0x56, 0x05, 0x56,
					0x05, 0x00, 0x03, 0x00, 0x03,
					0x21, 0x00, 0x21, 0x00, 0x30, 
					0x00, 0x30, 0x00, 0x03, 0x00, 
					0x03, 0x00, 0x28, 0x28, 0x1d, 
					0x1d, 0x28, 0x28, 0x0d, 0x0d, 
					0x00, 0x00, 0x00, 0x01};

	struct qup_i2c_dev *dev;
	char buf[2];
	struct i2c_msg msg;
	int ret,i;
	int soc_ver = board_soc_version(); //Get the CHIP version
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
}
