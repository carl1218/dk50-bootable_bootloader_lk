/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <debug.h>
#include <reg.h>
#include <platform/iomap.h>
#include <platform/gpio.h>
#include <gsbi.h>
#include <blsp_qup.h>

void gpio_tlmm_config(uint32_t gpio, uint8_t func,
		      uint8_t dir, uint8_t pull,
		      uint8_t drvstr, uint32_t enable)
{
	uint32_t val = 0;
	val |= pull;
	val |= func << 2;
	val |= drvstr << 6;
	val |= enable << 9;
	writel(val, (unsigned int *)GPIO_CONFIG_ADDR(gpio));
	return;
}

void gpio_set(uint32_t gpio, uint32_t dir)
{
	writel(dir, (unsigned int *)GPIO_IN_OUT_ADDR(gpio));
	return;
}

/* Configure gpio for blsp uart 2 */
void gpio_config_uart_dm(uint8_t id)
{
    /* configure rx gpio */
	gpio_tlmm_config(5, 2, GPIO_INPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_DISABLE);

    /* configure tx gpio */
	gpio_tlmm_config(4, 2, GPIO_OUTPUT, GPIO_NO_PULL,
				GPIO_8MA, GPIO_DISABLE);
}

void gpio_config_blsp_i2c(uint8_t blsp_id, uint8_t qup_id)
{
	if (blsp_id == BLSP_ID_2) {
		switch (qup_id) {
		case QUP_ID_1:  // Variscite SD800-CustomBoard DSI to LVDS
			gpio_tlmm_config(47, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
			gpio_tlmm_config(48, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
		break;
		case QUP_ID_3:	// Lumus DK-50 AUX board (imu, ncp6925)
			gpio_tlmm_config(55, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
			gpio_tlmm_config(56, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
		break;
		case QUP_ID_4: // Variscite DART SD800 EEPROM 
			gpio_tlmm_config(83, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
			gpio_tlmm_config(84, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
		break;
		default:
			dprintf(CRITICAL, "Configure gpios for BLSP2 QUP instance: %u\n",
					  qup_id);
			ASSERT(0);
		};
	}
	else if (blsp_id == BLSP_ID_1) {
		switch (qup_id) {
		case QUP_ID_2:  // Lumus DK-50 Display 1
		    dprintf(CRITICAL, "Lumus DK-50 display 1 QUP %u\n",
		        qup_id);
			gpio_tlmm_config(10, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
			gpio_tlmm_config(11, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
		break;
		case QUP_ID_0: // Lumus DK-50 Display 2
		    dprintf(CRITICAL, "Lumus DK-50 display 2 QUP %u\n",
		        qup_id);
			gpio_tlmm_config(2, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
			gpio_tlmm_config(3, 3, GPIO_OUTPUT, GPIO_NO_PULL,
						GPIO_6MA, GPIO_DISABLE);
		break;
		default:
			dprintf(CRITICAL, "Configure gpios for BLSP1 QUP instance: %u\n",
					   qup_id);
			ASSERT(0);
		};
	}
}
