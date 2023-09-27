/*
 * Copyright (c) 2019 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT custom_ambt53

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "ambt53_xip_driver.h"
#include <zephyr/types.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include "custom_mspi_ambiq.h"
#include <am_mcu_apollo.h>
/**
 * This is a minimal example of an out-of-tree driver
 * implementation. See the header file of the same name for details.
 */
struct ambt53_config {
	uint32_t dqs_en;
	const struct device *bus;
	struct gpio_dt_spec reset_gpio;
	uint32_t spi_bus_width;
	uint32_t max_frequency;
};

static struct ambt53_data {
	const struct device *mspi_dev;
} xip_data;

static struct spi_config spi_cfg = {
	.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPHA | SPI_WORD_SET(8),
	.frequency = 4000000,
};

static struct mspi_buf g_mspi_buf;
static int ambt53_remote_init(const struct device *xip_dev, am_hal_mspi_dev_config_t *pConfig, bool bDQSEn);
static void ambt53_remote_reset(const struct device *xip_dev);

static int ambt53_init(const struct device *xip_dev)
{
	int ret;
	struct ambt53_data *data = xip_dev->data;
	const struct ambt53_config *config = xip_dev->config;

	data->mspi_dev = config->bus;
	if (!device_is_ready(data->mspi_dev)) {
		printk("mspi_dev is not ready\n");
		return -ENODEV;
	}

	ret = custom_mspi_config(data->mspi_dev, &spi_cfg);
	if (ret < 0) {
		printk("SPI configure error: %d\n", ret);
	}

	uint8_t header_master[6] = { 0xF1,0xF2,0xF3,0xF4,0xF5, 0xF6 };
	
	g_mspi_buf.buf = header_master;
	g_mspi_buf.len = 6;
	g_mspi_buf.send_addr = 1;
	g_mspi_buf.instr = 0xAB;
	g_mspi_buf.addr = 0x88776655;
	ret = custom_mspi_write(data->mspi_dev, &g_mspi_buf);
	if (ret < 0) {
		printk("SPI write error: %d\n", ret);
	}
	ret = custom_mspi_read(data->mspi_dev, &g_mspi_buf);
	if (ret < 0) {
		printk("SPI read error: %d\n", ret);
	}
	for (uint32_t i = 0; i < 5; i++)
	{
		printk("g_mspi_buf=%d:", header_master[i]);
	}

	ambt53_remote_reset(xip_dev);
	struct mspi_ambiq_data *mspi_dev_data = (struct mspi_ambiq_data *)data->mspi_dev->data;
	ambt53_remote_init(xip_dev, &mspi_dev_data->mspi_dev_cfg, false);

	// Switch MSPI to Quad mode
	spi_cfg.operation &= (~SPI_LINES_MASK);
	spi_cfg.operation |= SPI_LINES_QUAD;
	printk("QSPI configure =: 0x%x\n", spi_cfg.operation);
	ret = custom_mspi_config(data->mspi_dev, &spi_cfg);
	if (ret < 0) {
		printk("QSPI configure error: %d\n", ret);
	}
	ret = custom_mspi_enable_xip(data->mspi_dev);
	if (ret < 0) {
		printk("Enable XIP error: %d\n", ret);
	}
	#define MSPI_XIPMM_BASE_ADDRESS          0x14000000
	uint32_t read_reg = 0xFFFFFFFF;
	read_reg = *(volatile uint32_t*)(MSPI_XIPMM_BASE_ADDRESS + 0x03000020);
	printk("\nread_reg=0x%x\n", read_reg);
	*(volatile uint32_t*)(MSPI_XIPMM_BASE_ADDRESS + 0x03000020) = 0x89ABCDEF;
	return 0;
}


//*****************************************************************************
//
//  Get the ambt53 xSPI basic configuration.
//
//*****************************************************************************
static void ambt53_basic_config_get(const struct device *xip_dev)
{
	struct ambt53_data *data = xip_dev->data;
    uint32_t ui32Ver = 0;
    uint32_t ui32Cfg = 0;
    ambt53_status_reg_t stStatus = {0};
	g_mspi_buf.send_addr = 0;
	g_mspi_buf.addr = 0;
	g_mspi_buf.buf = &ui32Ver;
	g_mspi_buf.len = sizeof(ui32Ver);
	g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_READ_VER;
    custom_mspi_read(data->mspi_dev, &g_mspi_buf);
	g_mspi_buf.buf = &ui32Cfg;
	g_mspi_buf.len = sizeof(ui32Cfg);
	g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_READ_CFG;
    custom_mspi_read(data->mspi_dev, &g_mspi_buf);
    if ( (ui32Ver == 0) || (ui32Cfg == 0) )
    {
        printk("The xSPI IP is not working! Please check your hardware...\n");
        while (1);
    }
    printk("xSPI IP version: %d.%d.%d\n", (ui32Ver & 0xFF), ((ui32Ver & 0xFF00) >> 8), ((ui32Ver & 0xFF0000) >> 16));
    printk("AHB_DW: %d\n", (ui32Cfg & 0xFF000000) >> 24);

    ambt53_status_get(xip_dev, &stStatus);
    printk("xSPI transfer status: 0x%08X\r\n", stStatus.STATUS);
    ambt53_status_clear(xip_dev, &stStatus);
}

//*****************************************************************************
//
//  Get the MSPI status.
//
//*****************************************************************************
void ambt53_status_get(const struct device *xip_dev, ambt53_status_reg_t *pStatus)
{
	struct ambt53_data *data = xip_dev->data;
	g_mspi_buf.send_addr = 0;
	g_mspi_buf.addr = 0;
	g_mspi_buf.buf = pStatus;
	g_mspi_buf.len = sizeof(ambt53_status_reg_t);
	g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_READ_STS;
    custom_mspi_read(data->mspi_dev, &g_mspi_buf);
    pStatus->STATUS = __bswap_32(pStatus->STATUS);
}

//*****************************************************************************
//
//  Clear the MSPI status.
//
//*****************************************************************************
void ambt53_status_clear(const struct device *xip_dev, ambt53_status_reg_t *pStatus)
{
    pStatus->STATUS = __bswap_32(pStatus->STATUS);
	struct ambt53_data *data = xip_dev->data;
	g_mspi_buf.send_addr = 0;
	g_mspi_buf.addr = 0;
	g_mspi_buf.buf = pStatus;
	g_mspi_buf.len = sizeof(ambt53_status_reg_t);
	g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_WRITE_STS;
    custom_mspi_write(data->mspi_dev, &g_mspi_buf);
}

//*****************************************************************************
//
//
//
//*****************************************************************************
static void ambt53_remote_reset(const struct device *xip_dev)
{
    //Reset to default
	struct ambt53_data *data = xip_dev->data;
	g_mspi_buf.send_addr = 0;
	g_mspi_buf.addr = 0;
	g_mspi_buf.buf = NULL;
	g_mspi_buf.len = 0;
	g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOFT_RESET;
    custom_mspi_write(data->mspi_dev, &g_mspi_buf);
	// Delay at least 5 SPI cycles
	k_busy_wait(1);
}

//*****************************************************************************
//
//  Initialize the MSPI remote driver.
//
//*****************************************************************************
static int ambt53_remote_init(const struct device *xip_dev, am_hal_mspi_dev_config_t *pConfig, bool bDQSEn)
{
    struct ambt53_data *data = xip_dev->data;
	g_mspi_buf.send_addr = 0;
	g_mspi_buf.addr = 0;
	g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SET_DUMMY;
    
    if ( !bDQSEn )
    {
        uint32_t DummyBytes = 0;
        if ( !pConfig->bTurnaround )
        {
            DummyBytes = 0;
        }
        else
        {
            // Dummy should between 4-255
            if ( pConfig->ui8TurnAround >= 4 )
            {
                DummyBytes = pConfig->ui8TurnAround;
            }
            else
            {
                return -EINVAL;
            }
        }

		g_mspi_buf.len = 1;
		g_mspi_buf.buf = &DummyBytes;
		custom_mspi_write(data->mspi_dev, &g_mspi_buf);
    }

	g_mspi_buf.len = 0;
	g_mspi_buf.buf = NULL;
    switch ( pConfig->eDeviceConfig )
    {
        case AM_HAL_MSPI_FLASH_SERIAL_CE0:
        case AM_HAL_MSPI_FLASH_SERIAL_CE1:
			g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_1_1_1;
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0:
        case AM_HAL_MSPI_FLASH_DUAL_CE1:
			g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_2_2_2;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0:
        case AM_HAL_MSPI_FLASH_QUAD_CE1:
            if ( bDQSEn )
            {
				g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_4_4_4_DQS;
            }
            else
            {
				g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_4_4_4;
            }
            break;
        case AM_HAL_MSPI_FLASH_OCTAL_CE0:
        case AM_HAL_MSPI_FLASH_OCTAL_CE1:
            if ( bDQSEn )
            {
				g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_8_8_8_DQS;
            }
            else
            {
				g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_8_8_8;
            }
            break;
        case AM_HAL_MSPI_FLASH_DUAL_CE0_1_2_2:
        case AM_HAL_MSPI_FLASH_DUAL_CE1_1_2_2:
			g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_1_2_2;
            break;
        case AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4:
        case AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4:
            if ( bDQSEn )
            {
				g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_1_4_4_DQS;
            }
            else
            {
				g_mspi_buf.instr = AM_DEVICES_MSPI_AMBT53_SOPM_1_4_4;
            }
            break;
        default:
            return -ENOTSUP;
    }

    custom_mspi_write(data->mspi_dev, &g_mspi_buf);
    return 0;
}

static struct ambt53_xip_driver_api ambt53_xip_driver_api = {
	.print = NULL
	};

// DEVICE_DEFINE(ambt53_xip, "AMBT53_XIP_DRIVER",
// 		    init, NULL, &xip_data, NULL,
// 		    APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
// 		    &ambt53_xip_driver_api);

#define AMBT53_INIT(index)                                                     \
	static const struct ambt53_config ambt53_config_##index = {	       \
		.bus = DEVICE_DT_GET(DT_INST_BUS(index)),			       \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(index, reset_gpios, {0}),  \
		.max_frequency = DT_INST_PROP(index, max_frequency),  \
		.spi_bus_width = DT_INST_PROP(index, spi_bus_width),  \
		.dqs_en =  DT_INST_PROP(index, dqs_en),		       \
	};								       \
	static struct ambt53_data ambt53_data_##index;			       \
	DEVICE_DT_INST_DEFINE(index, ambt53_init, NULL,			       \
			    &ambt53_data_##index, &ambt53_config_##index,      \
			    APPLICATION, CONFIG_CUSTOM_AMBT53_INIT_PRIORITY, &ambt53_xip_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AMBT53_INIT)
