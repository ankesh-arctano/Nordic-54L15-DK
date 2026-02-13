#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
#include "i2c_app.h"

/* Global configuration instance */
tlv_config_t g_tlv_config = {
    .resolution = 32,
    .TLVch1 = true,
    .TLVch2 = false,
    .TLVch3 = false
};

/* ================= I2C ================= */

#define I2C_NODE DT_NODELABEL(i2c21)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
static const struct i2c_dt_spec tlv = I2C_DT_SPEC_GET(DT_NODELABEL(tlv3120));


/* ================= I2C Functions ================= */

static void tlv320_write(uint8_t reg, uint8_t val)   {
    uint8_t buf[2] = { reg, val };
    if(0 == i2c_write_dt(&tlv, buf, sizeof(buf))) {
        printk("Wrote 0x%02X to register 0x%02X\n", val, reg);
    } else {
        printk("Failed to write to TLV320\n");
    }
}

static int tlv320_read(uint8_t reg, uint8_t *val)   {
	return i2c_write_read_dt(&tlv, &reg, 1, val, 1);
}


static int tlv320_write_verify(uint8_t reg, uint8_t val)    {
	int ret;
	uint8_t readback;

	tlv320_write(reg, val);
	k_usleep(50); /* small settle delay */

	ret = tlv320_read(reg, &readback);
	if (ret) {
		printk("TLV320 read failed: reg 0x%02X (%d)\n", reg, ret);
		return ret;
	}

	if (readback != val) {
		printk("TLV320 VERIFY FAIL: reg 0x%02X wrote 0x%02X read 0x%02X\n",
		       reg, val, readback);
		return -EIO;
	}

	printk("TLV320 OK: reg 0x%02X = 0x%02X\n", reg, readback);
	return 0;
}

int tlv320_init(void)   {

    /* Software reset */
uint8_t err = 0;
err |= tlv320_write_verify(0x01, 0x01);

/* AREG = 1.8V, device active */
err |= tlv320_write_verify(0x02, 0x81);

if (g_tlv_config.resolution == 32)  {
    printk("-----Resolution 32-bit------\n");

    /* I2S, 32-bit slot */
    err |= tlv320_write_verify(0x07, 0x70);

    /* FSYNC = 48kHz, Ratio = 64 */
    err |= tlv320_write_verify(0x14, 0x44);

}   else    {

    printk("-----Resolution 16-bit------\n");

    /* I2S, 16-bit slot */
    err |= tlv320_write_verify(0x07, 0x40);

    /* FSYNC = 48kHz, Ratio = 32 */
    err |= tlv320_write_verify(0x14, 0x42);
}

/* Transmit LSB full cycle, Bus Keeper enabled */
err |= tlv320_write_verify(0x08, 0x3F);

/* Slave mode, auto clock config, PLL enabled */
err |= tlv320_write_verify(0x13, 0x01);

/* MCLK_FREQ_SEL enabled, ratio = 512 */
err |= tlv320_write_verify(0x16, 0x08);

/* GPIO as MCLK input */
err |= tlv320_write_verify(0x21, 0xA0);

/* Input configuration */
err |= tlv320_write_verify(0x3C, 0x18);  // CH1 Differential, DC coupled, 20k
err |= tlv320_write_verify(0x41, 0x18);  // CH2 Differential, DC coupled, 20k
err |= tlv320_write_verify(0x46, 0x18);  // CH3 Differential, DC coupled, 20k
err |= tlv320_write_verify(0x4B, 0x38);  // CH4 Single-ended, DC coupled, 20k

/* Disable HPF */
err |= tlv320_write_verify(0x6B, 0x00);

/* Channel Routing Logic */

if (g_tlv_config.TLVch1)    {
    if (g_tlv_config.TLVch2)    {
        err |= tlv320_write_verify(0x0B, 0x00); // CH1 -> Left
        err |= tlv320_write_verify(0x0C, 0x20); // CH2 -> Right
        err |= tlv320_write_verify(0x73, 0xC0); // Enable CH1+2
        err |= tlv320_write_verify(0x74, 0xC0);
    }   else if (g_tlv_config.TLVch3)   {
        err |= tlv320_write_verify(0x0B, 0x00); // CH1 -> Left
        err |= tlv320_write_verify(0x0D, 0x20); // CH3 -> Right
        err |= tlv320_write_verify(0x73, 0xA0); // Enable CH1+3
        err |= tlv320_write_verify(0x74, 0xA0);
    }   else    {
        err |= tlv320_write_verify(0x0B, 0x00); // CH1 -> Left
        err |= tlv320_write_verify(0x0E, 0x20); // CH4 -> Right
        err |= tlv320_write_verify(0x73, 0x90); // Enable CH1+4
        err |= tlv320_write_verify(0x74, 0x90);
    }
}   else if (g_tlv_config.TLVch2)   {
            if (g_tlv_config.TLVch3)    {
                err |= tlv320_write_verify(0x0C, 0x00); // CH2 -> Left
                err |= tlv320_write_verify(0x0D, 0x20); // CH3 -> Right
                err |= tlv320_write_verify(0x73, 0x60); // Enable CH2+3
                err |= tlv320_write_verify(0x74, 0x60);
            }   else    {
                err |= tlv320_write_verify(0x0C, 0x00); // CH2 -> Left
                err |= tlv320_write_verify(0x0E, 0x20); // CH4 -> Right
                err |= tlv320_write_verify(0x73, 0x50); // Enable CH2+4
                err |= tlv320_write_verify(0x74, 0x50);
            }
        }   else if (g_tlv_config.TLVch3)   {
                err |= tlv320_write_verify(0x0D, 0x00); // CH3 -> Left
                err |= tlv320_write_verify(0x0E, 0x20); // CH4 -> Right
                err |= tlv320_write_verify(0x73, 0x30); // Enable CH3+4
                err |= tlv320_write_verify(0x74, 0x30);
        }   else    {
                err |= tlv320_write_verify(0x0E, 0x00); // CH4 -> Left
                err |= tlv320_write_verify(0x0B, 0x20); // CH1 -> Right
                err |= tlv320_write_verify(0x73, 0x90); // Enable CH1+4
                err |= tlv320_write_verify(0x74, 0x90);
        }

    /* Power up ADC + PLL */
    err |= tlv320_write_verify(0x75, 0x60);
    return err;
}

void confirm_tlv320(void)   {
    uint8_t dummy = 0x00;
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready!\n");
    }
    
    if(0 == i2c_write(i2c_dev, &dummy, 1, TLV_I2C_ADDR)) {   
        printk("Found device at 0x%02X\n", TLV_I2C_ADDR);
    }
}