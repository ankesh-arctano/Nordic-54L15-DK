#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
#include "i2s_config.h"
#include "../i2c_app/i2c_app.h"
#include <stdlib.h>
#define I2S_NODE DT_NODELABEL(i2s20)

/* ================= I2S ================= */
/* 1024 stereo frames × 32-bit × 2 channels = 8192 bytes */

#define SAMPLE_RATE 48000
#define WORD_SIZE   32
#define CHANNELS    2

#define FRAMES      128
#define BLOCK_SIZE  (FRAMES * CHANNELS * sizeof(int32_t))
#define NUM_BLOCKS  8   /* important: prevent starvation */

/* ================= DEVICES ================= */

static const struct device *i2s_dev;

/* RX + TX slabs */
K_MEM_SLAB_DEFINE(rx_slab, BLOCK_SIZE, NUM_BLOCKS, 4);
K_MEM_SLAB_DEFINE(tx_slab, BLOCK_SIZE, NUM_BLOCKS, 4);


/* ================= I2S Functions ================= */

static int start_i2s(void) {
    struct i2s_config cfg = {0};
    cfg.word_size = WORD_SIZE;
    cfg.channels = CHANNELS;
    cfg.format = I2S_FMT_DATA_FORMAT_I2S;
    cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
    cfg.frame_clk_freq = SAMPLE_RATE;
    cfg.block_size = BLOCK_SIZE;
    cfg.timeout = 2000;

    /* Configure RX */
    cfg.mem_slab = &rx_slab;
    int ret = i2s_configure(i2s_dev, I2S_DIR_RX, &cfg);
    if (ret) return ret;

    /* Configure TX */
    cfg.mem_slab = &tx_slab;
    ret = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
    return ret;
}

void audio_rx_thread()  {
    printk("\n*** TLV320 + I2S bring-up ***\n");
    i2s_dev = DEVICE_DT_GET(I2S_NODE);
    
    if (!device_is_ready(i2s_dev)) {
        printk("I2S device not ready\n");
    }

    int ret = start_i2s();
        printk("Return from i2s_init(): %d\n", ret);
    if (ret) {
        printk("I2S config failed\n");
    }

    k_msleep(10);
    /* Prime TX with zero blocks */
    for (int i = 0; i < NUM_BLOCKS; i++) {
        void *blk;
        k_mem_slab_alloc(&tx_slab, &blk, K_FOREVER);
        memset(blk, 0, BLOCK_SIZE);
        i2s_write(i2s_dev, blk, BLOCK_SIZE);
    }
    printk("Starting RX...\n");
    i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);

    k_busy_wait(100);   // allow RX arm

    printk("Starting TX clock...\n");
    i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);

    uint8_t status_tlv = tlv320_init();
    if(status_tlv != 0) {
        printk("TLV320 initialization failed: %d\n", status_tlv);
    } else {
        printk("TLV320 initialization succeeded: %d\n", status_tlv);
    }

    printk("Capturing audio...\n");

    while (1)   {
        void *block;
        size_t size;

        ret = i2s_read(i2s_dev, &block, &size);

        if (ret == 0)   {
            int32_t *samples = (int32_t *)block;
            int64_t sum1 = 0;
            int64_t sum2 = 0;
            //int64_t level = 0;
            for (int i = 0; i < FRAMES * CHANNELS; i++) {
                //level += llabs(samples[i]);
                int64_t sample_l= samples[i]>>8;  // 24-bit align
                int64_t sample_r= samples[i+1]>>8;  // 24-bit align
                sum1 += llabs(sample_l);
                sum2 += llabs(sample_r);
            }


            int64_t level_l = sum1 / (FRAMES * CHANNELS);
            int64_t level_r = sum2 / (FRAMES * CHANNELS);

            //printk("Audio Level Left: %lld      Audio Level Right: %lld\n", (long long)level_l, level_r);
            printk("%lld, %lld\n", (long long)level_l, (long long)level_r); //CSV for Arduino
            //printk("Audio Level: %lld\n",(long long)(level / (FRAMES * CHANNELS)));

            k_mem_slab_free(&rx_slab, block);
        }   else    {
            printk("i2s_read error: %d\n", ret);

            /* Proper recovery */
            i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
            i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);

            k_msleep(10);

            i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
            i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
        }
    }    
}

K_THREAD_STACK_DEFINE(audio_thread_stack, AUDIO_THREAD_STACK_SIZE);
static struct k_thread audio_thread_data;

void start_audio_thread(void)   {
    printk("Starting audio thread\n");
    confirm_tlv320();
    k_thread_create(&audio_thread_data,
                    audio_thread_stack,
                    AUDIO_THREAD_STACK_SIZE,
                    audio_rx_thread,
                    NULL, NULL, NULL,
                    AUDIO_THREAD_PRIORITY,
                    0,
                    K_NO_WAIT);

    k_thread_name_set(&audio_thread_data, "audio_rx");
}