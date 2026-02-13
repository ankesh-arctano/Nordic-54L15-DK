#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include "i2s_config.h"
#include "i2c_app.h"


/* ================= MAIN ================= */

int main(void)
{
    start_audio_thread();
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}
