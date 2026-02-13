#ifndef __I2C_APP_H__
#define __I2C_APP_H__

#define TLV_I2C_ADDR 0x4C

int tlv320_init(void);
void confirm_tlv320(void);

/* TLV Configuration Structure */
typedef struct
{
    uint8_t resolution;   // 16 or 32
    bool TLVch1;
    bool TLVch2;
    bool TLVch3;

} tlv_config_t;

/* Universal instance */
extern tlv_config_t g_tlv_config;

#endif /* __I2C_APP_H__ */