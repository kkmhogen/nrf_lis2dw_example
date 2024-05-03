#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "lis2dw12_reg.h"
#include <stdlib.h>

const nrf_drv_twi_t* m_p_acc_twi = NULL;
static stmdev_ctx_t dev_ctx;
static uint8_t whoamI, rst;
static lis2dw12_reg_t int_route;

#define LIS2DW12_ADDR   0x19
#define MAX_TX_BUFF_LEN 8
#define    BOOT_TIME            20 //20 ms

static int32_t platform_read(void *ptr, uint8_t addr, uint8_t *buf, uint16_t len)
{
    int32_t err_code = 0;
    uint8_t* p_buffer = buf;

    for (int i = 0; i < len; i++)
    {
        nrf_drv_twi_tx(m_p_acc_twi,  LIS2DW12_ADDR, &addr, 1, true);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_drv_twi_rx(m_p_acc_twi, LIS2DW12_ADDR, p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        p_buffer++;
        addr++;
    }

    return err_code;
}

static int32_t platform_write(void *ptr, uint8_t addr, const uint8_t *buf, uint16_t len)
{
    uint8_t txBuf[MAX_TX_BUFF_LEN];
    
    APP_ERROR_CHECK(len > MAX_TX_BUFF_LEN);

    txBuf[0] = addr;
    for (int i = 0; i < len; i++)
    {
        txBuf[i+1] = buf[i];
    }

    int32_t err_code = nrf_drv_twi_tx(m_p_acc_twi, LIS2DW12_ADDR, txBuf,  len+1, true);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

static void platform_delay(uint32_t millisec)
{
    nrf_delay_ms(millisec);
}

bool LIS2DW_init(nrf_drv_twi_t const * p_instance)
{
    m_p_acc_twi = p_instance;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &m_p_acc_twi;

    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);

    /* Check device ID */
    lis2dw12_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LIS2DW12_ID)
    {
        while (1);
        return false;
    }

    /* Restore default configuration */
    lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do
    {
        lis2dw12_reset_get(&dev_ctx, &rst);
    }
    while (rst);

    return true;
}

bool LIS2DW_motion_example(uint8_t odr, uint8_t accRange, uint8_t nDuration)
{
    //lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set full scale */
    lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_2g);

    /* Configure power mode */
    lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_2);

    /* Set Output Data Rate */
    LIS2DH_SetODR(odr);

    /* Apply high-pass digital filter on Wake-Up function */
    lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT);

    lis2dw12_wkup_dur_set(&dev_ctx, nDuration);
    
    /* Set wake-up threshold
    * Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6
    */
    lis2dw12_wkup_threshold_set(&dev_ctx, accRange);

    /* Enable interrupt generation on Wake-Up INT1 pin */
    lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
    int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE;

    lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

    LIS2DH_GetInt1Src(NULL);

    NRF_LOG_INFO("motion enable");

    return true;
}


