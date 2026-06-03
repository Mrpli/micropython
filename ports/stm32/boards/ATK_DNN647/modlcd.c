/*
 * ATK-DNN647 LTDC RGB LCD driver (800x480, RGB565)
 * Based on 5_AISecond CubeIDE project LTDC configuration.
 */

// This file is picked up by $(wildcard $(BOARD_DIR)/*.c) in both mboot and
// main firmware builds.  Skip compilation for mboot.
#if !defined(BUILDING_MBOOT)

#include "py/obj.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "modlcd.h"
#include <stm32n6xx_hal_gpio.h>

#define LCD_WIDTH  800
#define LCD_HEIGHT 480

// Small test framebuffer to prove hardware before enabling HyperRAM.
#define FB_WIDTH   240
#define FB_HEIGHT  160
static uint16_t lcd_framebuf[FB_WIDTH * FB_HEIGHT] __attribute__((aligned(32)));

static bool lcd_initialised = false;

// ── LTDC pin mux (from 5_AISecond ltdc.c) ──────────────────────────
static void ltdc_pin_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitTypeDef g = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    };

    // PA: LTDC_G3,G2,B5,B4,B3,R5,CLK,B6,B7
    g.Alternate = GPIO_AF14_LCD;
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 | GPIO_PIN_10
          | GPIO_PIN_11 | GPIO_PIN_15 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &g);

    // PB: LTDC_R3,G6,G5,G4,G7
    g.Pin = GPIO_PIN_4 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &g);

    // PF: LTDC_R6,HSYNC
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOF, &g);

    // PG0: LTDC_VSYNC (AF10!)
    g.Alternate = GPIO_AF10_LCD;
    g.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOG, &g);

    // PG9,PG13: LTDC_R7,DE (AF14)
    g.Alternate = GPIO_AF14_LCD;
    g.Pin = GPIO_PIN_13 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOG, &g);

    // PH4: LTDC_R4
    g.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOH, &g);
}

// ── LTDC clock config: PLL3 @ 33.33MHz (matching 5_AISecond) ──────
// HSE(48) / PLL3M(3) * PLL3N(25) = 400MHz VCO
// PLL3R(÷1, default) → IC16(÷12) → LTDC pixel clock = 33.33MHz
static void ltdc_clock_init(void) {
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL3.PLLState = RCC_PLL_ON;
    osc.PLL3.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL3.PLLM = 3;
    osc.PLL3.PLLN = 25;
    osc.PLL3.PLLP1 = 1;
    osc.PLL3.PLLP2 = 1;
    osc.PLL3.PLLFractional = 0;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("PLL3 start failed"));
    }

    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    clk.LtdcClockSelection = RCC_LTDCCLKSOURCE_IC16;
    clk.ICSelection[RCC_IC16].ClockSelection = RCC_ICCLKSOURCE_PLL3;
    clk.ICSelection[RCC_IC16].ClockDivider = 12;
    if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("LTDC routing failed"));
    }
    __HAL_RCC_LTDC_CLK_ENABLE();
}

// ── LTDC peripheral init ───────────────────────────────────────────
static LTDC_HandleTypeDef hltdc;

static void ltdc_periph_init(void) {
    // ── RIF: grant LTDC master access to memory ───────────────────
    __HAL_RCC_RIFSC_CLK_ENABLE();
    RIMC_MasterConfig_t rimc = {
        .MasterCID = RIF_CID_1,
        .SecPriv = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV,
    };
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1, &rimc);
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC2, &rimc);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1,
        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL2,
        RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);

    // ── LTDC peripheral config ────────────────────────────────────
    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync = 87;
    hltdc.Init.VerticalSync = 39;
    hltdc.Init.AccumulatedHBP = 135;
    hltdc.Init.AccumulatedVBP = 71;
    hltdc.Init.AccumulatedActiveW = 935;
    hltdc.Init.AccumulatedActiveH = 551;
    hltdc.Init.TotalWidth = 948;
    hltdc.Init.TotalHeigh = 554;
    hltdc.Init.Backcolor.Blue = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red = 0;
    if (HAL_LTDC_Init(&hltdc) != HAL_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("LTDC init failed"));
    }

    // ── Layer0: RGB565 on small test window ────────────────────────
    LTDC_LayerCfgTypeDef layer = {0};
    layer.WindowX0 = 0;
    layer.WindowX1 = FB_WIDTH;
    layer.WindowY0 = 0;
    layer.WindowY1 = FB_HEIGHT;
    layer.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    layer.Alpha = 255;
    layer.Alpha0 = 0;
    layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    layer.FBStartAdress = (uint32_t)lcd_framebuf;
    layer.ImageWidth = FB_WIDTH;
    layer.ImageHeight = FB_HEIGHT;
    layer.Backcolor.Blue = 0;
    layer.Backcolor.Green = 0;
    layer.Backcolor.Red = 0;
    if (HAL_LTDC_ConfigLayer(&hltdc, &layer, 0) != HAL_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("LTDC layer failed"));
    }

    // Force immediate reload so the layer takes effect now
    LTDC->SRCR = LTDC_SRCR_IMR;
    // Wait for reload to complete
    while ((LTDC->SRCR & LTDC_SRCR_IMR) != 0) { }
}

// ── Backlight PWM (PA3, TIM16_CH1) ─────────────────────────────────
static void backlight_init(void) {
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {
        .Pin = GPIO_PIN_3,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = GPIO_AF1_TIM16,
    };
    HAL_GPIO_Init(GPIOA, &g);

    TIM_HandleTypeDef htim16 = { .Instance = TIM16 };
    htim16.Init.Prescaler = 0;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = 999;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim16);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim16, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
}

static void backlight_set(uint8_t brightness) {
    TIM16->CCR1 = (uint32_t)brightness * 999 / 255;
}

// ── Public API ─────────────────────────────────────────────────────

mp_obj_t lcd_init(void) {
    if (!lcd_initialised) {
        ltdc_pin_init();
        ltdc_clock_init();
        ltdc_periph_init();
        backlight_init();
        backlight_set(128);
        lcd_initialised = true;
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(lcd_init_obj, lcd_init);

mp_obj_t lcd_fill(mp_obj_t color_obj) {
    if (!lcd_initialised) lcd_init();
    uint16_t c = (uint16_t)mp_obj_get_int(color_obj);
    for (int i = 0; i < FB_WIDTH * FB_HEIGHT; i++) {
        lcd_framebuf[i] = c;
    }
    SCB_CleanDCache_by_Addr((uint32_t *)lcd_framebuf, sizeof(lcd_framebuf));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(lcd_fill_obj, lcd_fill);

mp_obj_t lcd_brightness(mp_obj_t val_obj) {
    uint8_t v = (uint8_t)mp_obj_get_int(val_obj);
    backlight_set(v);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(lcd_brightness_obj, lcd_brightness);

mp_obj_t lcd_pixel(size_t n_args, const mp_obj_t *args) {
    if (!lcd_initialised) lcd_init();
    int x = mp_obj_get_int(args[0]);
    int y = mp_obj_get_int(args[1]);
    uint16_t c = (uint16_t)mp_obj_get_int(args[2]);
    if (x >= 0 && x < FB_WIDTH && y >= 0 && y < FB_HEIGHT) {
        lcd_framebuf[y * FB_WIDTH + x] = c;
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(lcd_pixel_obj, 3, 3, lcd_pixel);

mp_obj_t lcd_show(void) {
    SCB_CleanDCache_by_Addr((uint32_t *)lcd_framebuf, sizeof(lcd_framebuf));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_0(lcd_show_obj, lcd_show);

static const mp_rom_map_elem_t lcd_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_lcd) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&lcd_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_fill),        MP_ROM_PTR(&lcd_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_brightness),  MP_ROM_PTR(&lcd_brightness_obj) },
    { MP_ROM_QSTR(MP_QSTR_pixel),       MP_ROM_PTR(&lcd_pixel_obj) },
    { MP_ROM_QSTR(MP_QSTR_show),        MP_ROM_PTR(&lcd_show_obj) },
};
static MP_DEFINE_CONST_DICT(lcd_module_globals, lcd_module_globals_table);

const mp_obj_module_t lcd_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&lcd_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_lcd, lcd_module);

#endif // !defined(BUILDING_MBOOT)
