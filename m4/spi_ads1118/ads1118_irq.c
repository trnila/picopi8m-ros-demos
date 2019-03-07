// TODO: WIP, not working yet

#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_ecspi.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"

#define IRQ_PIN 26

union ads1118_conf {
    struct {
        uint8_t reserved : 1;
        uint8_t nop : 2;
        uint8_t pull_up_en : 1;
        uint8_t ts_mode : 1;
        uint8_t dr : 3;

        uint8_t mode : 1;
        uint8_t pga : 3;
        uint8_t mux : 3;
        uint8_t ss : 1;

    };
    uint16_t val;
};

float pga_to_voltage[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256, 0.256, 0.256};

union ads1118_conf config;
volatile uint32_t tick;


void BOARD_InitPins(void) {
    // ECSPI1 alternate function
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_MISO_ECSPI1_MISO, 1U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI1_MISO_ECSPI1_MISO, 
            IOMUXC_SW_PAD_CTL_PAD_DSE(0U) |
            IOMUXC_SW_PAD_CTL_PAD_SRE(2U) |
            IOMUXC_SW_PAD_CTL_PAD_ODE_MASK);
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_MOSI_ECSPI1_MOSI, 1U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI1_MOSI_ECSPI1_MOSI, 
            IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
            IOMUXC_SW_PAD_CTL_PAD_SRE(2U) |
            IOMUXC_SW_PAD_CTL_PAD_ODE_MASK);
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_SCLK_ECSPI1_SCLK, 1U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI1_SCLK_ECSPI1_SCLK, 
            IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
            IOMUXC_SW_PAD_CTL_PAD_SRE(2U) |
            IOMUXC_SW_PAD_CTL_PAD_ODE_MASK);

    // read ready signal
    IOMUXC_SetPinMux(IOMUXC_SAI2_TXD0_GPIO4_IO26, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI2_TXD0_GPIO4_IO26, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(0U) | // high impedance
                        IOMUXC_SW_PAD_CTL_PAD_SRE(2U) | // slew rate - fast
                        IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK); // enable pull-up to prevent pin floating
}

void SysTick_Handler() {
    tick++;
}

void delay(uint32_t usec) {
    usec /= 1000;

    uint32_t dest = usec;
    uint32_t start = tick;
    while(tick - start < dest);
}

uint16_t read(union ads1118_conf new_config) {
    uint32_t rx = 0;
    uint32_t tx = new_config.val;

    ecspi_transfer_t masterXfer;

    // first we need to change configuration
    if(new_config.val != config.val) {
        masterXfer.txData = &tx;
        masterXfer.rxData = &rx;
        masterXfer.dataSize = 1;
        masterXfer.channel = kECSPI_Channel0;
        ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);

//        for(;;);

        // wait for change
        delay(5000);

        config = new_config;
    }

    // read current measurement from register
    masterXfer.txData = &tx;
    masterXfer.rxData = &rx;
    masterXfer.dataSize = 1;
    masterXfer.channel = kECSPI_Channel0;
    ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);

    return rx;
}

float read_temp() {
    union ads1118_conf new_conf = config;
    new_conf.ts_mode = 1;
    return ((int16_t) (read(new_conf) >> 2)) * 0.03125;
}

/**
 * channel can be in range 0 - 3
 */
uint16_t read_channel(int channel) {
    union ads1118_conf new_conf = config;
    new_conf.ts_mode = 0;
    new_conf.mux = 0b100 + channel;
    return read(new_conf);
}

/**
 * channel can be in range 0-3
 */
float read_voltage(int channel) {
    return (int16_t) read_channel(channel) / 32768.0 * pga_to_voltage[config.pga];
}


volatile int irqd = 0;
void GPIO4_Combined_16_31_IRQHandler() {
    if(GPIO4->ISR & (1 << IRQ_PIN)) {
        GPIO_PortClearInterruptFlags(GPIO4, 1 << IRQ_PIN); // GPIO4->ISR = 1 << PIN;
        irqd++;
    }
}

    float vals[10];
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("SPI1 ADS1118 example started\r\n");

    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalInput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_IntFallingEdge;
    GPIO_PinInit(GPIO4, IRQ_PIN, &conf);
    EnableIRQ(GPIO4_Combined_16_31_IRQn);

    conf.direction = kGPIO_DigitalOutput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_IntFallingEdge;
    GPIO_PinInit(GPIO4, 23, &conf);

    // setup timer so that we have delay function for 1 ms
    uint32_t period_us = 1000;
    uint32_t ticks = SystemCoreClock / 1000000UL * period_us;
    SysTick_Config(ticks);

    config.ss         = 0; // continuous measurements
    config.mux        = 0b100; // IN0
    config.pga        = 0b001; // +- 4V
    config.mode       = 0; // continuous measurements
    config.dr         = 0b111; // 860 Samples/sec
    config.ts_mode    = 1; // temperature mode
    config.pull_up_en = 0;
    config.nop        = 0b01; // mark valid data in configuration
    config.reserved   = 1; // 1 is required for write

    CLOCK_EnableClock(kCLOCK_Ecspi1);
    CLOCK_SetRootMux(kCLOCK_RootEcspi1, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI2 source to SYSTEM PLL1 800MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi1, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */

    ecspi_master_config_t masterConfig;
    ECSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.burstLength = 16; // we are sending 16 bits in single transaction
    masterConfig.baudRate_Bps = 500000U;
    masterConfig.channelConfig.polarity = kECSPI_ClockPhaseSecondEdge;
//    masterConfig.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateHigh;
//    masterConfig.channelConfig.waveForm = kECSPI_WaveFormSingle;


    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi1)) / (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi1));

    ECSPI_MasterInit(ECSPI1, &masterConfig, clk);

    config.ts_mode = 0;
    config.mux = 0b100;
/*
    for(;;) {
        float temp = read_temp();

        printf("temp = %f", temp);

        for(int c = 0; c < 4; c++) {
            printf(", IN%d = %8.4f", c, read_voltage(c));
        }
        printf("\r\n");
    }
    */

    int k = 0;
    for(;;) {
        irqd = 0;

        uint32_t rx = 0;
        uint32_t tx = 0x00;


        ecspi_transfer_t masterXfer;
        masterXfer.txData = &tx;
        masterXfer.rxData = &rx;
        masterXfer.dataSize = 1;
        masterXfer.channel = kECSPI_Channel0;
        ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);

//        for(int i = 0; i < 640000; i++) asm("nop");
        GPIO_EnableInterrupts(GPIO4, 1 << IRQ_PIN);
        GPIO_PinWrite(GPIO4, 23, 1);
        while(irqd <= 1);
        GPIO_DisableInterrupts(GPIO4, 1 << IRQ_PIN);

        GPIO_PinWrite(GPIO4, 23, 0);

        float val = rx/ 32768.0 * pga_to_voltage[config.pga];

        vals[k++] = val;


        if(k >= 9) {
            for(int i = 0; i < k; i++) 
                printf("%f ", vals[i]);
            printf("\r\n");
            k = 0;
        }

    }
}
