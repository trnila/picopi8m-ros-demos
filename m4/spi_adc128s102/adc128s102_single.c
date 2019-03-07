#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_ecspi.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "common.h"


/**
 * this example requests all 8 channels in single SPI transaction
 * chip select is active until all channels are received
*/
int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    // 1. disable ECSPI block for reset
    ECSPI1->CONREG &= ~ECSPI_CONREG_EN_MASK;

    // 2. enable clocks
    CLOCK_EnableClock(kCLOCK_Ecspi1);
    CLOCK_SetRootMux(kCLOCK_RootEcspi1, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI1 source to SYSTEM PLL1 800 Mhz */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi1, 2U, 5U);                 /* Set root clock to 800 Mhz / 10 = 80 Mhz */

    // 3. configure + enable
    ECSPI1->CONREG = ECSPI_CONREG_BURST_LENGTH(16 * 8 - 1) // 16 bits per channel, -1 because minimum is 1 bit (which has value of 0)
        | ECSPI_CONREG_PRE_DIVIDER(4)  // 80 Mhz / (4+1) = 16 Mhz
        | ECSPI_CONREG_POST_DIVIDER(1)  // 16 Mhz / 2^(1+1) = 4 Mhz -> 8 Mhz SPI clock
        | ECSPI_CONREG_CHANNEL_MODE(0xF) // all master channels
        | ECSPI_CONREG_SMC(0) // start immediatelly transfer when we write to the TXDATA fifo
        | ECSPI_CONREG_EN_MASK; // finally enable block

    // 4. init pins
    BOARD_InitPins();
    
    // 5. configure rest of registers
    ECSPI1->CONFIGREG = ECSPI_CONFIGREG_SCLK_CTL(1) // clock is high when inactive
        | ECSPI_CONFIGREG_DATA_CTL(1) // data is high when inactive
        | ECSPI_CONFIGREG_SS_POL(0) // CS active low
        | ECSPI_CONFIGREG_SS_CTL(0) // just send single spi burst
        | ECSPI_CONFIGREG_SCLK_POL(1) // CPOL=1
        | ECSPI_CONFIGREG_SCLK_PHA(1) // CPHA=1
        ;

    // prepare transmit buffer
    // ADC will send previously set channel
    // so send channels 1 2 3 4 5 6 7 0
    // to receive chans 0 1 2 3 4 5 6 7
    uint32_t tx[] = {
        SAMPLE(1, 2),
        SAMPLE(3, 4),
        SAMPLE(5, 6),
        SAMPLE(7, 0),
    };
    int len = sizeof(tx) / sizeof(*tx);

    for(;;) {
        // send all words to tx FIFO
        for(int i = 0; i < len; i++) {
            ECSPI1->TXDATA = tx[i];
        }

        // start transmission 
        ECSPI1->CONREG |= ECSPI_CONREG_XCH_MASK;

        // wait until TX buffer is empty
        while(!(ECSPI1->STATREG & ECSPI_STATREG_TE_MASK));

        for(int i = 0; i < len; i++) {
            // RXFIFO Ready. This bit is set when one or more words are stored in the RXFIFO.
            while(ECSPI1->STATREG & ECSPI_STATREG_TF_MASK);

            volatile uint32_t r = ECSPI1->RXDATA;
            printf("%5d %5d ", SAMPLE_A(r), SAMPLE_B(r));
        }
        printf("\r\n");

        for(int i = 0; i < 240000; i++) asm("nop");
    }
}
