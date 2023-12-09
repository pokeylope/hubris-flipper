// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![no_std]
#![no_main]

// We have to do this if we don't otherwise use it to ensure its vector table
// gets linked in.
extern crate stm32wb;

use core::arch::asm;
use cortex_m_rt::entry;
use stm32wb::stm32wb55 as device;

const CYCLES_PER_MS_BOOT: u32 = 4_000;
const CYCLES_PER_MS_MAIN: u32 = 64_000;

#[entry]
fn main() -> ! {
    let p = device::Peripherals::take().unwrap();

    setup_buttons(&p);
    delay_ms(100);
    check_dfu(&p);

    setup_clocks(&p);

    unsafe { kern::startup::start_kernel(CYCLES_PER_MS_MAIN) }
}

fn setup_clocks(p: &device::Peripherals) {
    /* Prepare Flash memory for 64MHz system clock */
    //LL_FLASH_SetLatency(LL_FLASH_LATENCY_3);
    //while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_3)
    //    ;
    p.FLASH.acr.modify(|_, w| w.latency().variant(3));
    while p.FLASH.acr.read().latency() != 3 {
        // spin
    }

    //* HSE and HSI configuration and activation */
    //LL_RCC_HSE_SetCapacitorTuning(0x26);
    //LL_RCC_HSE_Enable();
    //LL_RCC_HSI_Enable();
    //while(!HS_CLOCK_IS_READY())
    //    ;
    //LL_RCC_HSE_EnableCSS();
    p.RCC.hsecr.write(|w| w.key().unlock());
    p.RCC.hsecr.modify(|_, w| w.hsetune().variant(0x26));
    p.RCC
        .cr
        .modify(|_, w| w.hseon().enabled().hsion().enabled());
    loop {
        let cr = p.RCC.cr.read();
        if cr.hserdy().is_ready() && cr.hsirdy().is_ready() {
            break;
        }
    }
    p.RCC.cr.modify(|_, w| w.csson().enabled());

    /* LSE and LSI1 configuration and activation */
    //LL_PWR_EnableBkUpAccess();
    //LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH);
    //LL_RCC_LSE_Enable();
    //LL_RCC_LSI1_Enable();
    //while(!LS_CLOCK_IS_READY())
    //    ;
    //LL_EXTI_EnableIT_0_31(
    //    LL_EXTI_LINE_18); /* Why? Because that's why. See RM0434, Table 61. CPU1 vector table. */
    //LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_18);
    //LL_RCC_EnableIT_LSECSS();
    //LL_RCC_LSE_EnableCSS();
    p.PWR.cr1.modify(|_, w| w.dbp().set_bit());
    p.RCC.bdcr.modify(|_, w| w.lsedrv().high().lseon().on());
    p.RCC.csr.modify(|_, w| w.lsi1on().on());
    while !(p.RCC.bdcr.read().lserdy().bit_is_set()
        && p.RCC.csr.read().lsi1rdy().is_ready())
    {
        // spin
    }
    p.EXTI.imr1.modify(|_, w| w.im18().set_bit());
    p.EXTI.rtsr1.modify(|_, w| w.rt18().set_bit());
    p.RCC.cier.modify(|_, w| w.lsecssie().set_bit());
    p.RCC.bdcr.modify(|_, w| w.lsecsson().set_bit());

    /* Main PLL configuration and activation */
    //LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 8, LL_RCC_PLLR_DIV_2);
    //LL_RCC_PLL_Enable();
    //LL_RCC_PLL_EnableDomain_SYS();
    //while(LL_RCC_PLL_IsReady() != 1)
    //    ;
    p.RCC.pllcfgr.modify(|_, w| {
        w.pllsrc()
            .hse32()
            .pllm()
            .div2()
            .plln()
            .variant(8)
            .pllr()
            .div2()
    });
    p.RCC.cr.modify(|_, w| w.pllon().on());
    p.RCC.pllcfgr.modify(|_, w| w.pllren().enabled());
    while !p.RCC.cr.read().pllrdy().bit_is_set() {
        // spin
    }

    //LL_RCC_PLLSAI1_ConfigDomain_48M(
    //    LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 6, LL_RCC_PLLSAI1Q_DIV_2);
    //LL_RCC_PLLSAI1_ConfigDomain_ADC(
    //    LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_2, 6, LL_RCC_PLLSAI1R_DIV_2);
    //LL_RCC_PLLSAI1_Enable();
    //LL_RCC_PLLSAI1_EnableDomain_48M();
    //LL_RCC_PLLSAI1_EnableDomain_ADC();
    //while(LL_RCC_PLLSAI1_IsReady() != 1)
    //    ;
    p.RCC
        .pllcfgr
        .modify(|_, w| w.pllsrc().hse32().pllm().div2());
    p.RCC
        .pllsai1cfgr
        .modify(|_, w| w.plln().variant(6).pllq().div2().pllr().div2());
    p.RCC.cr.modify(|_, w| w.pllsai1on().on());
    p.RCC
        .pllsai1cfgr
        .modify(|_, w| w.pllqen().set_bit().pllren().set_bit());
    while !p.RCC.cr.read().pllsai1rdy().bit_is_set() {
        // spin
    }

    /* Sysclk activation on the main PLL */
    /* Set CPU1 prescaler*/
    //LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    p.RCC.cfgr.modify(|_, w| w.hpre().div1());

    /* Set CPU2 prescaler*/
    //LL_C2_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    p.RCC.extcfgr.modify(|_, w| w.c2hpre().div2());

    //LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    //while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    //    ;
    p.RCC.cfgr.modify(|_, w| w.sw().pllr());
    while !p.RCC.cfgr.read().sws().is_pllr() {
        // spin
    }

    /* Set AHB SHARED prescaler*/
    //LL_RCC_SetAHB4Prescaler(LL_RCC_SYSCLK_DIV_1);
    p.RCC.extcfgr.modify(|_, w| w.shdhpre().div1());

    /* Set APB1 prescaler*/
    //LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    /* Set APB2 prescaler*/
    //LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    p.RCC.cfgr.modify(|_, w| w.ppre1().div1().ppre2().div1());

    /* Disable MSI */
    //LL_RCC_MSI_Disable();
    //while(LL_RCC_MSI_IsReady() != 0)
    //    ;
    p.RCC.cr.modify(|_, w| w.msion().clear_bit());
    while p.RCC.cr.read().msirdy().is_ready() {
        // spin
    }

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    //LL_SetSystemCoreClock(CPU_CLOCK_HZ_MAIN);

    /* Update the time base */
    //LL_Init1msTick(SystemCoreClock);
    //LL_SYSTICK_EnableIT();
    //NVIC_SetPriority(
    //    SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), TICK_INT_PRIORITY, 0));
    //NVIC_EnableIRQ(SysTick_IRQn);

    //LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
    //LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);
    //LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLLSAI1);
    //LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
    //LL_RCC_SetRNGClockSource(LL_RCC_RNG_CLKSOURCE_CLK48);
    //LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLLSAI1);
    //LL_RCC_SetCLK48ClockSource(LL_RCC_CLK48_CLKSOURCE_PLLSAI1);
    //LL_RCC_SetSMPSClockSource(LL_RCC_SMPS_CLKSOURCE_HSE);
    //LL_RCC_SetSMPSPrescaler(LL_RCC_SMPS_DIV_1);
    //LL_RCC_SetRFWKPClockSource(LL_RCC_RFWKP_CLKSOURCE_LSE);
    p.RCC.ccipr.modify(|_, w| {
        w.usart1sel()
            .pclk()
            .lpuart1sel()
            .pclk()
            .adcsel()
            .pllsai1()
            .i2c1sel()
            .pclk()
            .rngsel()
            .clk48()
            .clk48sel()
            .pllsai1()
    });
    p.RCC
        .smpscr
        .modify(|_, w| w.smpssel().hse().smpsdiv().variant(1));
    p.RCC.csr.modify(|_, w| w.rfwkpsel().lse());
}

fn setup_buttons(p: &device::Peripherals) {
    // Set up GPIO for left button
    p.RCC.ahb2enr.modify(|_, w| w.gpioben().enabled());
    cortex_m::asm::dmb();
    p.GPIOB.moder.modify(|_, w| w.moder11().variant(0));
    p.GPIOB.pupdr.modify(|_, w| w.pupdr11().variant(1));
    p.GPIOB.ospeedr.modify(|_, w| w.ospeedr11().variant(0));
}

fn check_dfu(p: &device::Peripherals) {
    // Enter DFU mode if left button is held during boot
    if !p.GPIOB.idr.read().idr11().bit_is_set() {
        p.SYSCFG.memrmp.modify(|_, w| w.mem_mode().system_flash());
        cortex_m::asm::dsb();
        unsafe {
            asm!(
                "
                mov r0, #0
                ldr r1, [r0]
                msr msp, r1
                ldr r1, [r0, #4]
                mov pc, r1
                "
            );
        }
    }
}

fn delay_ms(ms: u32) {
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay =
        cortex_m::delay::Delay::new(cp.SYST, CYCLES_PER_MS_BOOT * 1000);
    delay.delay_ms(ms);
}
