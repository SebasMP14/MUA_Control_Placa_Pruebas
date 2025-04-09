#include "power_manager.h"

void enterOffMode() {
    // Configurar el modo OFF
    PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_OFF_Val;

    // Esperar hasta que el modo de sueño esté correctamente configurado
    while (PM->SLEEPCFG.bit.SLEEPMODE != PM_SLEEPCFG_SLEEPMODE_OFF_Val);

    // Apagar osciladores principales (reemplazando DFLL)
    OSCCTRL->Dpll[0].DPLLCTRLA.bit.ENABLE = 0; // Apagar DPLL0
    while (OSCCTRL->Dpll[0].DPLLSTATUS.bit.LOCK);
    
    OSCCTRL->Dpll[1].DPLLCTRLA.bit.ENABLE = 0; // Apagar DPLL1
    while (OSCCTRL->Dpll[1].DPLLSTATUS.bit.LOCK);

    // Desactivar los generadores de reloj principales
    GCLK->GENCTRL[0].bit.DIVSEL = 1;  // Apagar GCLK0
    GCLK->GENCTRL[1].bit.DIVSEL = 1;  // Apagar GCLK1
    GCLK->GENCTRL[2].bit.DIVSEL = 1;  // Apagar GCLK2

    // Desactivar Watchdog si está habilitado
    WDT->CTRLA.bit.ENABLE = 0;

    // Esperar interrupción (sólo un reset lo sacará de este estado)
    __WFI();
}





// void setupRTCWakeup() {
//     RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_SWRST; // Reset RTC
//     while (RTC->MODE0.CTRL.reg & RTC_MODE0_CTRL_SWRST);
    
//     RTC->MODE0.CTRL.reg = RTC_MODE0_CTRL_MODE_COUNT32 | RTC_MODE0_CTRL_ENABLE;
//     while (RTC->MODE0.STATUS.bit.SYNCBUSY);
    
//     RTC->MODE0.COMP[0].reg = 32768; // 1 segundo si usa reloj de 32.768 kHz
//     RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0; // Habilita interrupción
//     NVIC_EnableIRQ(RTC_IRQn);
// }

// void RTC_Handler() {
//     RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0; // Limpiar flag de interrupción
// }
