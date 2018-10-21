#include "stm8s_conf.h"
#include "stm8s_it.h"

#ifndef TRAP_IRQ
//TRAP Interrupt routine
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
	while (1){};  
}
#endif

#ifndef TLI_IRQ
//Top Level Interrupt routine.
INTERRUPT_HANDLER(TLI_IRQHandler, 0)

{
	while (1){};
}
#endif

#ifndef AWU_IRQ 
//Auto Wake Up Interrupt routine.
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
	while (1){};
}
#endif

#ifndef CLK_IRQ 
//Clock Controller Interrupt routine.
INTERRUPT_HANDLER(CLK_IRQHandler, 2)
{
	while (1){};
}
#endif

#ifndef EXTI_PORTA_IRQ 
//External Interrupt PORTA Interrupt routine.
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
	while (1){};
}
#endif

#ifndef EXTI_PORTB_IRQ 
//External Interrupt PORTB Interrupt routine.
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
	while (1){};
}
#endif

#ifndef EXTI_PORTC_IRQ 
//External Interrupt PORTC Interrupt routine.
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
	while (1){};
}
#endif

#ifndef EXTI_PORTD_IRQ 
//External Interrupt PORTD Interrupt routine.
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
{
	while (1){};
}
#endif

#ifndef EXTI_PORTE_IRQ 
//External Interrupt PORTE Interrupt routine.
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7)
{
	while (1){};
}
#endif


#if defined (STM8S903) || defined (STM8AF622x) 
#ifndef EXTI_PORTF_IRQ
//External Interrupt PORTF Interrupt routine.
INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 8)
{
	while (1){};
}
#endif
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined (STM8AF52Ax)
#ifndef CAN_RX_IRQ
//CAN RX Interrupt routine.
INTERRUPT_HANDLER(CAN_RX_IRQHandler, 8)
{
	while (1){};
}
#endif

#ifndef CAN_TX_IRQ 
//CAN TX Interrupt routine.
INTERRUPT_HANDLER(CAN_TX_IRQHandler, 9)
{
	while (1){};
}
#endif
#endif /* (STM8S208) || (STM8AF52Ax) */

#ifndef SPI_IRQ
//SPI Interrupt routine.
INTERRUPT_HANDLER(SPI_IRQHandler, 10)
{
	while (1){};
}
#endif

#ifndef TIM1_UPD_OVF_TRG_BRK_IRQ 
//Timer1 Update/Overflow/Trigger/Break Interrupt routine.
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
	while (1){};
}
#endif

#ifndef TIM1_CAP_COM_IRQ 
//Timer1 Capture/Compare Interrupt routine.
INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12)
{
	while (1){};
}
#endif

#if defined (STM8S903) || defined (STM8AF622x)
#ifndef TIM5_UPD_OVF_BRK_TRG_IRQ 
//Timer5 Update/Overflow/Break/Trigger Interrupt routine.
INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13)
{
	while (1){};
}
#endif

#ifndef TIM5_CAP_COM_IRQ 
//Timer5 Capture/Compare Interrupt routine.
INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14)
{
	while (1){};
}
#endif
#else /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8S103) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x) */
#ifndef TIM2_UPD_OVF_BRK_IRQ
//Timer2 Update/Overflow/Break Interrupt routine.
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
	while (1){};
}
#endif

#ifndef TIM2_CAP_COM_IRQ 
//Timer2 Capture/Compare Interrupt routine.
INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14)
{
	while (1){};
}
#endif
#endif /* (STM8S903) || (STM8AF622x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8AF626x)
#ifndef TIM3_UPD_OVF_BRK_IRQ 
//Timer3 Update/Overflow/Break Interrupt routine.
INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
{
	while (1){};
}
#endif

#ifndef TIM3_CAP_COM_IRQ 
//Timer3 Capture/Compare Interrupt routine.
INTERRUPT_HANDLER(TIM3_CAP_COM_IRQHandler, 16)
{
	while (1){};
}
#endif
#endif /* (STM8S208) || (STM8S207) || (STM8S105) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x) */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S103) || \
    defined(STM8S003) ||  defined (STM8AF62Ax) || defined (STM8AF52Ax) || defined (STM8S903)
#ifndef UART1_TX_IRQ 
//UART1 TX Interrupt routine.
INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
{
	while (1){};
}
#endif

#ifndef UART1_RX_IRQ 
//UART1 RX Interrupt routine.
INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{
	while (1){};
}
#endif
#endif /* (STM8S208) || (STM8S207) || (STM8S103) || (STM8S903) || (STM8AF62Ax) || (STM8AF52Ax) */

#if defined(STM8AF622x)
#ifndef UART4_TX_IRQ
//UART4 TX Interrupt routine.
INTERRUPT_HANDLER(UART4_TX_IRQHandler, 17)
{
	while (1){};
}
#endif

#ifndef UART4_RX_IRQ
//UART4 RX Interrupt routine.
INTERRUPT_HANDLER(UART4_RX_IRQHandler, 18)
{
	while (1){};
}
#endif
#endif /* (STM8AF622x) */

#ifndef I2C_IRQ
//I2C Interrupt routine.
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
	while (1){};
}
#endif

#if defined(STM8S105) || defined(STM8S005) ||  defined (STM8AF626x)
#ifndef UART2_TX_IRQ
//UART2 TX interrupt routine.
INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{
	while (1){};
}
#endif

#ifndef UART2_RX_IRQ 
//UART2 RX interrupt routine.
INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
	while (1){};
}
#endif
#endif /* (STM8S105) || (STM8AF626x) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
#ifndef UART3_TX_IRQ 
//UART3 TX interrupt routine.
INTERRUPT_HANDLER(UART3_TX_IRQHandler, 20)
{
	while (1){};
}
#endif

#ifndef UART3_RX_IRQ 
//UART3 RX interrupt routine.
INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
{
	while (1){};
}
#endif
#endif /* (STM8S208) || (STM8S207) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
#ifndef ADC2_IRQ 
//ADC2 interrupt routine.
INTERRUPT_HANDLER(ADC2_IRQHandler, 22)
{
	while (1){};
}
#endif
#else /* STM8S105 or STM8S103 or STM8S903 or STM8AF626x or STM8AF622x */
#ifndef ADC1_IRQ 
//ADC1 interrupt routine.
INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{
	while (1){};
}
#endif
#endif /* (STM8S208) || (STM8S207) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined (STM8S903) || defined (STM8AF622x)
#ifndef TIM6_UPD_OVF_TRG_IRQ 
//Timer6 Update/Overflow/Trigger Interrupt routine.
INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23)
{
	while (1){};
}
#endif
#else /* STM8S208 or STM8S207 or STM8S105 or STM8S103 or STM8AF52Ax or STM8AF62Ax or STM8AF626x */
#ifndef TIM4_UPD_OVF_IRQ
//Timer4 Update/Overflow Interrupt routine.
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
	while (1){};
}
#endif
#endif /* (STM8S903) || (STM8AF622x)*/

#ifndef EEPROM_EEC_IRQ 
//Eeprom EEC Interrupt routine.
INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24)
{
	while (1){};
}
#endif