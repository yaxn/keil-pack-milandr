#include "MDR1986VE1T_PORT.h"

void ClkEnPort (MDR_PORT_TypeDef* Port)
{
	switch ((uint32_t)Port)
	{
		case ((uint32_t)MDR_PORTA): MDR_RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PCLK_EN_PORTA; break;
		case ((uint32_t)MDR_PORTB): MDR_RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PCLK_EN_PORTB; break;
		case ((uint32_t)MDR_PORTC): MDR_RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PCLK_EN_PORTC; break;
		case ((uint32_t)MDR_PORTD): MDR_RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PCLK_EN_PORTD; break;
		case ((uint32_t)MDR_PORTE): MDR_RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PCLK_EN_PORTE; break;
		case ((uint32_t)MDR_PORTF): MDR_RST_CLK->PER_CLOCK |= RST_CLK_PER_CLOCK_PCLK_EN_PORTF;
	}
}

void PortPinConfigure (MDR_PORT_TypeDef* Port, uint8_t Pin, uint8_t PinState)
{
	Port->RXTX &= ~(1<<Pin);
	Port->ANALOG |= (1<<Pin);
	Port->FUNC &= ~(PORT_FUNC_MODE_Msk<<Pin*2);
	
	if (PinState)
	{
		Port->OE |= (1<<Pin);
		Port->PWR |= (PORT_PWR_Msk<<Pin*2);
		Port->PD &= ~(1<<Pin);
	}
	else
	{
		Port->OE &= ~(1<<Pin);
		Port->PD &= ~(1<<(Pin+PORT_PD_SHM_Pos));
		Port->GFEN &= ~(1<<Pin);
	}
	
	Port->PULL &= ~((1<<Pin) | (1<<(Pin+PORT_PULL_UP_Pos)));
}

void PortPinWrite (MDR_PORT_TypeDef* Port, uint8_t Pin, uint8_t PinState)
{
	if (PinState)
		Port->RXTX |= (1<<Pin);
	else
		Port->RXTX &= ~(1<<Pin);
}
