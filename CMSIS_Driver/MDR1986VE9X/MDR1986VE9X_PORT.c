#include "MDR1986VE9X_PORT.h"

void ClkEnPort (MDR_PORT_TypeDef* Port)
{
	switch ((uint32_t)Port)
	{
		case ((uint32_t)MDR_PORTA): SET_BIT_PER (MDR_RST_CLK->PER_CLOCK, RST_CLK_PER_CLOCK_PCLK_EN_PORTA_Pos); break;
		case ((uint32_t)MDR_PORTB): SET_BIT_PER (MDR_RST_CLK->PER_CLOCK, RST_CLK_PER_CLOCK_PCLK_EN_PORTB_Pos); break;
		case ((uint32_t)MDR_PORTC): SET_BIT_PER (MDR_RST_CLK->PER_CLOCK, RST_CLK_PER_CLOCK_PCLK_EN_PORTC_Pos); break;
		case ((uint32_t)MDR_PORTD): SET_BIT_PER (MDR_RST_CLK->PER_CLOCK, RST_CLK_PER_CLOCK_PCLK_EN_PORTD_Pos); break;
		case ((uint32_t)MDR_PORTE): SET_BIT_PER (MDR_RST_CLK->PER_CLOCK, RST_CLK_PER_CLOCK_PCLK_EN_PORTE_Pos); break;
		case ((uint32_t)MDR_PORTF): SET_BIT_PER (MDR_RST_CLK->PER_CLOCK, RST_CLK_PER_CLOCK_PCLK_EN_PORTF_Pos);
	}
}

void PortPinConfigure (MDR_PORT_TypeDef* Port, uint8_t Pin, uint8_t PinState)
{
	SET_BIT_PER (Port->ANALOG, Pin);

	Port->FUNC &= ~(PORT_FUNC_MODE_Msk<<Pin*2);
	
	if (PinState)
	{
		CLR_BIT_PER (Port->RXTX, Pin);
		SET_BIT_PER (Port->OE, Pin);
		CLR_BIT_PER (Port->PD, Pin);
		Port->PWR |= (PORT_PWR_Msk<<Pin*2);
	}
	else
	{
		CLR_BIT_PER (Port->OE, Pin);
		CLR_BIT_PER (Port->PD, Pin+PORT_PD_SHM_Pos);
		CLR_BIT_PER (Port->GFEN, Pin);
	}
	
	Port->PULL &= ~((1<<Pin) | (1<<(Pin+PORT_PULL_UP_Pos)));
}

void PortPinWrite (MDR_PORT_TypeDef* Port, uint8_t Pin, uint8_t PinState)
{
	if (PinState)
		SET_BIT_PER (Port->RXTX, Pin);
	else
		CLR_BIT_PER (Port->RXTX, Pin);
}
