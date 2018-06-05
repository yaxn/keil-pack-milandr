/**
  ******************************************************************************
  * @file    system_MDR1986VE1T.c
  * @author  Milandr Application Team
  * @version V1.4.0
  * @date    11/06/2013
  * @brief   CMSIS Cortex-M1 Device Peripheral Access Layer System Source File.
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Milandr SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 Milandr</center></h2>
  ******************************************************************************
  * FILE system_MDR1986VE1T.c
  */


/** @addtogroup __CMSIS CMSIS
  * @{
  */

/** @defgroup MDR1986VE1T
 *  @{
 */

/** @addtogroup __MDR1986VE1T MDR1986VE1T System
  * @{
  */

/** @addtogroup System_Private_Includes System Private Includes
  * @{
  */

#include "MDR1986VE1T.h"
#include "MDR32F9Qx_config.h"
#include "MDR32F9Qx_rst_clk.h"

/** @} */ /* End of group System_Private_Includes */

/** @addtogroup __MDR1986VE1T_System_Private_Variables MDR1986VE1T System Private Variables
  * @{
  */

/*******************************************************************************
*  Clock Definitions
*******************************************************************************/
  uint32_t SystemCoreClock = (uint32_t)8000000;         /*!< System Clock Frequency (Core Clock)
                                                         *   default value */

/** @} */ /* End of group __MDR1986VE1T_System_Private_Variables */

/** @addtogroup __MDR1986VE1T_System_Private_Functions MDR1986VE1T System Private Functions
  * @{
  */

/**
  * @brief  Update SystemCoreClock according to Clock Register Values
  * @note   None
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate (void)
{
  uint32_t cpu_c1_freq, cpu_c2_freq, cpu_c3_freq;
  uint32_t pll_mul;

  /* Compute CPU_CLK frequency */

  /* Determine CPU_C1 frequency */
  if ((MDR_RST_CLK->CPU_CLOCK & (uint32_t)0x00000002) == (uint32_t)0x00000002)
  {
    cpu_c1_freq = HSE_Value;
  }
  else
  {
    cpu_c1_freq = HSI_Value;
  }

  if ((MDR_RST_CLK->CPU_CLOCK & (uint32_t)0x00000001) == (uint32_t)0x00000001)
  {
    cpu_c1_freq /= 2;
  }

  /* Determine CPU_C2 frequency */
  cpu_c2_freq = cpu_c1_freq;

  if ((MDR_RST_CLK->CPU_CLOCK & (uint32_t)0x00000004) == (uint32_t)0x00000004)
  {
    /* Determine CPU PLL output frequency */
    pll_mul = ((MDR_RST_CLK->PLL_CONTROL >> 8) & (uint32_t)0x0F) + 1;
    cpu_c2_freq *= pll_mul;
  }

  /*Select CPU_CLK from HSI, CPU_C3, LSE, LSI cases */
  switch ((MDR_RST_CLK->CPU_CLOCK >> 8) & (uint32_t)0x03)
  {
    uint32_t tmp;
    case 0 :
      /* HSI */
      SystemCoreClock = HSI_Value;
      break;
    case 1 :
      /* CPU_C3 */
      /* Determine CPU_C3 frequency */
      tmp = MDR_RST_CLK->CPU_CLOCK >> 4 & (uint32_t)0x0F;
      if (tmp & (uint32_t)0x8)
      {
        tmp &= (uint32_t)0x7;
        cpu_c3_freq = cpu_c2_freq / ((uint32_t)2 << tmp);
      }
      else
      {
        cpu_c3_freq = cpu_c2_freq;
      }
      SystemCoreClock = cpu_c3_freq;
      break;
    case 2 :
      /* LSE */
      SystemCoreClock = LSE_Value;
      break;
    default : /* case 3 */
      /* LSI */
      SystemCoreClock = LSI_Value;
      break;
  }
}

/**
  * @brief  Setup the microcontroller system
  *         RST clock configuration to the default reset state
  *         Setup SystemCoreClock variable.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
void SystemInit( void )
{
	NVIC->ICER[ 0 ] = 0xFFFFFFFF;  /* Disable all interrupts */
	NVIC->ICPR[ 0 ] = 0xFFFFFFFF;  /* Reset all interrupts */

	/* Reset all clock but RST_CLK and BKP_CLK */
	MDR_RST_CLK->PER_CLOCK =
	  RST_CLK_PCLK_RST_CLK
	| RST_CLK_PCLK_BKP;
	MDR_RST_CLK->CPU_CLOCK     = 0UL;
	MDR_RST_CLK->PLL_CONTROL   = 0UL;
	MDR_RST_CLK->HS_CONTROL    = 0UL;
	MDR_RST_CLK->USB_CLOCK     = 0UL;
	MDR_RST_CLK->ADC_MCO_CLOCK = 0UL;

	/* Initialize BKP */
	MDR_BKP->REG_0E &= ~( BKP_REG_0E_LOW_Msk | BKP_REG_0E_SELECTRI_Msk );
	MDR_BKP->REG_0E |=
	  ( BKP_REG_0E_LOW_Value << BKP_REG_0E_LOW_Pos )
	| ( BKP_REG_0E_LOW_Value << BKP_REG_0E_SELECTRI_Pos );

	/* Start external (HSE) oscillator */
	MDR_RST_CLK->HS_CONTROL =
	  ( 1UL << RST_CLK_HS_CONTROL_HSE_ON_Pos )
	| ( 0UL << RST_CLK_HS_CONTROL_HSE_BYP_Pos )
	| ( 1UL << RST_CLK_HS_CONTROL_HSE_ON2_Pos )
	| ( 0UL << RST_CLK_HS_CONTROL_HSE_BYP2_Pos );

	while (( MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_ALL_RDY ) == 0 ) __NOP();

	/* Switch CPU PLL to HSE oscillator */
	MDR_RST_CLK->CPU_CLOCK =
	  ( RST_CLK_CPU_CLOCK_CPU_C1_SEL_HSE    << RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos )
	| ( RST_CLK_CPU_CLOCK_CPU_C2_SEL_CPU_C1 << RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos )
	| ( RST_CLK_CPU_CLOCK_CPU_C3_SEL_CPU_C2 << RST_CLK_CPU_CLOCK_CPU_C3_SEL_Pos )
	| ( RST_CLK_CPU_CLOCK_HCLK_SEL_HSI      << RST_CLK_CPU_CLOCK_HCLK_SEL_Pos );

	/* Initialize CPU PLL */
	MDR_RST_CLK->PLL_CONTROL =
	  ( 0UL << RST_CLK_PLL_CONTROL_PLL_USB_ON_Pos )
	| ( 0UL << RST_CLK_PLL_CONTROL_PLL_USB_RLD_Pos )
	| ( 0UL << RST_CLK_PLL_CONTROL_PLL_CPU_ON_Pos )
	| ( 0UL << RST_CLK_PLL_CONTROL_PLL_CPU_PLD_Pos )
	| ( 0UL << RST_CLK_PLL_CONTROL_PLL_USB_MUL_Pos )
	| ( PLL_CPU_MUL_Value << RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos );

	/* Start CPU PLL */
	SET_BIT( MDR_RST_CLK->PLL_CONTROL, RST_CLK_PLL_CONTROL_PLL_CPU_ON_Pos );

	SET_BIT( MDR_RST_CLK->PLL_CONTROL, RST_CLK_PLL_CONTROL_PLL_CPU_PLD_Pos );
	CLR_BIT( MDR_RST_CLK->PLL_CONTROL, RST_CLK_PLL_CONTROL_PLL_CPU_PLD_Pos );

	while (( MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY ) == 0 ) __NOP();

	/* Switch CPU to CPU PLL */
	MDR_RST_CLK->CPU_CLOCK =
	  ( RST_CLK_CPU_CLOCK_CPU_C1_SEL_HSE     << RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos )
	| ( RST_CLK_CPU_CLOCK_CPU_C2_SEL_PLL_CPU << RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos )
	| ( RST_CLK_CPU_CLOCK_CPU_C3_SEL_CPU_C2  << RST_CLK_CPU_CLOCK_CPU_C3_SEL_Pos )
	| ( RST_CLK_CPU_CLOCK_HCLK_SEL_CPU_C3    << RST_CLK_CPU_CLOCK_HCLK_SEL_Pos );

	MDR_RST_CLK->PER_CLOCK |= RST_CLK_PCLK_EEPROM;
	MDR_EEPROM->CMD = ( EEPROM_CMD_DELAY_Value << EEPROM_CMD_DELAY_Pos );

  SystemCoreClockUpdate();
}

/** @} */ /* End of group __MDR1986VE1T_System_Private_Functions */

/** @} */ /* End of group __MDR1986VE1T */

/** @} */ /* End of group MDR1986VE1T */

/** @} */ /* End of group __CMSIS */

/******************* (C) COPYRIGHT 2013 Milandr *********************************
*
* END OF FILE system_MDR1986VE1T.c */
