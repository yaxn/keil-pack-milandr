/* -----------------------------------------------------------------------------
 * Copyright (c) 2015 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * $Date:        6. September 2015
 * $Revision:    V1.00
 *
 * Project:      Port definitions for Milandr MDR1986VE9X
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */

#ifndef __MDR1986VE9X_PORT_H
#define __MDR1986VE9X_PORT_H

#include <MDR32Fx.h>

#define PORT_PIN_IN	0
#define PORT_PIN_OUT	1
#define PORT_PIN_OFF	0
#define PORT_PIN_ON	1

void ClkEnPort (MDR_PORT_TypeDef* Port);
void PortPinConfigure (MDR_PORT_TypeDef* Port, uint8_t Pin, uint8_t PinState);
void PortPinWrite (MDR_PORT_TypeDef* Port, uint8_t Pin, uint8_t PinState);

#endif /* __MDR1986VE9X_PORT_H */
