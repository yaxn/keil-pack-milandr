/**
  ******************************************************************************
  * @file    Font_7x10_thin.c
  * @author  Milandr Application Team
  * @version V2.0.0
  * @date    10.09.2010
  * @brief   Font 7 x 10 pixels (thin). Analog of DOSApp-105 (7 x 12)
  *          Microsoft Windows (two bottom lines cut).
  ******************************************************************************
  * <br><br>
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, Milandr SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 Milandr</center></h2>
  */

#ifndef __CMCARM_DEMO__

/* Includes ------------------------------------------------------------------*/
#include "font_defs.h"

/** @addtogroup __MDR1986VE3_Eval_Demo MDR1986VE3 Demonstration Example
  * @{
  */

/** @addtogroup Fonts Fonts
  * @{
  */

/** @defgroup Font_7x10_thin Fonts 7x10 thin
  * @{
  */

/* The symbol representation has the following format:                      */
/* Every byte describes all columns of the symbol 8 upper lines.            */
/* Columns are represented from left to right.                              */
/* Lowest bit of a byte describes upper line of column,                     */
/* Highest - lower line.                                                    */
/* Then it's all repeated for all columns of lower 8 symbol lines.          */

static ucint8_t Font_7x10_thin_Data[] = {
  /* 0x00 - Space.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x01 - smiling face.*/
  0x78, 0xa4, 0x4a, 0x42, 0x4a, 0xa4, 0x78,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x02 - painted smiling face.*/
  0x78, 0xdc, 0xb6, 0xbe, 0xb6, 0xdc, 0x78,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x03 - hearts.*/
  0x1c, 0x3e, 0x7e, 0xfc, 0x7e, 0x3e, 0x1c,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x04 - diamonds.*/
  0x10, 0x38, 0x7c, 0xfe, 0x7c, 0x38, 0x10,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x05 - clubs.*/
  0x30, 0x78, 0x77, 0xbf, 0x77, 0x78, 0x30,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x06 - spades.*/
  0x38, 0x7c, 0x7e, 0xbf, 0x7e, 0x7c, 0x38,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x07 - filled circle at center.*/
  0x00, 0x70, 0xf8, 0xf8, 0xf8, 0x70, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x08 - inverted filled circle at center.*/
  0xff, 0x8f, 0x07, 0x07, 0x07, 0x8f, 0xff,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0x09 - unfilled circle at center.*/
  0x00, 0x70, 0x88, 0x88, 0x88, 0x70, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x0a - inverted unfilled circle at center (ring).*/
  0xff, 0x8f, 0x77, 0x77, 0x77, 0x8f, 0xff,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0x0b - male symbol (circle with pointer up).*/
  0xe0, 0x10, 0x10, 0x1a, 0xe6, 0x0e, 0x00,   0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,

  /* 0x0c - female symbol (circle with cross down).*/
  0x00, 0x4e, 0x51, 0xf1, 0x51, 0x4e, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x0d - note I.*/
  0x00, 0x80, 0x80, 0xfe, 0x04, 0x38, 0x00,   0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x0e - note II.*/
  0x80, 0x80, 0xfe, 0x0a, 0xc5, 0xc5, 0x7f,   0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x0f - sun (circle with outgoing rays).*/
  0x10, 0xba, 0x44, 0xc7, 0x44, 0xba, 0x10,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x10 - thick arrow right.*/
  0x00, 0x00, 0xfc, 0xf8, 0x70, 0x20, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x11 - thick arrow lefts.*/
  0x00, 0x20, 0x70, 0xf8, 0xfc, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

  /* 0x12 - thin arrow up-down.*/
  0x00, 0x44, 0xc6, 0xff, 0xc6, 0x44, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x13 - two exclamations.*/
  0x00, 0x00, 0xbf, 0x00, 0xbf, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x14 - "PI" symbol.*/
  0x00, 0x00, 0x1f, 0x11, 0xff, 0x01, 0xff,   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01,

  /* 0x15 - paragraph symbol.*/
  0x00, 0x96, 0x29, 0x29, 0x29, 0xd2, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x16 - thick underline.*/
  0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x17 - underlined thin arrow up-down.*/
  0x00, 0x24, 0x66, 0xff, 0x66, 0x24, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x18 - thin arrow up.*/
  0x00, 0x04, 0x06, 0xff, 0x06, 0x04, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x19 - thin arrow down.*/
  0x00, 0x40, 0xc0, 0xff, 0xc0, 0x40, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x1a - thin arrow right.*/
  0x20, 0x20, 0x20, 0xf8, 0x70, 0x20, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x1b - thin arrow left.*/
  0x20, 0x70, 0xf8, 0x20, 0x20, 0x20, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x1c - indentation symbol.*/
  0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x1d - thin arrow left-right.*/
  0x20, 0x70, 0xf8, 0x20, 0xf8, 0x70, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x1e - thick arrow up.*/
  0xc0, 0xf0, 0xfc, 0xff, 0xfc, 0xf0, 0xc0,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x1f - thick arrow down.*/
  0x06, 0x1e, 0x7e, 0xfe, 0x7e, 0x1e, 0x06,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x20 - space (empty place).*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x21 - excalmation.*/
  0x00, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x22 - double quote.*/
  0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x23 - number sign.*/
  0x00, 0x48, 0xfe, 0x48, 0xfe, 0x48, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x24 - dollar.*/
  0x00, 0x4c, 0x92, 0x93, 0x92, 0x64, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x25 - percent.*/
  0x00, 0x84, 0x4a, 0x24, 0x90, 0x48, 0x86,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x26 - ampersand.*/
  0x00, 0xf6, 0x09, 0x09, 0x09, 0xf6, 0x30,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01,

  /* 0x27 - apostrophe.*/
  0x00, 0x00, 0x00, 0x02, 0x06, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x28 - open bracket.*/
  0x00, 0x7c, 0x82, 0x01, 0x01, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

  /* 0x29 - close bracket.*/
  0x00, 0x01, 0x01, 0x82, 0x7c, 0x00, 0x00,   0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x2a - asterisk (multiplication).*/
  0x00, 0x54, 0x7c, 0x10, 0x7c, 0x54, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x2b - plus.*/
  0x00, 0x10, 0x10, 0x7c, 0x10, 0x10, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x2c - comma.*/
  0x00, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

  /* 0x2d - dash.*/
  0x00, 0x00, 0x20, 0x20, 0x20, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x2e - dot.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x2f - left-right slash ('/').*/
  0x00, 0x80, 0x60, 0x18, 0x06, 0x01, 0x00,   0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x30 - '0'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x31 - '1'.*/
  0x00, 0x04, 0x02, 0xff, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x32 - '2'.*/
  0x00, 0x86, 0x41, 0x21, 0x11, 0x0e, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x33 - '3'.*/
  0x00, 0x82, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x34 - '4'.*/
  0x00, 0x3e, 0x40, 0x40, 0x40, 0xfe, 0x40,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x35 - '5'.*/
  0x00, 0x8f, 0x09, 0x09, 0x09, 0xf1, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x36 - '6'.*/
  0x00, 0xfe, 0x11, 0x11, 0x11, 0xe2, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x37 - '7'.*/
  0x00, 0x01, 0x81, 0x61, 0x19, 0x07, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x38 - '8'.*/
  0x00, 0xee, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x39 - '9'.*/
  0x00, 0x0e, 0x11, 0x11, 0x91, 0x7e, 0x00,   0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,

  /* 0x3a - colon.*/
  0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x3b - semicolon.*/
  0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x3c - less.*/
  0x00, 0x10, 0x28, 0x44, 0x82, 0x01, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x3d - equal.*/
  0x00, 0x48, 0x48, 0x48, 0x48, 0x48, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x3e - greater.*/
  0x00, 0x01, 0x82, 0x44, 0x28, 0x10, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x3f - question-mark.*/
  0x00, 0x06, 0xa1, 0x11, 0x11, 0x0e, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x40 - "dog" ('@').*/
  0x00, 0x7c, 0x82, 0x01, 0x39, 0x45, 0x3e,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,

  /* 0x41 - 'A'.*/
  0x00, 0xfc, 0x22, 0x21, 0x22, 0xfc, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x42 - 'B'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x43 - 'C'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xc6, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x44 - 'D'.*/
  0x00, 0xff, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x45 - 'E'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0x01, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x46 - 'F'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0x01, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x47 - 'G'.*/
  0x00, 0xfe, 0x01, 0x01, 0x21, 0xe2, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x48 - 'H'.*/
  0x00, 0xff, 0x10, 0x10, 0x10, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x49 - 'I'.*/
  0x00, 0x00, 0x01, 0xff, 0x01, 0x00, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x4a - 'J'.*/
  0x00, 0xc0, 0x00, 0x00, 0x00, 0xff, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x4b - 'K'.*/
  0x00, 0xff, 0x10, 0x10, 0x28, 0xc7, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x4c - 'L'.*/
  0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x4d - 'M'.*/
  0x00, 0xff, 0x0c, 0x70, 0x70, 0x0c, 0xff,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01,

  /* 0x4e - 'N'.*/
  0x00, 0xff, 0x08, 0x10, 0x20, 0x40, 0xff,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01,

  /* 0x4f - 'O'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x50 - 'P'.*/
  0x00, 0xff, 0x11, 0x11, 0x11, 0x0e, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x51 - 'Q'.*/
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03,

  /* 0x52 - 'R'.*/
  0x00, 0xff, 0x11, 0x11, 0x31, 0xce, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x53 - 'S'.*/
  0x00, 0xce, 0x11, 0x11, 0x11, 0xe6, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x54 - 'T'.*/
  0x00, 0x01, 0x01, 0xff, 0x01, 0x01, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x55 - 'U'.*/
  0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x56 - 'V'.*/
  0x00, 0x7f, 0x80, 0x00, 0x80, 0x7f, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x57 - 'W'.*/
  0x00, 0x7f, 0x80, 0x7c, 0x80, 0x7f, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x58 - 'X'.*/
  0x00, 0xc7, 0x28, 0x10, 0x28, 0xc7, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x59 - 'Y'.*/
  0x00, 0x0f, 0x10, 0xe0, 0x10, 0x0f, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x5a - 'Z'.*/
  0x00, 0x81, 0x41, 0x39, 0x05, 0x03, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x5b - '['.*/
  0x00, 0x00, 0xff, 0x01, 0x01, 0x00, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x5c - '\'.*/
  0x00, 0x03, 0x0c, 0x30, 0xc0, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

  /* 0x5d - ']'.*/
  0x00, 0x00, 0x01, 0x01, 0xff, 0x00, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x5e - '^'.*/
  0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x5f - '_'.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00,

  /* 0x60 - back quote.*/
  0x00, 0x00, 0x07, 0x03, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x61 - 'a'.*/
  0x00, 0xc0, 0x28, 0x28, 0x28, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x62 - 'b'.*/
  0x00, 0xff, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x63 - 'c'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0x90, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x64 - 'd'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xff, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x65 - 'e'.*/
  0x00, 0xf0, 0x48, 0x48, 0x48, 0x70, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x66 - 'f'.*/
  0x00, 0x08, 0xfe, 0x09, 0x09, 0x01, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x67 - 'g'.*/
  0x00, 0x30, 0x48, 0x48, 0x48, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x68 - 'h'.*/
  0x00, 0xff, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x69 - 'i'.*/
  0x00, 0x00, 0x00, 0x09, 0xf9, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0x6a - 'j'.*/
  0x00, 0x80, 0x00, 0x0a, 0xfa, 0x00, 0x00,   0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,

  /* 0x6b - 'k'.*/
  0x00, 0xff, 0x20, 0x20, 0xd0, 0x08, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x6c - 'l'.*/
  0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0x6d - 'm'.*/
  0x00, 0xf8, 0x10, 0xe0, 0x10, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x6e - 'n'.*/
  0x00, 0xf8, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x6f - 'o'.*/
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x70 - 'p'.*/
  0x00, 0xf8, 0x48, 0x48, 0x48, 0x30, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x71 - 'q'.*/
  0x00, 0x30, 0x48, 0x48, 0x48, 0xf8, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x72 - 'r'.*/
  0x00, 0xf8, 0x10, 0x08, 0x08, 0x10, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x73 - 's'.*/
  0x00, 0x10, 0x28, 0x28, 0x28, 0xc8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x74 - 't'.*/
  0x00, 0x08, 0xfe, 0x08, 0x08, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0x75 - 'u'.*/
  0x00, 0xf8, 0x00, 0x00, 0x00, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x76 - 'v'.*/
  0x00, 0x78, 0x80, 0x00, 0x80, 0x78, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x77 - 'w'.*/
  0x00, 0xf8, 0x80, 0xf0, 0x80, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x78 - 'x'.*/
  0x00, 0x08, 0x90, 0x60, 0x90, 0x08, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0x79 - 'y'.*/
  0x00, 0x38, 0x40, 0x40, 0x40, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0x7a - 'z'.*/
  0x00, 0x88, 0x48, 0x48, 0x28, 0x18, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0x7b - '{'.*/
  0x00, 0x10, 0xee, 0x01, 0x01, 0x01, 0x00,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0x7c - '|'.*/
  0x00, 0x00, 0x00, 0xcf, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x7d - '}'.*/
  0x00, 0x01, 0x01, 0xee, 0x10, 0x00, 0x00,   0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,

  /* 0x7e - '~'.*/
  0x00, 0x06, 0x02, 0x07, 0x02, 0x03, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x7f - "house".*/
  0x00, 0xf0, 0x88, 0x84, 0x88, 0xf0, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x80 - net of points dispersed.*/
  0x44, 0x00, 0x11, 0x44, 0x00, 0x11, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00,

  /* 0x81 - net of points condensed.*/
  0xaa, 0x00, 0x55, 0xaa, 0x00, 0x55, 0x00,   0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00,

  /* 0x82 - net of lines.*/
  0xaa, 0x55, 0x55, 0xaa, 0x55, 0xaa, 0x55,   0x00, 0x01, 0x01, 0x00, 0x01, 0x00, 0x01,

  /* 0x83 - pseudo graphics - vertical line.*/
  0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x84 - pseudo graphics - vertical line with branch left from center.*/
  0x20, 0x20, 0x20, 0xff, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x85 - pseudo graphics - vertical line with double branch left from center.*/
  0x50, 0x50, 0x50, 0xff, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x86 - pseudo graphics - double vertical line with branch left from center.*/
  0x20, 0x20, 0xff, 0x00, 0xff, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x87 - pseudo graphics - upper right corner with double vertical line.*/
  0x20, 0x20, 0xe0, 0x20, 0xe0, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x88 - pseudo graphics - upper right corner with double horizontal line.*/
  0x50, 0x50, 0x50, 0xf0, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x89 - pseudo graphics - double vertical line with double branch left from center.*/
  0x50, 0x50, 0xdf, 0x00, 0xff, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x8a - pseudo graphics - double vertical line.*/
  0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x8b - pseudo graphics - double upper right corner.*/
  0x50, 0x50, 0xd0, 0x10, 0xf0, 0x00, 0x00,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x8c - pseudo graphics - double lower right corner.*/
  0x50, 0x50, 0x5f, 0x40, 0x7f, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x8d - pseudo graphics - lower right corner with double vertical line.*/
  0x20, 0x20, 0x3f, 0x20, 0x3f, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x8e - pseudo graphics - lower right corner with double horizontal line.*/
  0x50, 0x50, 0x50, 0x7f, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x8f - pseudo graphics - upper right corner.*/
  0x20, 0x20, 0x20, 0xe0, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x90 - pseudo graphics - lower left corner.*/
  0x00, 0x00, 0x00, 0x3f, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x91 - pseudo graphics - horizontal line with branch up from center.*/
  0x20, 0x20, 0x20, 0x3f, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x92 - pseudo graphics - horizontal line with branch down from center.*/
  0x20, 0x20, 0x20, 0xe0, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x93 - pseudo graphics - vertical line with branch right from center.*/
  0x00, 0x00, 0x00, 0xff, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x94 - pseudo graphics - horizontal line at the center.*/
  0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x95 - pseudo graphics - cross.*/
  0x20, 0x20, 0x20, 0xff, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x96 - pseudo graphics - vertical line with double branch right from center.*/
  0x00, 0x00, 0x00, 0xff, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0x97 - pseudo graphics - double vertical line with branch right from center.*/
  0x00, 0x00, 0xff, 0x00, 0xff, 0x20, 0x20,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x98 - pseudo graphics - double left lower corner.*/
  0x00, 0x00, 0x7f, 0x40, 0x5f, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x99 - pseudo graphics - double left upper corner.*/
  0x00, 0x00, 0xf0, 0x10, 0xd0, 0x50, 0x50,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x9a - pseudo graphics - double horizontal line with double branch up from center.*/
  0x50, 0x50, 0x5f, 0x40, 0x5f, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x9b - pseudo graphics - double horizontal line with double branch down from center.*/
  0x50, 0x50, 0xd0, 0x10, 0xd0, 0x50, 0x50,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x9c - pseudo graphics - double vertical line with double branch right from center.*/
  0x00, 0x00, 0xff, 0x00, 0xdf, 0x50, 0x50,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x9d - pseudo graphics - double horizontal line at the center.*/
  0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0x9e - pseudo graphics - double cross.*/
  0x50, 0x50, 0xdf, 0x00, 0xdf, 0x50, 0x50,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0x9f - pseudo graphics - double horizontal line with branch up from center.*/
  0x50, 0x50, 0x50, 0x5f, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xa0 - pseudo graphics - horizontal line with double branch up from center.*/
  0x20, 0x20, 0x3f, 0x20, 0x3f, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xa1 - pseudo graphics - double horizontal line with branch down from center.*/
  0x50, 0x50, 0x50, 0xd0, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xa2 - pseudo graphics - horizontal line with double branch down from center.*/
  0x20, 0x20, 0xe0, 0x20, 0xe0, 0x20, 0x20,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0xa3 - pseudo graphics - lower left corner, double vertical line.*/
  0x00, 0x00, 0x3f, 0x20, 0x3f, 0x20, 0x20,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xa4 - pseudo graphics - lower left corner, double horizontal line.*/
  0x00, 0x00, 0x00, 0x7f, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xa5 - pseudo graphics - upper left corner, double horizontal line.*/
  0x00, 0x00, 0x00, 0xf0, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xa6 - pseudo graphics - upper left corner, double vertical line.*/
  0x00, 0x00, 0xe0, 0x20, 0xe0, 0x20, 0x20,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0xa7 - pseudo graphics - double vertical line at the center with branches left and right.*/
  0x20, 0x20, 0xff, 0x00, 0xff, 0x20, 0x20,   0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,

  /* 0xa8 - Russian capital 'YO'.*/
  0x00, 0xfd, 0x25, 0x24, 0x25, 0x05, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xa9 - pseudo graphics - double horizontal line at the center with branches up and down.*/
  0x50, 0x50, 0x50, 0xdf, 0x50, 0x50, 0x50,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xaa - pseudo graphics - lower right corner.*/
  0x20, 0x20, 0x20, 0x3f, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xab - pseudo graphics - upper left corner.*/
  0x00, 0x00, 0x00, 0xe0, 0x20, 0x20, 0x20,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xac - pseudo graphics - filled place.*/
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xad - pseudo graphics - filled lower half.*/
  0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xae - pseudo graphics - filled left half.*/
  0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,   0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,

  /* 0xaf - pseudo graphics - filled right half.*/
  0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,   0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,

  /* 0xb0 - pseudo graphics - filled upper half.*/
  0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xb1 - hearts (not filled).*/
  0x00, 0xfe, 0x11, 0x11, 0x11, 0x82, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb2 - EX symbol.*/
  0x00, 0xf0, 0x28, 0x28, 0x08, 0x90, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb3 - unfilled EX symbol.*/
  0x00, 0x01, 0x01, 0xfc, 0x01, 0x01, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb4 - Special symbol: Russian "l".*/
  0x00, 0x02, 0x0a, 0x08, 0xfa, 0x02, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xb5 - Special symbol: Russian "l/ch".*/
/*  0x00, 0x9c, 0x21, 0x22, 0x21, 0xfc, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,*/

/* 0xb5 - Special symbol: Russian "l/ch".*/
0x00, 0x78, 0x84, 0x30, 0x30, 0x84, 0x78,    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xb6 - 'y' with upper tilde ('~').*/
  0x00, 0x38, 0x42, 0x44, 0x42, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb7 - small circle up.*/
  0x00, 0x0e, 0x11, 0x11, 0x0e, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xb8 - Russian low 'yo'.*/
  0x00, 0xf0, 0x2b, 0x28, 0x2b, 0xb0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xb9 - large filled circle at the center.*/
  0x00, 0x00, 0x08, 0x1c, 0x08, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xba - small filled circle at the center.*/
  0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xbb - square root symbol.*/
  0x00, 0x40, 0x80, 0x00, 0xfe, 0x02, 0x02,   0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,

  /* 0xbc - number sign.*/
  0xff, 0x04, 0x38, 0x40, 0xff, 0x19, 0x00,   0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,

  /* 0xbd - "sun".*/
  0x74, 0x88, 0x04, 0x04, 0x88, 0x74, 0x00,   0x01, 0x00, 0x01, 0x01, 0x00, 0x01, 0x00,

  /* 0xbe - filled square at the center.*/
  0x00, 0x00, 0x38, 0x38, 0x38, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xbf - empty place.*/
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xc0-0xdf - Russian capital letters.*/
  /* 0xc0 */
  0x00, 0xfc, 0x22, 0x21, 0x22, 0xfc, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xc1 */
  0x00, 0xff, 0x09, 0x09, 0x09, 0xf1, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xc2 */
  0x00, 0xff, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xc3 */
  0x00, 0xff, 0x01, 0x01, 0x01, 0x01, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xc4 */
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xff, 0x00,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xc5 */
  0x00, 0xff, 0x11, 0x11, 0x11, 0x01, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xc6 */
  0x00, 0xef, 0x10, 0xff, 0x10, 0xef, 0x00,   0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,

  /* 0xc7 */
  0x00, 0x82, 0x11, 0x11, 0x11, 0xee, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xc8 */
  0x00, 0xff, 0x40, 0x30, 0x08, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xc9 */
  0x00, 0xff, 0x40, 0x33, 0x08, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xca */
  0x00, 0xff, 0x10, 0x10, 0x28, 0xc7, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xcb */
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xcc */
  0x00, 0xff, 0x0c, 0x70, 0x0c, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xcd */
  0x00, 0xff, 0x10, 0x10, 0x10, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xce */
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xcf */
  0x00, 0xff, 0x01, 0x01, 0x01, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xd0 */
  0x00, 0xff, 0x11, 0x11, 0x11, 0x0e, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xd1 */
  0x00, 0xfe, 0x01, 0x01, 0x01, 0xc6, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xd2 */
  0x00, 0x01, 0x01, 0xff, 0x01, 0x01, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xd3 */
  0x00, 0x8f, 0x10, 0x10, 0x10, 0xff, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xd4 */
  0x00, 0x3c, 0x42, 0xff, 0x42, 0x3c, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xd5 */
  0x00, 0xc7, 0x28, 0x10, 0x28, 0xc7, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xd6 */
  0x00, 0xff, 0x00, 0x00, 0x00, 0xff, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xd7 */
  0x00, 0x0f, 0x10, 0x10, 0x10, 0xff, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xd8 */
  0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xd9 */
  0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xda */
  0x01, 0xff, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xdb */
  0x00, 0xff, 0x10, 0xe0, 0x00, 0xff, 0x00,   0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00,

  /* 0xdc */
  0x00, 0xff, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xdd */
  0x00, 0x82, 0x11, 0x11, 0x11, 0xfe, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xde */
  0x00, 0xff, 0x18, 0xff, 0x01, 0xff, 0x00,   0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0xdf */
  0x00, 0xee, 0x11, 0x11, 0x11, 0xff, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xe0-0xff - Russian low letters.*/
  /* 0xe0 */
  0x00, 0xc0, 0x28, 0x28, 0x28, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xe1 */
  0x00, 0xf8, 0x24, 0x24, 0x24, 0xc2, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xe2 */
  0x00, 0xf8, 0x28, 0x28, 0x28, 0xd0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xe3 */
  0x00, 0xf8, 0x08, 0x08, 0x08, 0x08, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xe4 */
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xf8, 0x00,   0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xe5 */
  0x00, 0xf0, 0x28, 0x28, 0x28, 0xb0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xe6 */
  0x00, 0xd8, 0x20, 0xf8, 0x20, 0xd8, 0x00,   0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,

  /* 0xe7 */
  0x00, 0x90, 0x08, 0x28, 0x28, 0xd0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xe8 */
  0x00, 0xf8, 0x80, 0x40, 0x20, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xe9 */
  0x00, 0xf8, 0x80, 0x44, 0x22, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xea */
  0x00, 0xf8, 0x20, 0x20, 0x50, 0x88, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xeb */
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xec */
  0x00, 0xf8, 0x10, 0x60, 0x10, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xed */
  0x00, 0xf8, 0x20, 0x20, 0x20, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xee */
  0x00, 0xf0, 0x08, 0x08, 0x08, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xef */
  0x00, 0xf8, 0x08, 0x08, 0x08, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xf0 */
  0x00, 0xf8, 0x48, 0x48, 0x48, 0x30, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,

  /* 0xf1 */
  0x00, 0xf0, 0x08, 0x08, 0x08, 0x90, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xf2 */
  0x00, 0x08, 0x08, 0xf8, 0x08, 0x08, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xf3 */
  0x00, 0x38, 0x40, 0x40, 0x40, 0xf8, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xf4 */
  0x00, 0x30, 0x48, 0xf8, 0x48, 0x30, 0x00,   0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,

  /* 0xf5 */
  0x00, 0x08, 0x90, 0x60, 0x90, 0x08, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xf6 */
  0x00, 0xf8, 0x00, 0x00, 0x00, 0xf8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xf7 */
  0x00, 0x38, 0x40, 0x40, 0x40, 0xf8, 0x00,   0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,

  /* 0xf8 */
  0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,

  /* 0xf9 */
  0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,

  /* 0xfa */
  0x08, 0xf8, 0x20, 0x20, 0x20, 0xc0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xfb */
  0x00, 0xf8, 0x20, 0xe0, 0x00, 0xf8, 0x00,   0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00,

  /* 0xfc */
  0x00, 0xf8, 0x20, 0x20, 0x20, 0xc0, 0x00,   0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xfd */
  0x00, 0x90, 0x08, 0x28, 0x28, 0xf0, 0x00,   0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00,

  /* 0xfe */
  0x00, 0xf8, 0x20, 0xf8, 0x08, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00,

  /* 0xff */
  0x00, 0xb0, 0x48, 0x48, 0x48, 0xf8, 0x00,   0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00
};

sFONT Font_7x10_thin = {
  10,                       /* Symbol height, in pixels.*/
  7,                        /* Symbol width, in pixels.*/
  255,                      /* Symbol number in the font.*/
  &Font_7x10_thin_Data[0]   /* Font description table address.*/
};

/** @} */ /* End of group Font_7x10_thin */

/** @} */ /* End of group Fonts */

/** @} */ /* End of group __MDR1986VE3_Eval_Demo */

#else /* __CMCARM_DEMO__ defined */

#pragma warn -180

#endif /* __CMCARM_DEMO__ */

/******************* (C) COPYRIGHT 2010 Milandr *********************************
*
* END OF FILE Font_7x10_thin.c */

