/*--------------------------------------------------------------------
Copyright 2020 fukuen

FFT demo is free software: you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This software is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with This software.  If not, see
<http://www.gnu.org/licenses/>.
--------------------------------------------------------------------*/

#include <Arduino.h>
#include <Sipeed_ST7789.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sysctl.h"
#include "plic.h"
#include "uarths.h"
#include "i2s.h"
#include "fpioa.h"
#include "utility/Button.h"
#include "gpio.h"
#include "fft.h"
#include "kendryte-standalone-sdk/lib/drivers/include/spi.h"

#define DEBOUNCE_MS 10
#ifdef PIN_KEY_UP // Maix-go
#define WIFI_EN_PIN 8
//#define AUDIO_PA_EN_PIN None  // Bit Dock and old MaixGo
#define AUDIO_PA_EN_PIN 32    // Maix Go(version 2.20)
Button BtnBOOT = Button(16, true, DEBOUNCE_MS);
//Button BtnBOOT = Button(KEY0, true, DEBOUNCE_MS);
//Button BtnA = Button(PIN_KEY_UP, true, DEBOUNCE_MS);
//Button BtnB = Button(PIN_KEY_PRESS, true, DEBOUNCE_MS);
//Button BtnC = Button(PIN_KEY_DOWN, true, DEBOUNCE_MS);
#else // Maixduino
#define WIFI_EN_PIN 8
#define AUDIO_PA_EN_PIN 2
Button BtnBOOT = Button(KEY0, true, DEBOUNCE_MS);
//Button BtnA = Button(7, true, DEBOUNCE_MS);
//Button BtnB = Button(8, true, DEBOUNCE_MS);
//Button BtnC = Button(9, true, DEBOUNCE_MS);
#endif

// common
#define WIDTH 320
#define HEIGHT 240
#define FFT_N              512U
#define FRAME_LEN          FFT_N
#define SAMPLING_FREQUENCY 16000

int i, j;
uint16_t rx_buf[FRAME_LEN * 2];
uint32_t g_rx_dma_buf[FRAME_LEN * 2 * 2];

volatile uint32_t g_index;
volatile uint8_t i2s_rec_flag;
volatile uint8_t i2s_start_flag = 0;

typedef enum _disp_mode
{
    MODE_FFT,
    MODE_SPECTRUM,
    MODE_WAVE
} disp_mode_t;

disp_mode_t mode = MODE_FFT;

// fft
#define FFT_FORWARD_SHIFT   0x0U
#define FFT_BACKWARD_SHIFT  0x1ffU

typedef enum _complex_mode
{
    FFT_HARD = 0,
    FFT_SOFT = 1,
    FFT_COMPLEX_MAX,
} complex_mode_t;

fft_data_t fft_in_data[FFT_N];
fft_data_t fft_out_data[FFT_N];
complex_hard_t data_hard[FFT_N] = {0};
float hard_power[FFT_N];

// draw wave
int xpos = 0;
int multiply = 10;
//#define XOFFSET 10
#define DRAWWIDTH 290

// draw fft
int X0 = 30;
int Y0 = 20;
int _height = 240 - Y0;
int _width = 320;
float dmax = 5.0;
//uint16_t g_lcd_gram[38400] __attribute__((aligned(64)));
uint16_t g_lcd_gram[WIDTH * HEIGHT] __attribute__((aligned(64)));
uint16_t g_lcd_gram_old[WIDTH * HEIGHT] __attribute__((aligned(64)));

// draw spectrogram
//#define SPECTROGRAM_LENGTH (WIDTH-20)
#define SPECTROGRAM_LENGTH (HEIGHT-20)
#define SPECTROGRAM_HEIGHT FFT_N/2//LCD_Y_MAX//FFT_N //(FFT_N/2 + 50)

#define MIN_DB -50
#define MAX_DB  30

/*static uint16_t gray2rgb565[64]={
0x0000, 0x2000, 0x4108, 0x6108, 0x8210, 0xa210, 0xc318, 0xe318, 
0x0421, 0x2421, 0x4529, 0x6529, 0x8631, 0xa631, 0xc739, 0xe739, 
0x0842, 0x2842, 0x494a, 0x694a, 0x8a52, 0xaa52, 0xcb5a, 0xeb5a, 
0x0c63, 0x2c63, 0x4d6b, 0x6d6b, 0x8e73, 0xae73, 0xcf7b, 0xef7b, 
0x1084, 0x3084, 0x518c, 0x718c, 0x9294, 0xb294, 0xd39c, 0xf39c, 
0x14a5, 0x34a5, 0x55ad, 0x75ad, 0x96b5, 0xb6b5, 0xd7bd, 0xf7bd, 
0x18c6, 0x38c6, 0x59ce, 0x79ce, 0x9ad6, 0xbad6, 0xdbde, 0xfbde, 
0x1ce7, 0x3ce7, 0x5def, 0x7def, 0x9ef7, 0xbef7, 0xdfff, 0xffff,
};*/


#define COLOR_LUT_SIZE 1024
static const uint16_t jetMap[COLOR_LUT_SIZE]={
0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0010, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011, 0x0011,
0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013,
0x0013, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015,
0x0015, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017,
0x0017, 0x0017, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019,
0x0019, 0x0019, 0x001A, 0x001A, 0x001A, 0x001A, 0x001A, 0x001A, 0x001A, 0x001A, 0x001B, 0x001B, 0x001B, 0x001B, 0x001B, 0x001B,
0x001B, 0x001B, 0x001B, 0x001C, 0x001C, 0x001C, 0x001C, 0x001C, 0x001C, 0x001C, 0x001C, 0x001D, 0x001D, 0x001D, 0x001D, 0x001D,
0x001D, 0x001D, 0x001D, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E, 0x001E, 0x001F, 0x001F, 0x001F, 0x001F, 0x001F,
0x001F, 0x001F, 0x003F, 0x003F, 0x003F, 0x003F, 0x005F, 0x005F, 0x005F, 0x005F, 0x007F, 0x007F, 0x007F, 0x007F, 0x009F, 0x009F,
0x009F, 0x009F, 0x00BF, 0x00BF, 0x00BF, 0x00BF, 0x00DF, 0x00DF, 0x00DF, 0x00DF, 0x00FF, 0x00FF, 0x00FF, 0x00FF, 0x011F, 0x011F,
0x011F, 0x011F, 0x013F, 0x013F, 0x013F, 0x013F, 0x015F, 0x015F, 0x015F, 0x015F, 0x017F, 0x017F, 0x017F, 0x017F, 0x019F, 0x019F,
0x019F, 0x019F, 0x01BF, 0x01BF, 0x01BF, 0x01BF, 0x01DF, 0x01DF, 0x01DF, 0x01DF, 0x01FF, 0x01FF, 0x01FF, 0x01FF, 0x021F, 0x021F,
0x021F, 0x021F, 0x021F, 0x023F, 0x023F, 0x023F, 0x023F, 0x025F, 0x025F, 0x025F, 0x025F, 0x027F, 0x027F, 0x027F, 0x027F, 0x029F,
0x029F, 0x029F, 0x029F, 0x02BF, 0x02BF, 0x02BF, 0x02BF, 0x02DF, 0x02DF, 0x02DF, 0x02DF, 0x02FF, 0x02FF, 0x02FF, 0x02FF, 0x031F,
0x031F, 0x031F, 0x031F, 0x033F, 0x033F, 0x033F, 0x033F, 0x035F, 0x035F, 0x035F, 0x035F, 0x037F, 0x037F, 0x037F, 0x037F, 0x039F,
0x039F, 0x039F, 0x039F, 0x03BF, 0x03BF, 0x03BF, 0x03BF, 0x03DF, 0x03DF, 0x03DF, 0x03DF, 0x03FF, 0x03FF, 0x03FF, 0x03FF, 0x041F,
0x041F, 0x041F, 0x041F, 0x041F, 0x043F, 0x043F, 0x043F, 0x043F, 0x045F, 0x045F, 0x045F, 0x045F, 0x047F, 0x047F, 0x047F, 0x047F,
0x049F, 0x049F, 0x049F, 0x049F, 0x04BF, 0x04BF, 0x04BF, 0x04BF, 0x04DF, 0x04DF, 0x04DF, 0x04DF, 0x04FF, 0x04FF, 0x04FF, 0x04FF,
0x051F, 0x051F, 0x051F, 0x051F, 0x053F, 0x053F, 0x053F, 0x053F, 0x055F, 0x055F, 0x055F, 0x055F, 0x057F, 0x057F, 0x057F, 0x057F,
0x059F, 0x059F, 0x059F, 0x059F, 0x05BF, 0x05BF, 0x05BF, 0x05BF, 0x05DF, 0x05DF, 0x05DF, 0x05DF, 0x05FF, 0x05FF, 0x05FF, 0x05FF,
0x05FF, 0x061F, 0x061F, 0x061F, 0x061F, 0x063F, 0x063F, 0x063F, 0x063F, 0x065F, 0x065F, 0x065F, 0x065F, 0x067F, 0x067F, 0x067F,
0x067F, 0x069F, 0x069F, 0x069F, 0x069F, 0x06BF, 0x06BF, 0x06BF, 0x06BF, 0x06DF, 0x06DF, 0x06DF, 0x06DF, 0x06FF, 0x06FF, 0x06FF,
0x06FF, 0x071F, 0x071F, 0x071F, 0x071F, 0x073F, 0x073F, 0x073F, 0x073F, 0x075F, 0x075F, 0x075F, 0x075F, 0x077F, 0x077F, 0x077F,
0x077F, 0x079F, 0x079F, 0x079F, 0x079F, 0x07BF, 0x07BF, 0x07BF, 0x07BF, 0x07DF, 0x07DF, 0x07DF, 0x07DF, 0x07FF, 0x07FF, 0x07FF,
0x07FF, 0x07FF, 0x07FF, 0x07FF, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x0FFE, 0x17FD, 0x17FD, 0x17FD, 0x17FD,
0x17FD, 0x17FD, 0x17FD, 0x17FD, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x1FFC, 0x27FB, 0x27FB, 0x27FB, 0x27FB,
0x27FB, 0x27FB, 0x27FB, 0x27FB, 0x27FB, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x2FFA, 0x37F9, 0x37F9, 0x37F9,
0x37F9, 0x37F9, 0x37F9, 0x37F9, 0x37F9, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x3FF8, 0x47F7, 0x47F7, 0x47F7,
0x47F7, 0x47F7, 0x47F7, 0x47F7, 0x47F7, 0x47F7, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x4FF6, 0x57F5, 0x57F5,
0x57F5, 0x57F5, 0x57F5, 0x57F5, 0x57F5, 0x57F5, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x5FF4, 0x67F3, 0x67F3,
0x67F3, 0x67F3, 0x67F3, 0x67F3, 0x67F3, 0x67F3, 0x67F3, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x6FF2, 0x77F1,
0x77F1, 0x77F1, 0x77F1, 0x77F1, 0x77F1, 0x77F1, 0x77F1, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x7FF0, 0x87F0,
0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x87EF, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE, 0x8FEE,
0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x97ED, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC, 0x9FEC,
0x9FEC, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xA7EB, 0xAFEA, 0xAFEA, 0xAFEA, 0xAFEA, 0xAFEA, 0xAFEA, 0xAFEA,
0xAFEA, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xB7E9, 0xBFE8, 0xBFE8, 0xBFE8, 0xBFE8, 0xBFE8, 0xBFE8, 0xBFE8,
0xBFE8, 0xBFE8, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xC7E7, 0xCFE6, 0xCFE6, 0xCFE6, 0xCFE6, 0xCFE6, 0xCFE6,
0xCFE6, 0xCFE6, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xD7E5, 0xDFE4, 0xDFE4, 0xDFE4, 0xDFE4, 0xDFE4, 0xDFE4,
0xDFE4, 0xDFE4, 0xDFE4, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xE7E3, 0xEFE2, 0xEFE2, 0xEFE2, 0xEFE2, 0xEFE2,
0xEFE2, 0xEFE2, 0xEFE2, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xF7E1, 0xFFE0, 0xFFE0, 0xFFE0, 0xFFE0, 0xFFE0,
0xFFE0, 0xFFE0, 0xFFC0, 0xFFC0, 0xFFC0, 0xFFC0, 0xFFA0, 0xFFA0, 0xFFA0, 0xFFA0, 0xFF80, 0xFF80, 0xFF80, 0xFF80, 0xFF60, 0xFF60,
0xFF60, 0xFF60, 0xFF40, 0xFF40, 0xFF40, 0xFF40, 0xFF20, 0xFF20, 0xFF20, 0xFF20, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFEE0, 0xFEE0,
0xFEE0, 0xFEE0, 0xFEC0, 0xFEC0, 0xFEC0, 0xFEC0, 0xFEA0, 0xFEA0, 0xFEA0, 0xFEA0, 0xFE80, 0xFE80, 0xFE80, 0xFE80, 0xFE60, 0xFE60,
0xFE60, 0xFE60, 0xFE40, 0xFE40, 0xFE40, 0xFE40, 0xFE20, 0xFE20, 0xFE20, 0xFE20, 0xFE00, 0xFE00, 0xFE00, 0xFE00, 0xFDE0, 0xFDE0,
0xFDE0, 0xFDE0, 0xFDE0, 0xFDC0, 0xFDC0, 0xFDC0, 0xFDC0, 0xFDA0, 0xFDA0, 0xFDA0, 0xFDA0, 0xFD80, 0xFD80, 0xFD80, 0xFD80, 0xFD60,
0xFD60, 0xFD60, 0xFD60, 0xFD40, 0xFD40, 0xFD40, 0xFD40, 0xFD20, 0xFD20, 0xFD20, 0xFD20, 0xFD00, 0xFD00, 0xFD00, 0xFD00, 0xFCE0,
0xFCE0, 0xFCE0, 0xFCE0, 0xFCC0, 0xFCC0, 0xFCC0, 0xFCC0, 0xFCA0, 0xFCA0, 0xFCA0, 0xFCA0, 0xFC80, 0xFC80, 0xFC80, 0xFC80, 0xFC60,
0xFC60, 0xFC60, 0xFC60, 0xFC40, 0xFC40, 0xFC40, 0xFC40, 0xFC20, 0xFC20, 0xFC20, 0xFC20, 0xFC00, 0xFC00, 0xFC00, 0xFC00, 0xFC00,
0xFBE0, 0xFBE0, 0xFBE0, 0xFBE0, 0xFBC0, 0xFBC0, 0xFBC0, 0xFBC0, 0xFBA0, 0xFBA0, 0xFBA0, 0xFBA0, 0xFB80, 0xFB80, 0xFB80, 0xFB80,
0xFB60, 0xFB60, 0xFB60, 0xFB60, 0xFB40, 0xFB40, 0xFB40, 0xFB40, 0xFB20, 0xFB20, 0xFB20, 0xFB20, 0xFB00, 0xFB00, 0xFB00, 0xFB00,
0xFAE0, 0xFAE0, 0xFAE0, 0xFAE0, 0xFAC0, 0xFAC0, 0xFAC0, 0xFAC0, 0xFAA0, 0xFAA0, 0xFAA0, 0xFAA0, 0xFA80, 0xFA80, 0xFA80, 0xFA80,
0xFA60, 0xFA60, 0xFA60, 0xFA60, 0xFA40, 0xFA40, 0xFA40, 0xFA40, 0xFA20, 0xFA20, 0xFA20, 0xFA20, 0xFA00, 0xFA00, 0xFA00, 0xFA00,
0xFA00, 0xF9E0, 0xF9E0, 0xF9E0, 0xF9E0, 0xF9C0, 0xF9C0, 0xF9C0, 0xF9C0, 0xF9A0, 0xF9A0, 0xF9A0, 0xF9A0, 0xF980, 0xF980, 0xF980,
0xF980, 0xF960, 0xF960, 0xF960, 0xF960, 0xF940, 0xF940, 0xF940, 0xF940, 0xF920, 0xF920, 0xF920, 0xF920, 0xF900, 0xF900, 0xF900,
0xF900, 0xF8E0, 0xF8E0, 0xF8E0, 0xF8E0, 0xF8C0, 0xF8C0, 0xF8C0, 0xF8C0, 0xF8A0, 0xF8A0, 0xF8A0, 0xF8A0, 0xF880, 0xF880, 0xF880,
0xF880, 0xF860, 0xF860, 0xF860, 0xF860, 0xF840, 0xF840, 0xF840, 0xF840, 0xF820, 0xF820, 0xF820, 0xF820, 0xF800, 0xF800, 0xF800,
0xF800, 0xF800, 0xF800, 0xF800, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xE800, 0xE800, 0xE800, 0xE800,
0xE800, 0xE800, 0xE800, 0xE800, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xE000, 0xD800, 0xD800, 0xD800, 0xD800,
0xD800, 0xD800, 0xD800, 0xD800, 0xD800, 0xD000, 0xD000, 0xD000, 0xD000, 0xD000, 0xD000, 0xD000, 0xD000, 0xC800, 0xC800, 0xC800,
0xC800, 0xC800, 0xC800, 0xC800, 0xC800, 0xC000, 0xC000, 0xC000, 0xC000, 0xC000, 0xC000, 0xC000, 0xC000, 0xB800, 0xB800, 0xB800,
0xB800, 0xB800, 0xB800, 0xB800, 0xB800, 0xB800, 0xB000, 0xB000, 0xB000, 0xB000, 0xB000, 0xB000, 0xB000, 0xB000, 0xA800, 0xA800,
0xA800, 0xA800, 0xA800, 0xA800, 0xA800, 0xA800, 0xA000, 0xA000, 0xA000, 0xA000, 0xA000, 0xA000, 0xA000, 0xA000, 0x9800, 0x9800,
0x9800, 0x9800, 0x9800, 0x9800, 0x9800, 0x9800, 0x9800, 0x9000, 0x9000, 0x9000, 0x9000, 0x9000, 0x9000, 0x9000, 0x9000, 0x8800,
0x8800, 0x8800, 0x8800, 0x8800, 0x8800, 0x8800, 0x8800, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000
};

float pmax = 0 , pmin = 0;

uint16_t hann[FFT_N];
//#define FPS_MODULO ((uint32_t) ((SAMPLE_RATE/FFT_N))/25) 
#define FPS_MODULO 0
uint16_t fps_counter;

SPIClass spi_(SPI0); // MUST be SPI0 for Maix series on board LCD
Sipeed_ST7789 lcd(320, 240, spi_);

int i2s_dma_irq(void *ctx)
{
    uint32_t i;
    if (i2s_start_flag)
    {
        int16_t s_tmp;
        if (g_index)
        {
            i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[g_index], FRAME_LEN * 2, DMAC_CHANNEL0);
            g_index = 0;
            for (i = 0; i < FRAME_LEN; i++)
            {
                s_tmp = (int16_t)(g_rx_dma_buf[2 * i] & 0xffff); //g_rx_dma_buf[2 * i + 1] Low left
                rx_buf[i] = s_tmp + 32768;
            }
            i2s_rec_flag = 1;
        }
        else
        {
            i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);
            g_index = FRAME_LEN * 2;
            for (i = FRAME_LEN; i < FRAME_LEN * 2; i++)
            {
                s_tmp = (int16_t)(g_rx_dma_buf[2 * i] & 0xffff);//g_rx_dma_buf[2 * i + 1] Low left
                rx_buf[i] = s_tmp + 32768;
            }
            i2s_rec_flag = 2;
        }
    }
    else
    {
        i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);
        g_index = FRAME_LEN * 2;
    }
    return 0;
}

void clear_led() {
    // The SK9822 has an overal brightness factor
    uint8_t brightness = 0;

    // The buffer to send the data to the mic array
    #define NUMPIX 12
    #define TX_LEN 4+NUMPIX*4+4
    uint8_t tx_buffer[TX_LEN];

    // data pin
    fpioa_set_function(MIC_LED_DAT, FUNC_SPI1_D0); //MOSI
    // clock pin
    fpioa_set_function(SPI0_CS1, FUNC_SPI1_SCLK); //CLK
    // init SPI
    spi_init(SPI_DEVICE_1, SPI_WORK_MODE_0, SPI_FF_STANDARD, 8, 0);

    // reset LED
    uint8_t tx_place=0;

    // the SK9822 first expects 4 times 0000 0000 
    for (i=0; i<4; i++) tx_buffer[tx_place+i] = 0; 
    tx_place+=4; 
    
    // write the color data to the tx_buffer
    for (i=0; i<NUMPIX; i++) {
        // set the overall brightness factor
        tx_buffer[tx_place] = (0b11100000 | brightness);
        // set the RGB values
        tx_buffer[tx_place+1] = 0; // R
        tx_buffer[tx_place+2] = 0; // G
        tx_buffer[tx_place+3] = 0; // B
        tx_place += 4;
    }

    // the SK9822 finally expects 4 times 1111 1111
    for (int i=0; i<4; i++) tx_buffer[tx_place+i] = 255; 

    // send the data to the mic array
    spi_send_data_standard(SPI_DEVICE_1, SPI_CHIP_SELECT_0, NULL, 0, tx_buffer, TX_LEN);

    // pin off
    fpioa_set_function(MIC_LED_DAT, FUNC_GPIO0); //MOSI
    gpio_set_drive_mode(0, GPIO_DM_OUTPUT);
    gpio_set_pin(0, GPIO_PV_HIGH);
    fpioa_set_function(SPI0_CS1, FUNC_GPIO1); //CLK
    gpio_set_drive_mode(1, GPIO_DM_OUTPUT);
    gpio_set_pin(1, GPIO_PV_HIGH);
}

int pio_init() {
    fpioa_set_function(MIC_DAT3, FUNC_I2S0_IN_D0);
    fpioa_set_function(MIC_WS, FUNC_I2S0_WS);
    fpioa_set_function(MIC_BCK, FUNC_I2S0_SCLK);

    clear_led();

    //i2s init
    i2s_init(I2S_DEVICE_0, I2S_RECEIVER, 0x03);

    i2s_rx_channel_config(I2S_DEVICE_0, I2S_CHANNEL_0,
            RESOLUTION_16_BIT, SCLK_CYCLES_32,
            TRIGGER_LEVEL_4, STANDARD_MODE);

    uint32_t ret = i2s_set_sample_rate(I2S_DEVICE_0, SAMPLING_FREQUENCY);
    printf("actual rate %ul\n", ret);

//    plic_init();
    dmac_init();
    dmac_set_irq(DMAC_CHANNEL0, i2s_dma_irq, NULL, 1);
    i2s_receive_data_dma(I2S_DEVICE_0, &g_rx_dma_buf[0], FRAME_LEN * 2, DMAC_CHANNEL0);

    /* Enable the machine interrupt */
    sysctl_enable_irq();
    return 0;
}

/** wave **/
void drawWave(int offset)
{
    xpos = 0;
    for (i = 1; i < FRAME_LEN; i++)
    {
        xpos++;
        if (xpos >= DRAWWIDTH)
        {
            xpos = 0;
        }
        else
        {
            int y0 = (rx_buf[i - 1 + offset] - 32768) * HEIGHT * multiply / 32768 + HEIGHT / 2;
            int y1 = (rx_buf[i + offset] - 32768) * HEIGHT * multiply / 32768 + HEIGHT / 2;
            if (y0 >= HEIGHT) y0 = HEIGHT - 1;
            if (y0 < 0) y0 = 0;
            if (y1 >= HEIGHT) y1 = HEIGHT - 1;
            if (y1 < 0) y1 = 0;
            lcd.drawFastVLine(xpos + X0, 0, HEIGHT, COLOR_BLACK);
            lcd.drawLine(xpos - 1 + X0, y0, xpos + X0, y1, COLOR_WHITE);
        }
    }
}

void FFT(int offset)
{
    for (i = 0; i < FFT_N / 2; i++)
    {
        fft_in_data[i].I1 = 0;
        fft_in_data[i].R1 = rx_buf[2 * i + offset] - 32768;
        fft_in_data[i].I2 = 0;
        fft_in_data[i].R2 = rx_buf[2 + i + 1 + offset] - 32768;
    }
    fft_complex_uint16_dma(DMAC_CHANNEL1, DMAC_CHANNEL2, FFT_FORWARD_SHIFT, FFT_DIR_FORWARD, (uint64_t *)fft_in_data, FFT_N, (uint64_t *)fft_out_data);
    for (i = 0; i < FFT_N / 2; i++)
    {
        data_hard[2 * i].imag = fft_out_data[i].I1;
        data_hard[2 * i].real = fft_out_data[i].R1;
        data_hard[2 * i + 1].imag = fft_out_data[i].I2;
        data_hard[2 * i + 1].real = fft_out_data[i].R2;
    }
}

/** fft **/
void drawChartFft(int nsamples, float pw_max)
{
    int band_width = 1;
    int band_pad = band_width - 1;

    lcd.fillRect(0, 0, WIDTH, HEIGHT, COLOR_BLACK);
    for (int band = 0; band < nsamples; band++)
    {
        int hpos = band * band_width + X0;
//        float d = vReal[band];
//        if (d > dmax) d = dmax;
//        int h = (int)((d / dmax) * (_height));

//        int h = (uint16_t)data_hard[band].real * _height / 65536;
        int h = (int)((hard_power[band] / pw_max) * (_height));
        if (h > _height) h = _height;
        if (h < 0) h = 0;
//        lcd.fillRect(hpos, _height - h, band_width, h, COLOR_WHITE);
        lcd.drawLine(hpos, _height - h, hpos, _height, COLOR_WHITE);
        if ((band % (nsamples / 4)) == 0)
        {
            lcd.setCursor(hpos, _height + Y0 - 10);
            lcd.printf("%.1fkHz", ((band * 1.0 * SAMPLING_FREQUENCY) / FFT_N / 1000));
        }
    }
}


#define SWAP_16(x) ((x >> 8 & 0xff) | (x << 8))

void update_image_fft(float* hard_power, float pw_max, uint32_t* pImage, uint32_t color, uint32_t bkg_color)
{
    uint32_t bcolor= SWAP_16((bkg_color << 16)) | SWAP_16(bkg_color);
    uint32_t fcolor= SWAP_16((color << 16)) | SWAP_16(color);

    int  h[80];

    int x = 0;

    for (i = 0; i < 80; i++)
    {
        h[i]=120*(hard_power[i])/pw_max;

        if (h[i]>120)
            h[i] = 120;
        if (h[i]<0)
            h[i] = 0;
    }

    for (i = 0; i < 80; i++)  // 53* 38640/512 => ~4000Hz
    {
        x=i*2;
        for( int y=0; y<120; y++)
        {
            if( y<(120 - h[i+2]) )
            {
                pImage[x+y*2*160]=bcolor;
                pImage[x+1+y*2*160]=bcolor;
                pImage[x+(y*2+1)*160]=bcolor;
                pImage[x+1+(y*2+1)*160]=bcolor;
            }
            else
            {
                pImage[x+y*2*160]=fcolor;
                pImage[x+1+y*2*160]=fcolor;
                pImage[x+(y*2+1)*160]=bcolor;
                pImage[x+1+(y*2+1)*160]=bcolor;
            }
        }
    }
}

void drawFft(int offset) {
    FFT(offset);
/*
    for (i = 0; i < FFT_N / 2; i++)
    {
        fft_in_data[i].I1 = 0;
        fft_in_data[i].R1 = rx_buf[2 * i + offset] - 32768;
        fft_in_data[i].I2 = 0;
        fft_in_data[i].R2 = rx_buf[2 + i + 1 + offset] - 32768;
    }
    fft_complex_uint16_dma(DMAC_CHANNEL1, DMAC_CHANNEL2, FFT_FORWARD_SHIFT, FFT_DIR_FORWARD, (uint64_t *)fft_in_data, FFT_N, (uint64_t *)fft_out_data);
    for (i = 0; i < FFT_N / 2; i++)
    {
        data_hard[2 * i].imag = fft_out_data[i].I1;
        data_hard[2 * i].real = fft_out_data[i].R1;
        data_hard[2 * i + 1].imag = fft_out_data[i].I2;
        data_hard[2 * i + 1].real = fft_out_data[i].R2;
    }
*/

    float pmax=10;
    for (i = 0; i < FFT_N / 2; i++)
    {
        hard_power[i] = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag);

        //Convert to dBFS
        hard_power[i] = 20*log(2*hard_power[i]/FFT_N);

        if( hard_power[i]>pmax)
            pmax = hard_power[i];
    }
    update_image_fft(hard_power, 140 /*MAX range dBFS*/, (uint32_t *)g_lcd_gram, COLOR_BLUE, COLOR_BLACK);
    lcd.drawImage(0, 0, WIDTH, HEIGHT, (uint16_t *)g_lcd_gram);
}

/** spectrogram **/
uint16_t get_bit1_num(uint32_t data)
{
    uint16_t num;
    for (num = 0; data; num++)
        data &= data - 1;
    return num;
}

uint16_t color_scale(const float value)
{
    /*uint16_t color_code ;
    if(value > 1) value = 1 ;
    if(value < 0) value = 0 ;
    int aR = 0;   int aG = 0; int aB=31;  // RGB for our 1st color (blue in this case).
    int bR = 31; int bG = 0; int bB=0;    // RGB for our 2nd color (red in this case).

    float red   = (float)(bR - aR) * value + aR;      // Evaluated as -255*value + 255.
    float green = (float)(bG - aG) * value + aG;      // Evaluates as 0.
    float blue  = (float)(bB - aB) * value + aB;      // Evaluates as 255*value + 0.

    uint16_t rs = ((uint16_t) red)  ; //5bits only
    uint16_t gs = ((uint16_t) green)  ; //6bits
    uint16_t bs = ((uint16_t) blue)  ;
    color_code = (rs << 11) | (gs << 5)| bs ;
    */
    int index = round(value * (COLOR_LUT_SIZE-1)) ;
    if (index < 0) index = 0;
    if (index > (COLOR_LUT_SIZE-1)) index = (COLOR_LUT_SIZE-1);
    return jetMap[index];
    //return color_code ;
}

void hann_init()
{
	for (int i = 0; i < FFT_N; i++)
    {
   	    float multiplier = 0.5f * (1.f - cos(2.f*PI*i/(FFT_N-1))) * 65535.f;// FP Q1.16
        hann[i] = floor(multiplier) ;
	}
}

void generate_sinewave_stereo(uint32_t freq, int32_t * buffer, uint32_t nb_samples){
	for (i = 0 ; i < nb_samples ; i ++)
    {
		int32_t temp = 2048*cosf(2*PI*freq*(i*1./SAMPLING_FREQUENCY));
		buffer[i*2] = temp ;
		buffer[i*2+1] = temp;
	}
}

void initSpectrogram()
{
    fps_counter = 0 ;
    hann_init();
    pmax = MAX_DB ;
    pmin = MIN_DB ;
    lcd.fillRect(0, 0, WIDTH, HEIGHT, COLOR_WHITE);
    lcd.setTextColor(COLOR_BLACK);
    lcd.setCursor(SPECTROGRAM_HEIGHT + 10 , 120);
    lcd.printf("STFT");
    lcd.setCursor(0 , SPECTROGRAM_LENGTH+5);
    lcd.printf("%u",0);
    lcd.setCursor(SPECTROGRAM_HEIGHT/4-5 , SPECTROGRAM_LENGTH+5);
    lcd.printf("%u",SAMPLING_FREQUENCY/8000);
    lcd.setCursor(SPECTROGRAM_HEIGHT/2-5 , SPECTROGRAM_LENGTH+5);
    lcd.printf("%u",SAMPLING_FREQUENCY/4000);
    lcd.setCursor(3*(SPECTROGRAM_HEIGHT/4)-5 , SPECTROGRAM_LENGTH+5);
    lcd.printf("%u",3*SAMPLING_FREQUENCY/8000);
    lcd.setCursor(SPECTROGRAM_HEIGHT-5 , SPECTROGRAM_LENGTH+5);
    lcd.printf("%u",SAMPLING_FREQUENCY/2000);
}

uint16_t dbToColor(float db, const float max, const float min)
{
	db = (db - min)/(max - min);
	if (db < 0) db = 0;
	if (db > 1.0) db = 1.0;
	return color_scale(db);
}

void update_image_spectrogram( float* hard_power, float pw_max, float pw_min, uint16_t* pImage)
{
    for (j = 0 ; j < SPECTROGRAM_LENGTH ; j ++)
    {
        for (i = 0 ; i < SPECTROGRAM_HEIGHT ; i ++)
        {
            float v = (hard_power[i+(j*SPECTROGRAM_HEIGHT)] - pw_min)/(pw_max - pw_min);
            pImage[i+(j*HEIGHT)] = color_scale(v);
        }
    }
}

void drawSpectrogram(int offset) {
/*
	//generate_sinewave_stereo(2000, (int32_t *) rx_buf, FRAME_LEN);
    for ( i = 0; i < FFT_N / 2; ++i)
    {
    input_data = (fft_data_t *)&buffer_input[i];
    int32_t v = ((int32_t) (rx_buf[4*i])) * ((int32_t) hann[(i*2)]);
    v = v >> 16 ;
        input_data->R1 = v ;
    //input_data->R1 = rx_buf[2*i];   // data_hard[2 * i].real;
        input_data->I1 = 0;             // data_hard[2 * i].imag;
        v = ((uint32_t) rx_buf[(4*i)+2]) * ((uint32_t) hann[(i*2)+1]);
        v = v >> 16 ;
    input_data->R2 = v;
    //input_data->R2 = rx_buf[2*i+1]; // data_hard[2 * i + 1].real;
        input_data->I2 = 0;             // data_hard[2 * i + 1].imag;
    }
*/
    FFT(offset);

	memcpy(&g_lcd_gram[SPECTROGRAM_HEIGHT], g_lcd_gram_old, (SPECTROGRAM_HEIGHT*(SPECTROGRAM_LENGTH-1))*sizeof(uint16_t));
	for (i = 0; i < SPECTROGRAM_HEIGHT; i++)//Only interested in one size of the FFT
        {
           float pow = data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag;
           pow = 20*(0.5*log(pow)-log(FFT_N)) ;
	   /*float pow = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag)/FFT_N; 
	   pow = 20*log10(pow);*/
           uint16_t c = dbToColor(pow, pmax, pmin);
            c = SWAP_16(c);
           g_lcd_gram[i] = c ;
        }
        memcpy(&g_lcd_gram_old, g_lcd_gram, (SPECTROGRAM_HEIGHT*SPECTROGRAM_LENGTH)*sizeof(uint16_t));
	fps_counter ++ ;
	if (fps_counter >= FPS_MODULO)
    {
		fps_counter = 0;
		lcd.drawImage(0, 0, SPECTROGRAM_HEIGHT, SPECTROGRAM_LENGTH, (uint16_t*) g_lcd_gram);
	}
}


void setup() {
  pll_init();
  plic_init();
//  dmac_init();
  uarths_init();

  lcd.begin(15000000, COLOR_BLACK);
//  Serial.begin(115200);   // initialize serial for debugging

  pio_init();

  g_index = 0;
  i2s_rec_flag = 0;
  i2s_start_flag = 1;
}

void loop() {
    BtnBOOT.read();
    if (BtnBOOT.wasPressed())
    {
        if (mode == MODE_WAVE)
        {
            lcd.fillRect(0, 0, WIDTH, HEIGHT, COLOR_BLACK);
            for (i = 0; i < WIDTH * HEIGHT; i++)
            {
                g_lcd_gram[i] = 0;
            }
            mode = MODE_FFT;
        }
        else if (mode == MODE_FFT)
        {
            for (i = 0; i < WIDTH * HEIGHT; i++)
            {
                g_lcd_gram[i] = 0;
            }
            memcpy(g_lcd_gram_old, g_lcd_gram, WIDTH * HEIGHT * 2);
            initSpectrogram();
            mode = MODE_SPECTRUM;
        }
        else
        {
            lcd.fillRect(0, 0, WIDTH, HEIGHT, COLOR_BLACK);
            mode = MODE_WAVE;
        }
    }
    while (i2s_rec_flag == 0)
        continue;
    if (i2s_rec_flag == 1) {
        if (mode == MODE_WAVE)
        {
            drawWave(0);
        }
        else if (mode == MODE_FFT)
        {
            drawFft(0);
        }
        else
        {
            drawSpectrogram(0);
        }
    } else {
        if (mode == MODE_WAVE)
        {
            drawWave(FRAME_LEN);
        }
        else if (mode == MODE_FFT)
        {
            drawFft(FRAME_LEN);
        }
        else
        {
            drawSpectrogram(FRAME_LEN);
        }
    }
    i2s_rec_flag = 0;
}