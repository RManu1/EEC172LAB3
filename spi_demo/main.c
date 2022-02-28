//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required
//                        initialization sequence to enable the CC3200 SPI
//                        module in full duplex 4-wire master and slave mode(s).
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_apps_rcm.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "gpio.h"
#include "timer.h"

// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"
#include "gpio_if.h"
#include "timer_if.h"

#define APPLICATION_VERSION     "1.4.0"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "Adafruit_GFX.h"

extern int cursor_x;
extern int cursor_y;

float p = 3.1415926;

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

static int ONE[] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1 };
static int TWO[] = { 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1 };
static int THREE[] = { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1 };
static int FOUR[] = { 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 1 };
static int FIVE[] = { 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1 };
static int SIX[] = { 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1 };
static int SEVEN[] = { 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1 };
static int EIGHT[] = { 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1 };
static int NINE[] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1 };
static int ENTER[] = { 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1 };
static int LAST[] = { 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1 };
static int ADDRESS[] = { 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, };
#define ONE_LENGTH 226 // 2.255 ms
#define ZERO_LENGTH 113 // 1.129 ms
#define INTRO 1349 // 13.493 ms
#define END 4048 //40.479 ms
#define MICROSECONDS_TO_TICKS(us)   ((SYS_CLK/1000000) * (us))
#define CLOCK_TICKS 1129

#define APPLICATION_VERSION     "1.4.0"
#define APP_NAME                "I2C Demo"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}
//*****************************************************************************
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      0

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

volatile unsigned long receiver_intcount;
volatile unsigned char receiver_intflag;

typedef struct PinSetting
{
    unsigned long port;
    unsigned int pin;
} PinSetting;

static PinSetting receiver = { .port = GPIOA0_BASE, .pin = 0x40 };


static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulRefTimerInts = 0;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
void TimerBaseIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear (g_ulBase);

    g_ulTimerInts++;
}
//*****************************************************************************

//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************

static void BoardInit(void);

static void GPIOIntHandler(void)
{ // SW3 handler
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus(receiver.port, true);
    MAP_GPIOIntClear(receiver.port, ulStatus);     // clear interrupts on GPIOA1
    receiver_intcount++;
    receiver_intflag = 1;
}

static void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long) &g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
void Timer_Start(unsigned long ulBase, unsigned long ulTimer,
                 unsigned long ulValue)
{
    MAP_TimerLoadSet(ulBase, ulTimer, MICROSECONDS_TO_TICKS(ulValue));
    //
    // Enable the GPT
    //
    MAP_TimerEnable(ulBase, ulTimer);
}
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************

void writeCommand(unsigned char c)
{

//TODO 1
    /* Write a function to send a command byte c to the OLED via
     *  SPI.
     *
     */
    unsigned long ulDummy;

    GPIOPinWrite(GPIOA3_BASE, 0x10, 0x00);    // Set DC Low
    GPIOPinWrite(GPIOA3_BASE, 0x80, 0x00);   // Set CS Low

    MAP_SPIDataPut(GSPI_BASE, c);        // Send c to OLED
    MAP_SPIDataGet(GSPI_BASE, &ulDummy);

}
//*****************************************************************************

void writeData(unsigned char c)
{

//TODO 2
    /* Write a function to send a data byte c to the OLED via
     *  SPI.
     */
    unsigned long ulDummy;

    GPIOPinWrite(GPIOA3_BASE, 0x10, 0xFF);    // Set DC High
    GPIOPinWrite(GPIOA3_BASE, 0x80, 0x00);   // Set CS Low

    MAP_SPIDataPut(GSPI_BASE, c);        // Send c to OLED
    MAP_SPIDataGet(GSPI_BASE, &ulDummy);
}

//*****************************************************************************
void Adafruit_Init(void)
{

//TODO 3
    /* NOTE: This function assumes that the RESET pin of the
     *  OLED has been wired to GPIO28, pin 18 (P2.2). If you
     *  use a different pin for the OLED reset, then you should
     *  update the GPIOPinWrite commands below that set RESET
     *  high or low.
     */
//Reset is pin 15 which is 0x40
    volatile unsigned long delay;

    GPIOPinWrite(GPIOA2_BASE, 0x40, 0);   // RESET = RESET_LOW

    for (delay = 0; delay < 100; delay = delay + 1)
        ;   // delay minimum 100 ns

    GPIOPinWrite(GPIOA2_BASE, 0x40, 0xFF);    // RESET = RESET_HIGH

    // Initialization Sequence
    writeCommand(SSD1351_CMD_COMMANDLOCK);  // set command lock
    writeData(0x12);
    writeCommand(SSD1351_CMD_COMMANDLOCK);  // set command lock
    writeData(0xB1);

    writeCommand(SSD1351_CMD_DISPLAYOFF);         // 0xAE

    writeCommand(SSD1351_CMD_CLOCKDIV);       // 0xB3
    writeCommand(0xF1); // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)

    writeCommand(SSD1351_CMD_MUXRATIO);
    writeData(127);

    writeCommand(SSD1351_CMD_SETREMAP);
    writeData(0x74);

    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(0x00);
    writeData(0x7F);
    writeCommand(SSD1351_CMD_SETROW);
    writeData(0x00);
    writeData(0x7F);

    writeCommand(SSD1351_CMD_STARTLINE);      // 0xA1
    if (SSD1351HEIGHT == 96)
    {
        writeData(96);
    }
    else
    {
        writeData(0);
    }

    writeCommand(SSD1351_CMD_DISPLAYOFFSET);  // 0xA2
    writeData(0x0);

    writeCommand(SSD1351_CMD_SETGPIO);
    writeData(0x00);

    writeCommand(SSD1351_CMD_FUNCTIONSELECT);
    writeData(0x01); // internal (diode drop)
    //writeData(0x01); // external bias

//    writeCommand(SSSD1351_CMD_SETPHASELENGTH);
//    writeData(0x32);

    writeCommand(SSD1351_CMD_PRECHARGE);          // 0xB1
    writeCommand(0x32);

    writeCommand(SSD1351_CMD_VCOMH);              // 0xBE
    writeCommand(0x05);

    writeCommand(SSD1351_CMD_NORMALDISPLAY);      // 0xA6

    writeCommand(SSD1351_CMD_CONTRASTABC);
    writeData(0xC8);
    writeData(0x80);
    writeData(0xC8);

    writeCommand(SSD1351_CMD_CONTRASTMASTER);
    writeData(0x0F);

    writeCommand(SSD1351_CMD_SETVSL);
    writeData(0xA0);
    writeData(0xB5);
    writeData(0x55);

    writeCommand(SSD1351_CMD_PRECHARGE2);
    writeData(0x01);

    writeCommand(SSD1351_CMD_DISPLAYON);      //--turn on oled panel
}

/***********************************/

void goTo(int x, int y)
{
    if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
        return;

    // set x and y coordinate
    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(x);
    writeData(SSD1351WIDTH - 1);

    writeCommand(SSD1351_CMD_SETROW);
    writeData(y);
    writeData(SSD1351HEIGHT - 1);

    writeCommand(SSD1351_CMD_WRITERAM);
}

unsigned int Color565(unsigned char r, unsigned char g, unsigned char b)
{
    unsigned int c;
    c = r >> 3;
    c <<= 6;
    c |= g >> 2;
    c <<= 5;
    c |= b >> 3;

    return c;
}

/**************************************************************************/
/*!
 @brief  Draws a filled rectangle using HW acceleration
 */
/**************************************************************************/
void fillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h,
              unsigned int fillcolor)
{
    unsigned int i;

    // Bounds check
    if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
        return;

    // Y bounds check
    if (y + h > SSD1351HEIGHT)
    {
        h = SSD1351HEIGHT - y - 1;
    }

    // X bounds check
    if (x + w > SSD1351WIDTH)
    {
        w = SSD1351WIDTH - x - 1;
    }

    // set location
    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(x);
    writeData(x + w - 1);
    writeCommand(SSD1351_CMD_SETROW);
    writeData(y);
    writeData(y + h - 1);
    // fill!
    writeCommand(SSD1351_CMD_WRITERAM);

    for (i = 0; i < w * h; i++)
    {
        writeData(fillcolor >> 8);
        writeData(fillcolor);
    }
}

void fillScreen(unsigned int fillcolor)
{
    fillRect(0, 0, SSD1351WIDTH, SSD1351HEIGHT, fillcolor);
}

void drawFastVLine(int x, int y, int h, unsigned int color)
{

    unsigned int i;
    // Bounds check
    if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
        return;

    // X bounds check
    if (y + h > SSD1351HEIGHT)
    {
        h = SSD1351HEIGHT - y - 1;
    }

    if (h < 0)
        return;

    // set location
    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(x);
    writeData(x);
    writeCommand(SSD1351_CMD_SETROW);
    writeData(y);
    writeData(y + h - 1);
    // fill!
    writeCommand(SSD1351_CMD_WRITERAM);

    for (i = 0; i < h; i++)
    {
        writeData(color >> 8);
        writeData(color);
    }
}

void drawFastHLine(int x, int y, int w, unsigned int color)
{

    unsigned int i;
    // Bounds check
    if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
        return;

    // X bounds check
    if (x + w > SSD1351WIDTH)
    {
        w = SSD1351WIDTH - x - 1;
    }

    if (w < 0)
        return;

    // set location
    writeCommand(SSD1351_CMD_SETCOLUMN);
    writeData(x);
    writeData(x + w - 1);
    writeCommand(SSD1351_CMD_SETROW);
    writeData(y);
    writeData(y);
    // fill!
    writeCommand(SSD1351_CMD_WRITERAM);

    for (i = 0; i < w; i++)
    {
        writeData(color >> 8);
        writeData(color);
    }
}

void drawPixel(int x, int y, unsigned int color)
{
    if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
        return;
    if ((x < 0) || (y < 0))
        return;

    goTo(x, y);

    writeData(color >> 8);
    writeData(color);
}

void invert(char v)
{
    if (v)
    {
        writeCommand(SSD1351_CMD_INVERTDISPLAY);
    }
    else
    {
        writeCommand(SSD1351_CMD_NORMALDISPLAY);
    }
}

unsigned char returnChar(int n, int r)
{
        return 's';
}
void main()
{

    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Initialize Board configurations
    //
    unsigned long ulStatus;

    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
    SPI_IF_BIT_RATE,
                           SPI_MODE_MASTER, SPI_SUB_MODE_0, (SPI_SW_CTRL_CS |
                           SPI_4PIN_MODE |
                           SPI_TURBO_OFF |
                           SPI_CS_ACTIVEHIGH |
                           SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    MAP_SPITransfer(GSPI_BASE, g_ucTxBuff, g_ucRxBuff, 50,
    SPI_CS_ENABLE | SPI_CS_DISABLE);
    MAP_SPICSEnable(GSPI_BASE);

    Adafruit_Init();

    InitTerm();

    ClearTerm();

    MAP_GPIOIntRegister(receiver.port, GPIOIntHandler);

    MAP_GPIOIntTypeSet(receiver.port, receiver.pin, GPIO_FALLING_EDGE);

    ulStatus = MAP_GPIOIntStatus(receiver.port, false);
    MAP_GPIOIntClear(receiver.port, ulStatus);

    receiver_intcount = 0;
    receiver_intflag = 0;

    MAP_GPIOIntEnable(receiver.port, receiver.pin);

    g_ulBase = TIMERA0_BASE;

    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);

    //Timer_IF_Start(g_ulBase, TIMER_A, 1);

    Timer_Start(g_ulBase, TIMER_A, CLOCK_TICKS);
    unsigned int x[34];
    Message("Welcome to remote presser\n\r");
    int i = 0;
    int button = 0;

    /*   while(1)
     {

     //

     // Clear screen
     fillScreen(WHITE);

     // Print "Hello World!"
     drawChar(1,1,'H',BLACK,WHITE,1);
     drawChar(7,1,'e',BLACK,WHITE,1);
     drawChar(13,1,'l',BLACK,WHITE,1);
     drawChar(19,1,'l',BLACK,WHITE,1);
     drawChar(25,1,'o',BLACK,WHITE,1);
     drawChar(31,1,' ',BLACK,WHITE,1);
     drawChar(37,1,'W',BLACK,WHITE,1);
     drawChar(43,1,'o',BLACK,WHITE,1);
     drawChar(49,1,'r',BLACK,WHITE,1);
     drawChar(55,1,'l',BLACK,WHITE,1);
     drawChar(61,1,'d',BLACK,WHITE,1);
     drawChar(67,1,'!',BLACK,WHITE,1);
     MAP_UtilsDelay(8000000);





     }*/

fillScreen(WHITE);
fillCircle(64,64,2,BLACK);
int xpos = 1;
int ypos = 1;
int shift = 0;
int current = 0;
int repeat = 0;
int new1 = 0;
int clr = 0;
    while (FOREVER)
    {
        g_ulTimerInts = 0;
        while (receiver_intflag == 0)
        {
            ;
        }
        Report("%d ", g_ulTimerInts);
        if (g_ulTimerInts > 2000)
        {shift = 1;}
        x[i] = g_ulTimerInts - 1;
        if (i == 33)
        {

            int j;
            /* for (j = 18; j < 34; j++)
             {
             Report("%d ",x[j]);
             }
             Message("\n\r");
             for (j = 0; j < 16; j++)
             {
             Report("%d ",ONE[j]);
             }
             Message("\n\r");*/
            int k = 0;
            int sum = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == ONE[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("1 was pressed\r\n");
                button = 1;
                new1 = 1;
                clr++;
                if(clr > 4)
                 clr = 0;
                 if (clr == 0)
                  {
                      fillCircle(64,64,2,BLACK);
                  }
                  if (clr == 1)
                    {
                        fillCircle(64,64,2,CYAN);
                    }
                  if (clr == 2)
                    {
                        fillCircle(64,64,2,GREEN);
                    }
                  if (clr == 3)
                    {
                        fillCircle(64,64,2,RED);
                    }
                  if (clr == 4)
                    {
                        fillCircle(64,64,2,BLUE);
                    }
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == TWO[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("2 was pressed\r\n");
                button = 1;
                new1 = 2;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == THREE[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("3 was pressed\r\n");
                button = 1;
                new1 = 3;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == FOUR[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("4 was pressed\r\n");
                button = 1;
                new1 = 4;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == FIVE[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("5 was pressed\r\n");
                button = 1;
                new1 = 5;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == SIX[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("6 was pressed\r\n");
                button = 1;
                new1 = 6;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == SEVEN[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("7 was pressed\r\n");
                button = 1;
                new1 = 7;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == EIGHT[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("8 was pressed\r\n");
                button = 1;
                new1 = 8;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == NINE[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("9 was pressed\r\n");
                button = 1;
                new1 = 9;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == ENTER[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("Enter was pressed\r\n");
                button = 1;
                new1 = 10;
            }
            sum = 0;
            k = 0;
            for (j = 18; j < 34; j++)
            {
                if (x[j] == LAST[k])
                {
                    sum++;
                }
                k++;
            }
            if (sum == 16)
            {
                Message("Last was pressed\r\n");
                button = 1;
                new1 = 11;
            }
            if (button == 0)
            {
                Message("Button not recognized.\r\n");
            }
  if(new1 == 10)
  {
      fillScreen(WHITE);
  }
  else if (new1 == 11)
  {
      drawChar(xpos,ypos,' ',BLACK,WHITE,1);
      if (xpos < 7 && ypos > 1)
      {
          xpos = 1;
          ypos = ypos - 6;
      }
      else
        {
          xpos = xpos - 6;
        }

  }
  else if (new1 == 1)
  {
      clr++;
      if(clr > 4)
      {clr = 0;}
      if (clr == 0)
      {
          fillCircle(64,64,2,BLACK);
      }
      if (clr == 1)
        {
            fillCircle(64,64,2,CYAN);
        }
      if (clr == 2)
        {
            fillCircle(64,64,2,GREEN);
        }
      if (clr == 3)
        {
            fillCircle(64,64,2,RED);
        }
      if (clr == 4)
        {
            fillCircle(64,64,2,BLUE);
        }
  }
  else
  {
      if(new1 == current)
      {
          repeat++;
      }
      else
      {
          current = new1;
          shift = 1;
      }
      if(shift == 1)
      {
          repeat = 0;
          if(xpos > 121)
          {ypos = ypos + 6;
              xpos = 0;}
          else
          {xpos = xpos + 6;}
          shift = 0;
      }
      else
      {
          unsigned char c = returnChar(new1, repeat);
          if (clr == 0)
                {
                  drawChar(xpos,ypos,c,BLACK,WHITE,1);
                }
          if (clr == 1)
                  {
              drawChar(xpos,ypos,c,CYAN,WHITE,1);
                  }
          if (clr == 2)
                  {
              drawChar(xpos,ypos,c,GREEN,WHITE,1);
                  }
          if (clr == 3)
                  {
              drawChar(xpos,ypos,c,RED,WHITE,1);
                  }
          if (clr == 4)
                  {
              drawChar(xpos,ypos,c,BLUE,WHITE,1);
                  }
      }
  }


            i = -1;
            button = 0;
            MAP_UtilsDelay(8000000);
            MAP_UtilsDelay(8000000);
        }
        //Report("press read %d time %d\r\n", i, g_ulTimerInts);
        i++;

        receiver_intflag = 0;
    }

    MAP_SPICSDisable(GSPI_BASE);
    MAP_SPIDisable(GSPI_BASE);
}


