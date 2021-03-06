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
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "gpio.h"

// Common interface includes
#include "uart_if.h"
#include "pinmux.h"
#include "gpio_if.h"


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
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************
static void SlaveIntHandler()
{
    unsigned long ulRecvData;
    unsigned long ulStatus;

    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);

    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    if(ulStatus & SPI_INT_TX_EMPTY)
    {
        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;
    }

    if(ulStatus & SPI_INT_RX_FULL)
    {
        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        Report("%c",ulRecvData);
        ucRxBuffNdx++;
    }
}

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{

    unsigned long ulUserData;
    unsigned long ulDummy;

    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,MASTER_MSG,sizeof(MASTER_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Master Mode\n\r");

    //
    // User input
    //
    Report("Press any key to transmit data....");

    //
    // Read a character from UART terminal
    //
    ulUserData = MAP_UARTCharGet(UARTA0_BASE);


    //
    // Send the string to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //
    MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
            SPI_CS_ENABLE|SPI_CS_DISABLE);

    //
    // Report to the user
    //
    Report("\n\rSend      %s",g_ucTxBuff);
    Report("Received  %s",g_ucRxBuff);

    //
    // Print a message
    //
    Report("\n\rType here (Press enter to exit) :");

    //
    // Initialize variable
    //
    ulUserData = 0;

    //
    // Enable Chip select
    //
    MAP_SPICSEnable(GSPI_BASE);

    //
    // Loop until user "Enter Key" is
    // pressed
    //
    while(ulUserData != '\r')
    {
        //
        // Read a character from UART terminal
        //
        ulUserData = MAP_UARTCharGet(UARTA0_BASE);

        //
        // Echo it back
        //
        MAP_UARTCharPut(UARTA0_BASE,ulUserData);

        //
        // Push the character over SPI
        //
        MAP_SPIDataPut(GSPI_BASE,ulUserData);

        //
        // Clean up the receive register into a dummy
        // variable
        //
        MAP_SPIDataGet(GSPI_BASE,&ulDummy);
    }

    //
    // Disable chip select
    //
    MAP_SPICSDisable(GSPI_BASE);
}

//*****************************************************************************
//
//! SPI Slave mode main loop
//!
//! This function configures SPI modelue as slave and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void SlaveMain()
{
    //
    // Initialize the message
    //
    memcpy(g_ucTxBuff,SLAVE_MSG,sizeof(SLAVE_MSG));

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_SLAVE,SPI_SUB_MODE_0,
                     (SPI_HW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Register Interrupt Handler
    //
    MAP_SPIIntRegister(GSPI_BASE,SlaveIntHandler);

    //
    // Enable Interrupts
    //
    MAP_SPIIntEnable(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //
    // Print mode on uart
    //
    Message("Enabled SPI Interface in Slave Mode\n\rReceived : ");
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
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
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
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************

void writeCommand(unsigned char c) {

//TODO 1
/* Write a function to send a command byte c to the OLED via
*  SPI.
*
*/
    unsigned long ulDummy;

    GPIOPinWrite(GPIOA3_BASE,0x10,0x00);    // Set DC Low
    GPIOPinWrite(GPIOA3_BASE,0x80,0x00);   // Set CS Low

    MAP_SPIDataPut(GSPI_BASE,c);        // Send c to OLED
    MAP_SPIDataGet(GSPI_BASE,&ulDummy);

}
//*****************************************************************************

void writeData(unsigned char c) {

//TODO 2
/* Write a function to send a data byte c to the OLED via
*  SPI.
*/
    unsigned long ulDummy;

    GPIOPinWrite(GPIOA3_BASE,0x10,0xFF);    // Set DC High
    GPIOPinWrite(GPIOA3_BASE,0x80,0x00);   // Set CS Low

    MAP_SPIDataPut(GSPI_BASE,c);        // Send c to OLED
    MAP_SPIDataGet(GSPI_BASE,&ulDummy);
}

//*****************************************************************************
void Adafruit_Init(void){

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

  for(delay=0; delay<100; delay=delay+1);// delay minimum 100 ns

  GPIOPinWrite(GPIOA2_BASE, 0x40, 0xFF);    // RESET = RESET_HIGH

    // Initialization Sequence
  writeCommand(SSD1351_CMD_COMMANDLOCK);  // set command lock
  writeData(0x12);
  writeCommand(SSD1351_CMD_COMMANDLOCK);  // set command lock
  writeData(0xB1);

  writeCommand(SSD1351_CMD_DISPLAYOFF);         // 0xAE

  writeCommand(SSD1351_CMD_CLOCKDIV);       // 0xB3
  writeCommand(0xF1);                       // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)

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
  if (SSD1351HEIGHT == 96) {
    writeData(96);
  } else {
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

  writeCommand(SSD1351_CMD_SETVSL );
  writeData(0xA0);
  writeData(0xB5);
  writeData(0x55);

  writeCommand(SSD1351_CMD_PRECHARGE2);
  writeData(0x01);

  writeCommand(SSD1351_CMD_DISPLAYON);      //--turn on oled panel
}

/***********************************/

void goTo(int x, int y) {
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;

  // set x and y coordinate
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(SSD1351WIDTH-1);

  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(SSD1351HEIGHT-1);

  writeCommand(SSD1351_CMD_WRITERAM);
}

unsigned int Color565(unsigned char r, unsigned char g, unsigned char b) {
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
void fillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int fillcolor)
{
  unsigned int i;

  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  // Y bounds check
  if (y+h > SSD1351HEIGHT)
  {
    h = SSD1351HEIGHT - y - 1;
  }

  // X bounds check
  if (x+w > SSD1351WIDTH)
  {
    w = SSD1351WIDTH - x - 1;
  }

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x+w-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y+h-1);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);

  for (i=0; i < w*h; i++) {
    writeData(fillcolor >> 8);
    writeData(fillcolor);
  }
}

void fillScreen(unsigned int fillcolor) {
  fillRect(0, 0, SSD1351WIDTH, SSD1351HEIGHT, fillcolor);
}

void drawFastVLine(int x, int y, int h, unsigned int color) {

  unsigned int i;
  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  // X bounds check
  if (y+h > SSD1351HEIGHT)
  {
    h = SSD1351HEIGHT - y - 1;
  }

  if (h < 0) return;

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y+h-1);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);

  for (i=0; i < h; i++) {
    writeData(color >> 8);
    writeData(color);
  }
}

void drawFastHLine(int x, int y, int w, unsigned int color) {

  unsigned int i;
  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  // X bounds check
  if (x+w > SSD1351WIDTH)
  {
    w = SSD1351WIDTH - x - 1;
  }

  if (w < 0) return;

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x+w-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);

  for (i=0; i < w; i++) {
    writeData(color >> 8);
    writeData(color);
  }
}

void drawPixel(int x, int y, unsigned int color)
{
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  if ((x < 0) || (y < 0)) return;

  goTo(x, y);

  writeData(color >> 8);
  writeData(color);
}

void  invert(char v) {
   if (v) {
     writeCommand(SSD1351_CMD_INVERTDISPLAY);
   } else {
        writeCommand(SSD1351_CMD_NORMALDISPLAY);
   }
 }
void main()
{

    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
            // Initialize Board configurations
            //
            BoardInit();

            //
            // Muxing UART and SPI lines.
            //
            PinMuxConfig();

            //
            // Enable the SPI module clock
            //
            MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

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
                MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                                 SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                                 (SPI_SW_CTRL_CS |
                                 SPI_4PIN_MODE |
                                 SPI_TURBO_OFF |
                                 SPI_CS_ACTIVEHIGH |
                                 SPI_WL_8));

                //
                // Enable SPI for communication
                //
                MAP_SPIEnable(GSPI_BASE);

                MAP_SPITransfer(GSPI_BASE,g_ucTxBuff,g_ucRxBuff,50,
                                SPI_CS_ENABLE|SPI_CS_DISABLE);
                MAP_SPICSEnable(GSPI_BASE);

                Adafruit_Init();


        while(1)
        {
            int x = 1;
               int y = 1;

        // Print full character-set
        unsigned int i;
        for (i = 0; i < sizeof(font); i++)
        {
            drawChar(x,y,font[i],BLACK,WHITE,1);
            x += 6;
            if (x > 121)
            {
                y += 8;
                x = 1;
            }
        }
        MAP_UtilsDelay(8000000);

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

        // Display Horizontal bands
        for (i = 1; i < 17; i++)
        {
            drawFastHLine(1,i,128,BLACK);
        }
        for (i = 17; i < 33; i++)
        {
            drawFastHLine(1,i,128,BLUE);
        }
        for (i = 33; i < 49; i++)
        {
            drawFastHLine(1,i,128,GREEN);
        }
        for (i = 49; i < 65; i++)
        {
            drawFastHLine(1,i,128,CYAN);
        }
        for (i = 65; i < 81; i++)
        {
            drawFastHLine(1,i,128,RED);
        }
        for (i = 81; i < 97; i++)
        {
            drawFastHLine(1,i,128,MAGENTA);
        }
        for (i = 97; i < 113; i++)
        {
            drawFastHLine(1,i,128,YELLOW);
        }
        for (i = 113; i < 129; i++)
        {
            drawFastHLine(1,i,128,WHITE);
        }
        MAP_UtilsDelay(8000000);

        //Display Vertical Bars
        for (i = 1; i < 17; i++)
        {
            drawFastVLine(i,1,128,BLACK);
        }
        for (i = 17; i < 33; i++)
        {
            drawFastVLine(i,1,128,BLUE);
        }
        for (i = 33; i < 49; i++)
        {
            drawFastVLine(i,1,128,GREEN);
        }
        for (i = 49; i < 65; i++)
        {
            drawFastVLine(i,1,128,CYAN);
        }
        for (i = 65; i < 81; i++)
        {
            drawFastVLine(i,1,128,RED);
        }
        for (i = 81; i < 97; i++)
        {
            drawFastVLine(i,1,128,MAGENTA);
        }
        for (i = 97; i < 113; i++)
        {
            drawFastVLine(i,1,128,YELLOW);
        }
        for (i = 113; i < 129; i++)
        {
            drawFastVLine(i,1,128,WHITE);
        }
        MAP_UtilsDelay(8000000);

        // Draw Diagonal Lines
        testlines(CYAN);
        MAP_UtilsDelay(8000000);

        // Display Rectangular Grid
        testfastlines(MAGENTA,GREEN);
        MAP_UtilsDelay(8000000);

        // Display Rectangle Pattern
        testdrawrects(YELLOW);
        MAP_UtilsDelay(8000000);

        // Fill Rectangles
        testfillrects(WHITE,BLUE);
        MAP_UtilsDelay(8000000);

        // Circles
        fillScreen(WHITE);
        testfillcircles(10,MAGENTA);
        MAP_UtilsDelay(8000000);
        testdrawcircles(10,BLACK);
        MAP_UtilsDelay(8000000);

        // Round Rectangles
        testroundrects();
        MAP_UtilsDelay(8000000);

        // Triangles
        testtriangles();
        MAP_UtilsDelay(8000000);

        // Clear screen
               fillScreen(WHITE);



        }
        MAP_SPICSDisable(GSPI_BASE);
              MAP_SPIDisable(GSPI_BASE);
}

//*****************************************************************************
//  function delays 3*ulCount cycles
void delay(unsigned long ulCount){
    int i;

  do{
    ulCount--;
        for (i=0; i< 65535; i++) ;
    }while(ulCount);
}


//*****************************************************************************
void testfastlines(unsigned int color1, unsigned int color2) {
    unsigned int x;
    unsigned int y;

   fillScreen(BLACK);
   for (y=0; y < height()-1; y+=8) {
     drawFastHLine(0, y, width()-1, color1);
   }
     delay(100);
   for (x=0; x < width()-1; x+=8) {
     drawFastVLine(x, 0, height()-1, color2);
   }
     delay(100);
}

//*****************************************************************************

void testdrawrects(unsigned int color) {
    unsigned int x;

 fillScreen(BLACK);
 for (x=0; x < height()-1; x+=6) {
   drawRect((width()-1)/2 -x/2, (height()-1)/2 -x/2 , x, x, color);
     delay(10);
 }
}

//*****************************************************************************

void testfillrects(unsigned int color1, unsigned int color2) {

    unsigned char x;

 fillScreen(BLACK);
 for (x=height()-1; x > 6; x-=6) {
   fillRect((width()-1)/2 -x/2, (height()-1)/2 -x/2 , x, x, color1);
   drawRect((width()-1)/2 -x/2, (height()-1)/2 -x/2 , x, x, color2);
     delay(10);
 }
}

//*****************************************************************************

void testfillcircles(unsigned char radius, unsigned int color) {
    unsigned char x;
    unsigned char y;

  for (x=radius; x < width()-1; x+=radius*2) {
    for (y=radius; y < height()-1; y+=radius*2) {
      fillCircle(x, y, radius, color);
            delay(10);
    }
  }
}

//*****************************************************************************

void testdrawcircles(unsigned char radius, unsigned int color) {
    unsigned char x;
    unsigned char y;

  for (x=0; x < width()-1+radius; x+=radius*2) {
    for (y=0; y < height()-1+radius; y+=radius*2) {
      drawCircle(x, y, radius, color);
            delay(10);
    }
  }
}

//*****************************************************************************

void testtriangles() {
  int color = 0xF800;
  int t;
  int w = width()/2;
  int x = height()-1;
  int y = 0;
  int z = width()-1;

  fillScreen(BLACK);
  for(t = 0 ; t <= 15; t+=1) {
    drawTriangle(w, y, y, x, z, x, color);
    x-=4;
    y+=4;
    z-=4;
    color+=100;
        delay(10);
  }
}

//*****************************************************************************

void testroundrects() {
  int color = 100;

    int i;
  int x = 0;
  int y = 0;
  int w = width();
  int h = height();

  fillScreen(BLACK);

  for(i = 0 ; i <= 24; i++) {
    drawRoundRect(x, y, w, h, 5, color);
    x+=2;
    y+=3;
    w-=4;
    h-=6;
    color+=1100;
  }
}

//*****************************************************************************
void testlines(unsigned int color) {
    unsigned int x;
    unsigned int y;

   fillScreen(BLACK);
   for (x=0; x < width()-1; x+=6) {
     drawLine(0, 0, x, height()-1, color);
   }
     delay(10);
   for (y=0; y < height()-1; y+=6) {
     drawLine(0, 0, width()-1, y, color);
   }
     delay(100);

   fillScreen(BLACK);
   for (x=0; x < width()-1; x+=6) {
     drawLine(width()-1, 0, x, height()-1, color);
   }
     delay(100);
   for (y=0; y < height()-1; y+=6) {
     drawLine(width()-1, 0, 0, y, color);
   }
     delay(100);

   fillScreen(BLACK);
   for (x=0; x < width()-1; x+=6) {
     drawLine(0, height()-1, x, 0, color);
   }
     delay(100);
   for (y=0; y < height()-1; y+=6) {
     drawLine(0, height()-1, width()-1, y, color);
   }
     delay(100);

   fillScreen(BLACK);
   for (x=0; x < width()-1; x+=6) {
     drawLine(width()-1, height()-1, x, 0, color);
   }
     delay(100);
   for (y=0; y < height()-1; y+=6) {
     drawLine(width()-1, height()-1, 0, y, color);
   }
     delay(100);

}

//*****************************************************************************

void lcdTestPattern(void)
{
  unsigned int i,j;
  goTo(0, 0);

  for(i=0;i<128;i++)
  {
    for(j=0;j<128;j++)
    {
      if(i<16){writeData(RED>>8); writeData((unsigned char) RED);}
      else if(i<32) {writeData(YELLOW>>8);writeData((unsigned char) YELLOW);}
      else if(i<48){writeData(GREEN>>8);writeData((unsigned char) GREEN);}
      else if(i<64){writeData(CYAN>>8);writeData((unsigned char) CYAN);}
      else if(i<80){writeData(BLUE>>8);writeData((unsigned char) BLUE);}
      else if(i<96){writeData(MAGENTA>>8);writeData((unsigned char) MAGENTA);}
      else if(i<112){writeData(BLACK>>8);writeData((unsigned char) BLACK);}
      else {writeData(WHITE>>8); writeData((unsigned char) WHITE);}
    }
  }
}
/**************************************************************************/
void lcdTestPattern2(void)
{
  unsigned int i,j;
  goTo(0, 0);

  for(i=0;i<128;i++)
  {
    for(j=0;j<128;j++)
    {
      if(j<16){writeData(RED>>8); writeData((unsigned char) RED);}
      else if(j<32) {writeData(YELLOW>>8);writeData((unsigned char) YELLOW);}
      else if(j<48){writeData(GREEN>>8);writeData((unsigned char) GREEN);}
      else if(j<64){writeData(CYAN>>8);writeData((unsigned char) CYAN);}
      else if(j<80){writeData(BLUE>>8);writeData((unsigned char) BLUE);}
      else if(j<96){writeData(MAGENTA>>8);writeData((unsigned char) MAGENTA);}
      else if(j<112){writeData(BLACK>>8);writeData((unsigned char) BLACK);}
      else {writeData(WHITE>>8);writeData((unsigned char) WHITE);}
    }
  }
}

/**************************************************************************/
