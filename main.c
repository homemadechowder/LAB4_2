
// Standard includes
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "pin.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "spi.h"
#include "uart.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "timer_if.h"
#include "timer.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "test.h"

// Common interface includes
#include "uart_if.h"
#include "gpio_if.h"

#include "pin_mux_config.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

#define SPI_IF_BIT_RATE  400000
#define MAX_BUFFER  80
#define SAMPLE_SIZE 410

#define ZERO  11
#define ONE   1
#define TWO   2
#define THREE 3
#define FOUR  4
#define FIVE  5
#define SIX   6
#define SEVEN 7
#define EIGHT 8
#define NINE  9
#define MUTE  10
#define LAST  12

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define GREEN           0x07E0
#define CYAN            0x07FF
#define RED             0xF800
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

extern void (* const g_pfnVectors[])(void);

volatile long currentPress;
volatile long previousPress;
volatile long currentButton;
volatile long previousButton;
signed long buffer[SAMPLE_SIZE]; // buffer to store ADC data
unsigned long A0TICK = (80000000 / 16000);
unsigned long A1TICK = (80000000 / 1);
unsigned short sample_size = 0;
bool isSampling = false;
bool isProcessing = false;
int new_digit = 0;
long int coeff_array[7] = {31548, 31281, 30951, 30556, 29144, 28361, 27409};
long int power_all[7];

int isButton = -1;  // check if it is a valid button
int sameButton = 0; // check if press the same button consecutively flag
int bufferSize = 0; // the buffer size for composing message
int receiveSize = 0; // the buffer size for received message
int composing_x = 5, composing_y = 68; // composing message position
int received_x = 5, received_y = 4; // received message position
int messageReady = 0; // indicate if the received message is read to display
int previousSize = 0;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

typedef struct PinSetting {
    unsigned long port;
    unsigned int pin;
} PinSetting;

typedef struct Letter {
    unsigned int x;
    unsigned int y;
    char letter;
} Letter;

static PinSetting PIN8  = { .port = GPIOA2_BASE, .pin = 0x2 }; // GPIOPIN8 for MISO of ADC
static PinSetting PIN61  = { .port = GPIOA0_BASE, .pin = 0x40 }; // GPIOPIN61 for CS for OLED
static PinSetting PIN62 = { .port = GPIOA0_BASE, .pin = 0x80 }; // GPIOPIN62 for DC for OLED
Letter ComposingLetter[MAX_BUFFER];
Letter ReceivedLetter[MAX_BUFFER];
Letter PreviousLetter[MAX_BUFFER];

static void BoardInit(void);

void CollectData();
unsigned short ReadADC();

long int goertzel(long int coeff);
signed char decode();
void Display(unsigned long value);
void MasterMain();
char ToLetter(unsigned long value);
char forwardLetter(char letter, unsigned long value);
void ClearComposingMessage();
void ClearReceivedMessage();
void SendMessage();
void CheckMessage();
void DisplayMessage();

static void SampleCollectHandler(void) {

    Timer_IF_InterruptClear(TIMERA0_BASE);

    sample_size++;
    isSampling = true;

    if (sample_size == SAMPLE_SIZE)
        isProcessing = true;
}

static void ConsecutivePressingHandler(void)
{
    Timer_IF_InterruptClear(TIMERA1_BASE);
    currentPress++;
}

unsigned short ReadADC() {
    unsigned char data0;
    unsigned char data1;
    GPIOPinWrite(PIN8.port, PIN8.pin, 0x00);
    MAP_SPITransfer(GSPI_BASE, 0, &data0, PIN8.pin, SPI_CS_ENABLE);
    MAP_SPITransfer(GSPI_BASE, 0, &data1, PIN8.pin, SPI_CS_DISABLE);
    GPIOPinWrite(PIN8.port, PIN8.pin, PIN8.pin);
    unsigned short data = 0x1f & data0;
    data = (data << 5) | ((0xf8 & data1) >> 3);
    return data;
}

long int goertzel(long int coeff)
{
    //initialize variables to be used in the function
    int Q, Q_prev, Q_prev2,i;
    long prod1, prod2, prod3, power;

    Q_prev = 0;         //set delay element1 Q_prev as zero
    Q_prev2 = 0;        //set delay element2 Q_prev2 as zero
    power=0;            //set power as zero

    for (i = 0; i < SAMPLE_SIZE; i++) // loop SAMPLE_SPACE times and calculate Q, Q_prev, Q_prev2 at each iteration
    {
        Q = (buffer[i]) + ((coeff* Q_prev)>>14) - (Q_prev2); // >>14 used as the coeff was used in Q15 format
        Q_prev2 = Q_prev;                                    // shuffle delay elements
        Q_prev = Q;
    }

    //calculate the three products used to calculate power
    prod1=((long) Q_prev*Q_prev);
    prod2=((long) Q_prev2*Q_prev2);
    prod3=((long) Q_prev *coeff)>>14;
    prod3=(prod3 * Q_prev2);

    power = ((prod1+prod2-prod3))>>8; //calculate power using the three products and scale the result down

    return power;
}

signed char decode() // post_test() function from the Github example
{
    //initialize variables to be used in the function
    int max_power,i, row, col;

    // find the maximum power in the row frequencies and the row number
    max_power=0;            //initialize max_power=0
    for(i=0;i<4;i++) {      //loop 4 times from 0>3 (the indecies of the rows)
        if (power_all[i] > max_power) { //if power of the current row frequency > max_power
            max_power=power_all[i];     //set max_power as the current row frequency
            row=i;                      //update row number
        }
    }

    // find the maximum power in the column frequencies and the column number
    max_power=0;            //initialize max_power=0
    for(i=4;i<7;i++) {      //loop 3 times from 4>7 (the indecies of the columns)
        if (power_all[i] > max_power) { //if power of the current column frequency > max_power
            max_power=power_all[i];     //set max_power as the current column frequency
            col=i;                      //update column number
        }
    }

    if(power_all[col]<=40000 && power_all[row]<=20000) //instead 0 in the original example, set a threshold to avoid noise in the lab
        new_digit = 1;

    if((power_all[col]>20000 && power_all[row]>20000) && (new_digit == 1)) { // check if maximum powers of row & column exceed certain threshold
        new_digit = 0;
        return (3*row + col - 4 + 1);
    }

    return -1;
}

char toLetter(unsigned long value) {
    char letter;
    switch(value) {
        case ZERO:
            letter = ' ';
            break;
        case ONE:
            letter = '?';
            break;
        case TWO:
            letter = 'a';
            break;
        case THREE:
            letter = 'd';
            break;
        case FOUR:
            letter = 'g';
            break;
        case FIVE:
            letter = 'j';
            break;
        case SIX:
            letter = 'm';
            break;
        case SEVEN:
            letter = 'p';
            break;
        case EIGHT:
            letter = 't';
            break;
        case NINE:
            letter = 'w';
            break;
        case MUTE:
            letter = '-';
            break;
        case LAST:
            letter = '+';
            break;
        default:
            letter = '/';
            break;
    }
    return letter;
}

char forwardLetter(char letter, unsigned long value) {

    char newLetter;
    switch(value) {
        case ONE:
            if(letter == '?') {
                newLetter = '.';
            }

            else if(letter == '.') {
                newLetter = ',';
            }

            else {
                newLetter = '?';
            }
            break;
        case TWO:
            if(letter == 'c') {
                newLetter = 'a';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case THREE:
            if(letter == 'f') {
                newLetter = 'd';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case FOUR:
            if(letter == 'i') {
                newLetter = 'g';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case FIVE:
            if(letter == 'l') {
                newLetter = 'j';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case SIX:
            if(letter == 'o') {
                newLetter = 'm';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case SEVEN:
            if(letter == 's') {
                newLetter = 'p';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case EIGHT:
            if(letter == 'v') {
                newLetter = 't';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        case NINE:
            if(letter == 'z') {
                newLetter = 'w';
            }
            else {
                newLetter = letter + 1;
            }
            break;
        default:
            newLetter = letter;
            break;
    }
    return newLetter;
}

void MasterMain()
{
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
    // Initialize Adafruit
    Adafruit_Init();
}

void SetupCommunication()
{
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    PinTypeUART(PIN_58, PIN_MODE_6); //UART1_TX
    PinTypeUART(PIN_59, PIN_MODE_6); //UART1_RX
    PinTypeUART(PIN_55, PIN_MODE_3); //UART0_TX
    PinTypeUART(PIN_57, PIN_MODE_3); //UART0_RX

    //UART Setup
    UARTConfigSetExpClk(UARTA1_BASE, 80000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
    UART_CONFIG_PAR_NONE));
    UARTConfigSetExpClk(UARTA0_BASE, 80000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
    UART_CONFIG_PAR_NONE));
    UARTEnable(UARTA1_BASE);
    UARTEnable(UARTA0_BASE);
    UARTDMADisable(UARTA1_BASE, (UART_DMA_RX | UART_DMA_TX));
    UARTFIFODisable(UARTA1_BASE) ;
}

void ClearComposingMessage() {
    int i;
    for(i = 0; i < bufferSize; i++) {
        drawChar(ComposingLetter[i].x, ComposingLetter[i].y, ComposingLetter[i].letter, BLACK, BLACK, 1);
    }
    bufferSize = 0;
    composing_x = 5;
    composing_y = 68;
}

void ClearPreviousMessage() {
    int i;
    for(i = 0; i < previousSize; i++) {
        drawChar(PreviousLetter[i].x, PreviousLetter[i].y, PreviousLetter[i].letter, BLACK, BLACK, 1);
    }
}

void SendMessage() {

    // if there's something to send

    if(bufferSize > 0) {

        int i;
        for(i = 0; i < bufferSize; i++) {
            // wait for UART to be available
            while(UARTBusy(UARTA1_BASE));
            UARTCharPut(UARTA1_BASE, ComposingLetter[i].letter);
        }

        // end line character
        UARTCharPut(UARTA1_BASE,'\0');
        ClearComposingMessage();
    }
}
void CheckMessage() {

    // clear UART interrupt
    UARTIntClear(UARTA1_BASE,UART_INT_RX);

    // when UART is available
    while(UARTCharsAvail(UARTA1_BASE))
    {
        char c = UARTCharGet(UARTA1_BASE);

        if(c == '\0') {
            messageReady = 1;
        }
        else {
            ReceivedLetter[receiveSize].letter = c;
            ReceivedLetter[receiveSize].x = received_x;
            ReceivedLetter[receiveSize].y = received_y;
            // increase buffer size
            receiveSize++;
            // increment pixel position
            received_x += 7;
            // position boundaries
            if(received_x >= 124) {
                received_x = 5;
                received_y += 10;
            }
        }
    }
}

void DisplayMessage() {
    if(messageReady) {
        // clear flag
        messageReady = 0;

        int i;
        ClearPreviousMessage();
        for(i = 0; i < receiveSize; i++) {
            drawChar(ReceivedLetter[i].x, ReceivedLetter[i].y, ReceivedLetter[i].letter, RED, RED, 1);
            PreviousLetter[i] = ReceivedLetter[i];
        }
        previousSize = receiveSize;
        received_x = 5;
        received_y = 4;
        receiveSize = 0;
    }
    if(isProcessing) {
        isProcessing = false;
        // restart timer
        MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, A0TICK);
        MAP_TimerIntEnable(TIMERA0_BASE, TIMER_A);
        MAP_TimerEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
    }
}

void CollectData() {
    unsigned long ulStatus;
    if (isProcessing) {
        // disable sampling timer
        MAP_TimerDisable(TIMERA0_BASE, TIMER_A);
        MAP_TimerIntDisable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
        ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, false);
        MAP_TimerIntClear(TIMERA0_BASE, ulStatus);

        sample_size = 0;

        int i;
        for (i = 0; i < 7; i++) {
            power_all[i] = goertzel(coeff_array[i]); // call goertzel to calculate the power at each frequency and store it in the p
        }

        currentButton = decode();

        // print out composing message on OLED

        // valid button from remote
        if(currentButton != -1) {
            Timer_IF_Start(TIMERA1_BASE, TIMER_A, 420);
            // set up same button flag
            if(previousButton == currentButton) {
                sameButton = 1;
            }

            else {
                sameButton = 0;
            }
            // delete a character
            if(toLetter(currentButton) == '-') {

                // draw previous black
                if(bufferSize > 0) {
                    bufferSize--;
                    drawChar(ComposingLetter[bufferSize].x, ComposingLetter[bufferSize].y, ComposingLetter[bufferSize].letter, BLACK, BLACK, 1);
                }

                // set new composing position for letters
                if(composing_x >= 12) {
                    composing_x -= 7;
                }
                else if(composing_x == 5) {
                    if(composing_y >= 78) {
                        composing_y -= 10;
                        composing_x = 117;
                    }
                }
            }

            // send a character
            else if(toLetter(currentButton) == '+') {
                SendMessage();
            }

            // add a character
            else {
                char letter;
                letter = toLetter(currentButton);

                if(bufferSize < MAX_BUFFER) {

                    // consecutive button for switching character
                    if(previousPress == currentPress && sameButton) {
                        int index = bufferSize - 1;
                        char l = ComposingLetter[index].letter;

                        // clear previous letter
                        drawChar(ComposingLetter[index].x, ComposingLetter[index].y, l, BLACK, BLACK, 1);

                        // draw the next letter
                        ComposingLetter[index].letter = forwardLetter(l, currentButton);
                        drawChar(ComposingLetter[index].x, ComposingLetter[index].y, ComposingLetter[index].letter, BLUE, BLUE, 1);
                    }

                    else {
                        Letter CL;
                        CL.x = composing_x;
                        CL.y = composing_y;
                        CL.letter = letter;
                        ComposingLetter[bufferSize] = CL;

                        drawChar(composing_x, composing_y, letter, BLUE, BLUE, 1);

                        // adjust pixel positions
                        composing_x += 7;
                        if(composing_x >= 124) {
                            composing_x = 5;
                            composing_y += 10;
                        }

                        // increase buffer size for next input
                        bufferSize++;
                    }
                }
            }
            // update press flag and button
            previousPress = currentPress;
            previousButton = currentButton;
        }
    }

}

static void
BoardInit(void) {
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//!
//! \return None.
//
//****************************************************************************
int main() {
    unsigned long ulStatus;

    BoardInit();
    
    PinMuxConfig();
    
    SetupCommunication();

    InitTerm();

    ClearTerm();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Reset SPI
    MAP_SPIReset(GSPI_BASE);

    MAP_PRCMPeripheralReset(PRCM_GSPI);

    // set up Timer interrupt
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, ConsecutivePressingHandler);

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, SampleCollectHandler);

    MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, A0TICK);
    ulStatus = MAP_TimerIntStatus(TIMERA0_BASE, false);
    MAP_TimerIntClear(TIMERA0_BASE, ulStatus);
    MAP_TimerIntEnable(TIMERA0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);

    // clear global variables
    currentPress = 0;
    previousPress = 1;
    currentButton = -2;
    previousButton = -1;

    // Display banner on UART
    Message("\t****************************************************\n\r");
    Message("\tPress a button on the Remote to see which button you pressed\n\r");
    Message("\t****************************************************\n\r");
    Message("\n\n\n\r");

    MasterMain();
    fillScreen(BLACK);

    // UART interrupt
    UARTIntEnable( UARTA1_BASE, UART_INT_RX) ;
    UARTIntRegister(UARTA1_BASE, CheckMessage);

    // main for loop
    while (1) {
        if(isSampling) {
            isSampling = false;
            buffer[sample_size - 1] = ((signed long) ReadADC()) - 372;
        }
        CollectData();
        DisplayMessage();
    }
}
