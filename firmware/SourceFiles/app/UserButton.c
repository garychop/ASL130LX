//------------------------------------------------------------------------------
// Filename: UserButton.c
//
// Description: BSP level and application level functions for processing
//      the User Buttons.
//
// Author(s): G. Chopcinski (Kg Solutions, LLC)
//
// Created for ASL on Date: 6/22/2020
//
//------------------------------------------------------------------------------
// NOTE: This must ALWAYS be the first include in a file.
#include "device_xc8.h"

#include "bsp.h"
#include "UserButton.h"

//------------------------------------------------------------------------------
// Macros and defines


#define MAX_DEBOUNCE (8)

#ifdef FUTURE_WORK
typedef enum {
    CALIBRATION_BUTTON,
    USER_PORT_BUTTON,
    MODE_PORT_BUTTON,
    MAX_BUTTONS
} BUTTON_ENUM;

typedef struct {
    uint16_t m_PortBit;
    uint8_t m_DebounceCounter;
    bool  m_ButtonState;
} BUTTON_STRUCT;

#endif // FUTURE_WORK

//------------------------------------------------------------------------------
// Local Variables

//BUTTON_STRUCT g_ButtonInfo [MAX_BUTTONS];

static uint8_t g_CalButton_DebounceCounter;
static bool g_CalButtonState;

static uint8_t g_UserPort_DebounceCounter;
static bool g_UserPort_State;

static uint8_t g_ModeButton_DebounceCounter;
static bool g_ModeButton_State;

//------------------------------------------------------------------------------
// Forward Prototype Declarations

//------------------------------------------------------------------------------
// Initialize the User Button inputs as Digital Inputs.
//------------------------------------------------------------------------------

void UserButtonInit(void)
{
    // Initialize the Calibration Button input
    TRISBbits.TRISB2 = GPIO_BIT_INPUT;
    g_CalButton_DebounceCounter = 0;
    g_CalButtonState = PORTBbits.RB2;
    
    //Initialize the Mode Button on the Joystick
    TRISBbits.TRISB0 = GPIO_BIT_INPUT;
    g_ModeButton_DebounceCounter = 0;
    g_ModeButton_State = PORTBbits.RB0;
    
#ifndef DEBUG
    // Setup USER PORT as input.
    // Unfortunately, PIN RB6 is shared with PGD programming pin
    TRISBbits.TRISB7 = GPIO_BIT_INPUT;  // USER PORT
    g_UserPort_State = PORTBbits.RB7;
    g_UserPort_DebounceCounter = 0;
#else
    g_UserPort_State = true;    // 1=Switch is Open, 0=Switch is closed

#endif    
}

//------------------------------------------------------------------------------
// Functions returns true if the Calibration Button is pressed.
bool IsCalibrationButtonActive (void)
{
//#ifndef DEBUG
    return (g_CalButtonState ? false : true);   // Closed is active low.
//#else
//    return false;
//#endif
}

//------------------------------------------------------------------------------

bool IsUserPortButtonActive (void)
{
    return (g_UserPort_State ? false : true);   // Closed is active low.
}

//------------------------------------------------------------------------------

bool IsModeButtonActive (void)
{
    return (g_ModeButton_State ? false : true);     // Closed is active low
}

//------------------------------------------------------------------------------

void Read_User_Buttons (void)
{
    // Read and debounce the Calibration Button
    if (PORTBbits.RB2 == g_CalButtonState)
    {
        // Nothing to do, the signal is stable.
        g_CalButton_DebounceCounter = 0;
    }
    else
    {
        ++g_CalButton_DebounceCounter;
        if (g_CalButton_DebounceCounter > MAX_DEBOUNCE)
        {
            g_CalButtonState = PORTBbits.RB2;
            g_CalButton_DebounceCounter = 0;
        }
    }

    // Read and debounce the Mode Button on the Joystick.
    if (PORTBbits.RB0 == g_ModeButton_State)
    {
        // Nothing to do, the signal is stable.
        g_ModeButton_DebounceCounter = 0;
    }
    else
    {
        ++g_ModeButton_DebounceCounter;
        if (g_ModeButton_DebounceCounter > MAX_DEBOUNCE)
        {
            g_ModeButton_State = PORTBbits.RB0;
            g_ModeButton_DebounceCounter = 0;
        }
    }
    
#ifndef DEBUG
    if (PORTBbits.RB7 == g_UserPort_State)
    {
        // Nothing to do, the signal is stable.
        g_UserPort_DebounceCounter = 0;
    }
    else
    {
        ++g_UserPort_DebounceCounter;
        if (g_UserPort_DebounceCounter > MAX_DEBOUNCE)
        {
            g_UserPort_State = PORTBbits.RB7;
            g_UserPort_DebounceCounter = 0;
        }
    }
#endif    
}

