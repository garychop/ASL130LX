///////////////////////////////////////////////////////////////////////////////
//                    	   Prop ASL130 with Bluetooth Module
//                 		by changing value of MULT and CENTER
//				   PC Board version PROP_BT_V2/3
//  This version fix the issue of joystick cable disconnected from PC board
///////////////////////////////////////////////////////////////////////////////

//#include <18f4550.H>
#include <xc.h>

// NOTE: This must ALWAYS be the first include in a file.
#include "device_xc8.h"

// from stdlib
#include <stdint.h>
#include <stdbool.h>
//#include "user_assert.h"

/* **************************   Header Files   *************************** */

#include "bsp.h"
#include "beeper.h"
#include "dac_bsp.h"
#include "eeprom_bsp.h"
#include "AnalogInput.h"

/* ******************************   Macros   ****************************** */

#define NEUTRAL_DEMAND_OUTPUT (1115)    // "1115" sets output voltage to 1.62 which
                                        // .. is Neutral Demand for LiNX TPI
                                        // 1115D = 0x45b
#define MAX_DAC_OUTPUT (1700)           // 2.54 Volts * 0.00146 volts / bit
#define MIN_DAC_OUTPUT (520)            // 0.76 Volts * 0.00146 volts / bit

#define NEUTRAL_JOYSTICK_INPUT (0x202)
#define NEUTRAL_ERROR_MARGIN (0x10)     // The amount of deviation from the Neutral
#define JOYSTICK_RAW_MAX_DEFLECTION (220)   // This is the max that the joystick 
                                        // .. input can deviate from neutral.

enum JOYSTICK_CHANNEL_ENUM {SPEED_ARRAY, DIRECTION_ARRAY, NUM_JS_POTS};

enum STATE_ENUM {
    NO_STATE = 0,
    POWERUP_STATE,
    DRIVING_STATE,
    BLUETOOTH_STATE,
    CALIBRATION_STATE,
}  gp_State;

/* ***********************   Function Prototypes   ************************ */

static void DriveDemand (void);
static void BluetoothControl (void);
static void SetTPI_Demands (uint16_t speedDemand, uint16_t directionDemand);
static void GetJoystickNeutral(void);

/* ***********************   Global Variables ***************************** */

uint8_t xVal, yVal;
bool eepromStatus;
uint16_t g_RawSpeedInput;
uint16_t g_RawDirectionInput;

typedef struct
{
    uint16_t m_rawInput;
    uint16_t m_rawNeutral;
    uint16_t m_rawMinNeutral;
    uint16_t m_rawMaxNuetral;
} JOYSTICK_STRUCT;
JOYSTICK_STRUCT Joystick_Data[NUM_JS_POTS];

//------------------------------------------------------------------------------

int main (void)
{
    int i;
    
	UTRDIS = 1; 						//	USB transceiver disable 
    bspInitCore();
    beeperInit();
    dacBspInit();
    eepromBspInit();
    AnalogInputInit();

#ifdef WEBEREADINTHEEEPROMP    
    // Try reading the EEPROM
    eepromStatus = false;
    eepromBspWriteByte (0, 0xde, 100);
    eepromBspWriteByte (1, 0xad, 100);
    eepromStatus = eepromBspReadSection (0, 1, &xVal, 100);
    eepromStatus = eepromBspReadSection (1, 1, &yVal, 100);
    eepromBspWriteByte (0, 0xaa, 100);
    eepromBspWriteByte (1, 0x55, 100);
    eepromStatus = eepromBspReadSection (0, 1, &xVal, 100);
    eepromStatus = eepromBspReadSection (1, 1, &yVal, 100);
#endif
    dacBspSet (DAC_SELECT_FORWARD_BACKWARD, NEUTRAL_DEMAND_OUTPUT);
    dacBspSet (DAC_SELECT_LEFT_RIGHT, NEUTRAL_DEMAND_OUTPUT);

    // Short breather to allow board to power up normally.
    for (i = 0; i < 2000; ++i)
        bspDelayUs (US_DELAY_500_us);
    // Announce the startup
//    TurnBeeper(BEEPER_ON);
    for (i = 0; i < 50; ++i)
        bspDelayUs (US_DELAY_500_us);
    TurnBeeper(BEEPER_OFF);
    
    dacBspSet (DAC_SELECT_FORWARD_BACKWARD, NEUTRAL_DEMAND_OUTPUT);
    dacBspSet (DAC_SELECT_LEFT_RIGHT, NEUTRAL_DEMAND_OUTPUT);

    gp_State = POWERUP_STATE;
    
    while (1)
    {
        
        switch (gp_State)
        {
            case POWERUP_STATE:
                GetJoystickNeutral();
                break;
            case DRIVING_STATE:
                DriveDemand();
                break;
            case BLUETOOTH_STATE:
                break;
            case CALIBRATION_STATE:
                break;
            default:
                break;
        }
    }
}

//------------------------------------------------------------------------------
// This function is executed at power and determines the Neutral Window and
// joystick's input Upper and Lower Limits.
//------------------------------------------------------------------------------

static void GetJoystickNeutral(void)
{
    uint16_t speed, direction;
    
    // Get the Speed and Direction
    GetSpeedAndDirection (&speed, &direction);
        
    // Check for the Speed and Direction Analog Inputs to be in the Neutral Window
    if ((speed < (NEUTRAL_JOYSTICK_INPUT + NEUTRAL_ERROR_MARGIN))
    && (speed > (NEUTRAL_JOYSTICK_INPUT - NEUTRAL_ERROR_MARGIN))
    && (direction < (NEUTRAL_JOYSTICK_INPUT + NEUTRAL_ERROR_MARGIN))
    && (direction > (NEUTRAL_JOYSTICK_INPUT - NEUTRAL_ERROR_MARGIN)))
    {
        // Setup Speed Neutral Window and Limits
        Joystick_Data[SPEED_ARRAY].m_rawInput = speed;
        Joystick_Data[SPEED_ARRAY].m_rawNeutral = speed;
        Joystick_Data[SPEED_ARRAY].m_rawMinNeutral = speed - NEUTRAL_ERROR_MARGIN;
        Joystick_Data[SPEED_ARRAY].m_rawMaxNuetral = speed + NEUTRAL_ERROR_MARGIN;

        // Setup Direction Neutral Window and Limits
        Joystick_Data[DIRECTION_ARRAY].m_rawInput = direction;
        Joystick_Data[DIRECTION_ARRAY].m_rawNeutral = direction;
        Joystick_Data[DIRECTION_ARRAY].m_rawMinNeutral = direction - NEUTRAL_ERROR_MARGIN;
        Joystick_Data[DIRECTION_ARRAY].m_rawMaxNuetral = direction + NEUTRAL_ERROR_MARGIN;
        
        gp_State = DRIVING_STATE;
    }
            
}

//------------------------------------------------------------------------------
// This function translates the Joystick's Speed and Direction Analog Input
// signals into the voltage expected by the LiNX TPI board.
// In the calculations below, here's what the hard-coded number represent.
//  "240.0f" is the expected max deflection of the Analog Inputs from Neutral.
//  "630.0f" is the factor to convert the Analog Inputs to DAC bits.
//------------------------------------------------------------------------------


static void DriveDemand (void)
{
    uint16_t rawSpeed, rawDirection;
    float offset, demand;
    uint16_t int_SpeedDemand, int_DirectionDemand; 
    
    GetSpeedAndDirection (&rawSpeed, &rawDirection);
    
    int_SpeedDemand = NEUTRAL_DEMAND_OUTPUT;
    int_DirectionDemand = NEUTRAL_DEMAND_OUTPUT;
    
    // Process the Joystick Speed signal
    if (rawSpeed > Joystick_Data[SPEED_ARRAY].m_rawMaxNuetral)
    {
        demand = (float) NEUTRAL_DEMAND_OUTPUT; 
        offset = rawSpeed - Joystick_Data[SPEED_ARRAY].m_rawNeutral;
        offset = (offset / 240.0f) * 630.0f;
        demand += offset;
        int_SpeedDemand = (uint16_t) demand;
    }
    else if (rawSpeed < Joystick_Data[SPEED_ARRAY].m_rawMinNeutral)
    {
        demand = (float) NEUTRAL_DEMAND_OUTPUT; 
        offset = Joystick_Data[SPEED_ARRAY].m_rawNeutral - rawSpeed;
        offset = (offset / 240.0f) * 630.0f;
        demand -= offset;
        int_SpeedDemand = (uint16_t) demand;
    }
    // Process the Joystick Directional signal
    if (rawDirection > Joystick_Data[DIRECTION_ARRAY].m_rawMaxNuetral)
    {
        demand = (float) NEUTRAL_DEMAND_OUTPUT; 
        offset = rawDirection - Joystick_Data[DIRECTION_ARRAY].m_rawNeutral;
        offset = (offset / 240.0f) * 630.0f;
        demand += offset;
        int_DirectionDemand = (uint16_t) demand;
    }
    else if (rawDirection < Joystick_Data[DIRECTION_ARRAY].m_rawMinNeutral)
    {
        demand = (float) NEUTRAL_DEMAND_OUTPUT; 
        offset = Joystick_Data[DIRECTION_ARRAY].m_rawNeutral - rawDirection;
        offset = (offset / 240.0f) * 630.0f;
        demand -= offset;
        int_DirectionDemand = (uint16_t) demand;
    }
    
    SetTPI_Demands (int_SpeedDemand, int_DirectionDemand);
}

//------------------------------------------------------------------------------
// This functions sends the Demands to the TPI board via the DAC's.
// Also perform a min and max text.
//------------------------------------------------------------------------------

static void SetTPI_Demands (uint16_t speedDemand, uint16_t directionDemand)
{
    uint16_t mySpeed, myDirection;
    
    mySpeed = speedDemand;
    if (mySpeed > MAX_DAC_OUTPUT)
        mySpeed = MAX_DAC_OUTPUT;
    if (mySpeed < MIN_DAC_OUTPUT)
        mySpeed = MIN_DAC_OUTPUT;
    
    myDirection = directionDemand;
    if (myDirection > MAX_DAC_OUTPUT)
        myDirection = MAX_DAC_OUTPUT;
    if (myDirection < MIN_DAC_OUTPUT)
        myDirection = MIN_DAC_OUTPUT;

    dacBspSet (DAC_SELECT_FORWARD_BACKWARD, mySpeed);
    dacBspSet (DAC_SELECT_LEFT_RIGHT, myDirection);
}

