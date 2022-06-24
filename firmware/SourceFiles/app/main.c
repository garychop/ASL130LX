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
#include "DigitalOutput.h"
#include "dac_bsp.h"
#include "eeprom_bsp.h"
#include "AnalogInput.h"
#include "UserButton.h"
#include "BluetoothControl.h"

/* ******************************   Macros   ****************************** */

#define NEUTRAL_DEMAND_OUTPUT (1115)    // "1115" sets output voltage to 1.62 which
                                        // .. is Neutral Demand for LiNX TPI
                                        // 1115D = 0x45b
#define MAX_DAC_OUTPUT (1700)           // 2.54 Volts * 0.00146 volts / bit
#define MIN_DAC_OUTPUT (520)            // 0.76 Volts * 0.00146 volts / bit


enum JOYSTICK_CHANNEL_ENUM {SPEED_ARRAY, DIRECTION_ARRAY, NUM_JS_POTS};

enum STATE_ENUM {
    NO_STATE = 0,
    POWERUP_STATE,
    ENTER_DRIVING_STATE,
    DRIVING_STATE,
    ENTER_BLUETOOTH_STATE,
    BLUETOOTH_STATE,
    EXIT_BLUETOOTH_STATE,
    ENTER_MODE_CHANGE_STATE,
    MODE_CHANGE_STATE,
    EXIT_MODE_CHANGE_STATE,
    ENTER_CALIBRATION_STATE,
    DO_JOYSTICK_CALIBRATION_STATE,
    EXIT_JOYSTICK_CALIBRATION_STATE,
}  gp_State;

// Define the locations in EEPROM
#define EEPROM_SPEED_LOWER_SCALE 0
#define EEPROM_SPEED_UPPER_SCALE (EEPROM_SPEED_LOWER_SCALE + 2)
#define EEPROM_DIRECTION_LOWER_SCALE (EEPROM_SPEED_UPPER_SCALE + 2)
#define EEPROM_DIRECTION_UPPER_SCALE (EEPROM_DIRECTION_LOWER_SCALE + 2)

/* ***********************   Function Prototypes   ************************ */

static void EnterDrivingState (void);
static void DrivingState (void);
static void EnterBluetoothState (void);
static void BluetoothControlState (void);
static void EnterModeChangeState (void);
static void ModeChangeState (void);
static void ExitModeChangeState (void);

static void EnterCalibrationState(void);
static void JoystickCalibrationState(void);
static void ExitCalibrationState(void);

static void SetTPI_Demands (uint16_t speedDemand, uint16_t directionDemand);
void InitializeJoystickData (void);
static void EstablishJoystickNeutral(void);

/* ***********************   Global Variables ***************************** */

// This holds the Joystick information 
typedef struct 
{
    uint16_t m_rawInput;
    uint16_t m_rawNeutral;
    uint16_t m_rawMinNeutral;
    uint16_t m_rawMaxNuetral;
    uint16_t m_rawMinimum;
    uint16_t m_rawMaximum;
    uint16_t m_PositiveScale;   // This is used to scale from neutral to Most Positive
    uint16_t m_NegativeScale;   // This is used to scale from neutral to Most Negative
} JOYSTICK_STRUCT;
JOYSTICK_STRUCT Joystick_Data[NUM_JS_POTS];

//------------------------------------------------------------------------------

int main (void)
{
    int i;
    
	UTRDIS = 1; 						//	USB transceiver disable 
    bspInitCore();
    beeperInit();
    DigitalOutputInit();
    dacBspInit();
    eepromBspInit();
    BluetoothControlInit();
    AnalogInputInit();
    UserButtonInit();
    
    dacBspSet (DAC_SELECT_FORWARD_BACKWARD, NEUTRAL_DEMAND_OUTPUT);
    dacBspSet (DAC_SELECT_LEFT_RIGHT, NEUTRAL_DEMAND_OUTPUT);

    // Short breather to allow board to power up normally.
    for (i = 0; i < 2000; ++i)
        bspDelayUs (US_DELAY_500_us);

    InitializeJoystickData();

    // Announce the startup
//    TurnBeeper(BEEPER_ON);
    for (i = 0; i < 50; ++i)
    {
        Read_User_Buttons();  // Get and debounce the User Buttons.
        bspDelayUs (US_DELAY_500_us);
    }
    TurnBeeper(BEEPER_OFF);
    
    dacBspSet (DAC_SELECT_FORWARD_BACKWARD, NEUTRAL_DEMAND_OUTPUT);
    dacBspSet (DAC_SELECT_LEFT_RIGHT, NEUTRAL_DEMAND_OUTPUT);

    gp_State = POWERUP_STATE;

    while (1)
    {
        Read_User_Buttons();  // Get and debounce the User Buttons.

        switch (gp_State)
        {
            case POWERUP_STATE:
                EstablishJoystickNeutral();
                break;
            case ENTER_DRIVING_STATE:
                EnterDrivingState();
                break;
            case DRIVING_STATE:
                DrivingState();
                break;
            case ENTER_BLUETOOTH_STATE:
                EnterBluetoothState();
                break;
            case BLUETOOTH_STATE:
                BluetoothControlState();
                break;
            case ENTER_MODE_CHANGE_STATE:
                EnterModeChangeState();
                break;
            case MODE_CHANGE_STATE:
                ModeChangeState();
                break;
            case EXIT_MODE_CHANGE_STATE:
                ExitModeChangeState();
                break;
            case ENTER_CALIBRATION_STATE:
                EnterCalibrationState();
                break;
            case DO_JOYSTICK_CALIBRATION_STATE:
                JoystickCalibrationState();
                break;
            case EXIT_JOYSTICK_CALIBRATION_STATE:
                ExitCalibrationState();
                break;
            default:
                break;
        }
        bspDelayUs (US_DELAY_200_us);
    }
}

//------------------------------------------------------------------------------
// This function waits for the Joystick to be in neutral and no buttons are
// active.
//------------------------------------------------------------------------------

static void EnterDrivingState (void)
{
    int i;
    
    //TurnBeeper(BEEPER_ON);
    for (i=0; i<100; ++i)
        bspDelayUs (US_DELAY_200_us);
    TurnBeeper(BEEPER_OFF);

    if (IsJoystickInNeutral())
    {
        if (IsUserPortButtonActive() == false)
        {
            if (IsCalibrationButtonActive() == false)
            {
                gp_State = DRIVING_STATE;
            }
        }
    }
}

//------------------------------------------------------------------------------
// This function translates the Joystick's Speed and Direction Analog Input
// signals into the voltage expected by the LiNX TPI board.
// In the calculations below, here's what the hard-coded number represent.
//  "240.0f" is the expected max deflection of the Analog Inputs from Neutral.
//  "630.0f" is the factor to convert the Analog Inputs to DAC bits.
//------------------------------------------------------------------------------

static void DrivingState (void)
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
        if (rawSpeed > (Joystick_Data[SPEED_ARRAY].m_PositiveScale + Joystick_Data[SPEED_ARRAY].m_rawNeutral))
            rawSpeed = Joystick_Data[SPEED_ARRAY].m_PositiveScale + Joystick_Data[SPEED_ARRAY].m_rawNeutral;
        demand = (float) NEUTRAL_DEMAND_OUTPUT; 
        offset = rawSpeed - Joystick_Data[SPEED_ARRAY].m_rawNeutral;
        offset = (offset / (float)Joystick_Data[SPEED_ARRAY].m_PositiveScale) * 630.0f;
        demand += offset;
        int_SpeedDemand = (uint16_t) demand;
    }
    else if (rawSpeed < Joystick_Data[SPEED_ARRAY].m_rawMinNeutral)
    {
        if (rawSpeed < (Joystick_Data[SPEED_ARRAY].m_rawNeutral - Joystick_Data[SPEED_ARRAY].m_NegativeScale))
            rawSpeed = Joystick_Data[SPEED_ARRAY].m_rawNeutral - Joystick_Data[SPEED_ARRAY].m_NegativeScale;
        demand = (float) NEUTRAL_DEMAND_OUTPUT; 
        offset = Joystick_Data[SPEED_ARRAY].m_rawNeutral - rawSpeed;
        offset = (offset / (float) Joystick_Data[SPEED_ARRAY].m_NegativeScale) * 630.0f;
        demand -= offset;
        int_SpeedDemand = (uint16_t) demand;
    }
    // Process the Joystick Directional signal
    if (rawDirection > Joystick_Data[DIRECTION_ARRAY].m_rawMaxNuetral)
    {
        // Check to see if the joystick is past the calibrated value.
        if (rawDirection > (Joystick_Data[DIRECTION_ARRAY].m_PositiveScale + Joystick_Data[DIRECTION_ARRAY].m_rawNeutral))
            rawDirection = Joystick_Data[DIRECTION_ARRAY].m_PositiveScale + Joystick_Data[DIRECTION_ARRAY].m_rawNeutral;
        demand = (float) NEUTRAL_DEMAND_OUTPUT; 
        offset = rawDirection - Joystick_Data[DIRECTION_ARRAY].m_rawNeutral;
        offset = (offset / (float)Joystick_Data[DIRECTION_ARRAY].m_PositiveScale) * 630.0f;
        demand += offset;
        int_DirectionDemand = (uint16_t) demand;
    }
    else if (rawDirection < Joystick_Data[DIRECTION_ARRAY].m_rawMinNeutral)
    {
        // Check to see if the joystick is past the calibrated value.
        if (rawDirection < (Joystick_Data[DIRECTION_ARRAY].m_rawNeutral - Joystick_Data[DIRECTION_ARRAY].m_NegativeScale))
            rawDirection = Joystick_Data[DIRECTION_ARRAY].m_rawNeutral - Joystick_Data[DIRECTION_ARRAY].m_NegativeScale;
        demand = (float) NEUTRAL_DEMAND_OUTPUT; 
        offset = Joystick_Data[DIRECTION_ARRAY].m_rawNeutral - rawDirection;
        offset = (offset / (float)Joystick_Data[DIRECTION_ARRAY].m_NegativeScale) * 630.0f;
        demand -= offset;
        int_DirectionDemand = (uint16_t) demand;
    }
    
    // Shall we do some calibration?
    if (IsCalibrationButtonActive())
    {
        gp_State = ENTER_CALIBRATION_STATE;
        // Let's stop driving if we are.
        int_SpeedDemand = NEUTRAL_DEMAND_OUTPUT;
        int_DirectionDemand = NEUTRAL_DEMAND_OUTPUT;
    }
    
    // Shall we change to Bluetooth Mode?
    if (IsUserPortButtonActive())
    {
        gp_State = ENTER_BLUETOOTH_STATE;
        // Let's stop driving if we are.
        int_SpeedDemand = NEUTRAL_DEMAND_OUTPUT;
        int_DirectionDemand = NEUTRAL_DEMAND_OUTPUT;
    }
    
    // Shall we change Modes
    if (IsModeButtonActive())
    {
        gp_State = ENTER_MODE_CHANGE_STATE;
        // Let's stop driving if we are.
        int_SpeedDemand = NEUTRAL_DEMAND_OUTPUT;
        int_DirectionDemand = NEUTRAL_DEMAND_OUTPUT;
    }

    // Send joystick demands to the TPI board.
    SetTPI_Demands (int_SpeedDemand, int_DirectionDemand);
}

//------------------------------------------------------------------------------

static void EnterBluetoothState (void)
{
    TurnBeeper(BEEPER_ON);
    if (IsUserPortButtonActive() == false)
    {
        TurnBeeper(BEEPER_OFF);
        EnableBluetooth();
        gp_State = BLUETOOTH_STATE;
    }
}

//------------------------------------------------------------------------------

static void BluetoothControlState (void)
{
    uint16_t rawSpeed, rawDirection;

    if (IsUserPortButtonActive())
    {
        DisableBluetooth();
        gp_State = ENTER_DRIVING_STATE; // This checks for neutral and no switches
                                        // ... before allowing to drive
    }
    if (IsJoystickInNeutral())
    {
        SendBlueToothSignal (GPIO_HIGH, FWD_BT);
        SendBlueToothSignal (GPIO_HIGH, REV_BT);
        SendBlueToothSignal (GPIO_HIGH, LEFT_BT);
        SendBlueToothSignal (GPIO_HIGH, RIGHT_BT);
    }
    else
    {
        // Determine which joystick direction is active and send signal
        // to Bluetooth module.
        GetSpeedAndDirection (&rawSpeed, &rawDirection);
        // Process SPEED demand
        if (rawSpeed > (Joystick_Data[SPEED_ARRAY].m_rawNeutral + (Joystick_Data[SPEED_ARRAY].m_PositiveScale / 2)))
        {
            // Forward is active
            SendBlueToothSignal (GPIO_LOW, FWD_BT);
        }
        else if (rawSpeed < (Joystick_Data[SPEED_ARRAY].m_rawNeutral - (Joystick_Data[SPEED_ARRAY].m_NegativeScale / 2)))
        {
            // Reverse is active
            SendBlueToothSignal (GPIO_LOW, REV_BT);
        }
        // Process DIRECTION demand
        if (rawDirection > (Joystick_Data[DIRECTION_ARRAY].m_rawNeutral + (Joystick_Data[DIRECTION_ARRAY].m_PositiveScale / 2)))
        {
            // Right is active
            SendBlueToothSignal (GPIO_LOW, RIGHT_BT);
        }
        else if (rawDirection < (Joystick_Data[DIRECTION_ARRAY].m_rawNeutral - (Joystick_Data[DIRECTION_ARRAY].m_NegativeScale / 2)))
        {
            // Left is active
            SendBlueToothSignal (GPIO_LOW, LEFT_BT);
        }
    }
}

//------------------------------------------------------------------------------

static void EnterModeChangeState (void)
{
    SetResetOutput (GPIO_HIGH);
    gp_State = MODE_CHANGE_STATE;
}

//------------------------------------------------------------------------------

static void ModeChangeState (void)
{
    if (IsModeButtonActive() == false)
    {
        gp_State = EXIT_MODE_CHANGE_STATE;
        SetResetOutput (GPIO_LOW);
    }
}

//------------------------------------------------------------------------------

static void ExitModeChangeState (void)
{
    gp_State = ENTER_DRIVING_STATE; // This checks for neutral and no switches
                                    // ... before allowing to drive
}

//------------------------------------------------------------------------------
// This function prepares the unit for Joystick Calibration
//  - sound the beeper and wait for the button to be released.
//------------------------------------------------------------------------------

static void EnterCalibrationState(void)
{
    TurnBeeper(BEEPER_ON);
    if (IsCalibrationButtonActive() == false)
    {
        TurnBeeper(BEEPER_OFF);
        if (IsJoystickInNeutral())
        {
            // Preset the min and max to very small.
            Joystick_Data[SPEED_ARRAY].m_rawMaximum = Joystick_Data[SPEED_ARRAY].m_rawNeutral;
            Joystick_Data[SPEED_ARRAY].m_rawMinimum = Joystick_Data[SPEED_ARRAY].m_rawNeutral;
            Joystick_Data[DIRECTION_ARRAY].m_rawMaximum = Joystick_Data[DIRECTION_ARRAY].m_rawNeutral;
            Joystick_Data[DIRECTION_ARRAY].m_rawMinimum = Joystick_Data[DIRECTION_ARRAY].m_rawNeutral;
            
            gp_State = DO_JOYSTICK_CALIBRATION_STATE;
        }
    }
}

//------------------------------------------------------------------------------
// This function processes the Joystick Signals looking for the Smallest and 
// largest Speed and Direction ADC values.
// This function changes state when the Calibration Button is pressed again
// and then values are stored in the EEPROM.
//------------------------------------------------------------------------------
static void JoystickCalibrationState(void)
{
    uint16_t rawSpeed, rawDirection;
    
    // Get the Joystick's Speed and Direction and seek the lowest and highest
    // signals.
    GetSpeedAndDirection (&rawSpeed, &rawDirection);
    
    if (rawSpeed > Joystick_Data[SPEED_ARRAY].m_rawMaximum)
        Joystick_Data[SPEED_ARRAY].m_rawMaximum = rawSpeed;
    if (rawSpeed < Joystick_Data[SPEED_ARRAY].m_rawMinimum)
        Joystick_Data[SPEED_ARRAY].m_rawMinimum = rawSpeed;
    
    if (rawDirection > Joystick_Data[DIRECTION_ARRAY].m_rawMaximum)
        Joystick_Data[DIRECTION_ARRAY].m_rawMaximum = rawDirection;
    if (rawDirection < Joystick_Data[DIRECTION_ARRAY].m_rawMinimum)
        Joystick_Data[DIRECTION_ARRAY].m_rawMinimum = rawDirection;
    
    // Check to see if we want to exit the procedure.
    if (IsCalibrationButtonActive())
    {
        // Calculate the scales and store them in EEPROM.
        Joystick_Data[SPEED_ARRAY].m_PositiveScale = Joystick_Data[SPEED_ARRAY].m_rawMaximum - Joystick_Data[SPEED_ARRAY].m_rawNeutral;
        Joystick_Data[SPEED_ARRAY].m_NegativeScale = Joystick_Data[SPEED_ARRAY].m_rawNeutral - Joystick_Data[SPEED_ARRAY].m_rawMinimum;
        Joystick_Data[DIRECTION_ARRAY].m_PositiveScale = Joystick_Data[DIRECTION_ARRAY].m_rawMaximum - Joystick_Data[DIRECTION_ARRAY].m_rawNeutral;
        Joystick_Data[DIRECTION_ARRAY].m_NegativeScale = Joystick_Data[DIRECTION_ARRAY].m_rawNeutral - Joystick_Data[DIRECTION_ARRAY].m_rawMinimum;
                
        EEPROM_writeInt16 (EEPROM_SPEED_LOWER_SCALE, Joystick_Data[SPEED_ARRAY].m_NegativeScale);
        EEPROM_writeInt16 (EEPROM_SPEED_UPPER_SCALE, Joystick_Data[SPEED_ARRAY].m_PositiveScale);

        EEPROM_writeInt16 (EEPROM_DIRECTION_LOWER_SCALE, Joystick_Data[DIRECTION_ARRAY].m_NegativeScale);
        EEPROM_writeInt16 (EEPROM_DIRECTION_UPPER_SCALE, Joystick_Data[DIRECTION_ARRAY].m_PositiveScale);
        
        TurnBeeper(BEEPER_ON);

        gp_State = EXIT_JOYSTICK_CALIBRATION_STATE;
    }
    
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

static void ExitCalibrationState(void)
{
    if (IsCalibrationButtonActive() == false)
    {
        TurnBeeper(BEEPER_OFF);
        gp_State = POWERUP_STATE;   // This will re-establish the "smart neutral" window.
    }
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

//------------------------------------------------------------------------------
// This function is executed at power and determines the Neutral Window and
// joystick's input Upper and Lower Limits.
//------------------------------------------------------------------------------

static void EstablishJoystickNeutral(void)
{
    uint16_t speed, direction;
    
    if (IsJoystickInNeutral())
    {
        GetSpeedAndDirection (&speed, &direction);
        
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
        
        gp_State = ENTER_DRIVING_STATE;
    }
            
}

//------------------------------------------------------------------------------
// This function sets the Joystick data information. Some is retrieved
// from the EEPROM, i.e. Max Limits.
//------------------------------------------------------------------------------

void InitializeJoystickData (void)
{
    Joystick_Data[SPEED_ARRAY].m_rawInput = NEUTRAL_JOYSTICK_INPUT;
    Joystick_Data[SPEED_ARRAY].m_rawNeutral = NEUTRAL_JOYSTICK_INPUT;
    Joystick_Data[SPEED_ARRAY].m_rawMinNeutral = NEUTRAL_JOYSTICK_INPUT - NEUTRAL_ERROR_MARGIN;
    Joystick_Data[SPEED_ARRAY].m_rawMaxNuetral = NEUTRAL_JOYSTICK_INPUT + NEUTRAL_ERROR_MARGIN;
    Joystick_Data[SPEED_ARRAY].m_rawMinimum = NEUTRAL_JOYSTICK_INPUT - JOYSTICK_RAW_MAX_DEFLECTION;
    Joystick_Data[SPEED_ARRAY].m_rawMaximum = NEUTRAL_JOYSTICK_INPUT + JOYSTICK_RAW_MAX_DEFLECTION;
    Joystick_Data[SPEED_ARRAY].m_PositiveScale = JOYSTICK_RAW_MAX_DEFLECTION;
    Joystick_Data[SPEED_ARRAY].m_NegativeScale = JOYSTICK_RAW_MAX_DEFLECTION;

    Joystick_Data[DIRECTION_ARRAY].m_rawInput = NEUTRAL_JOYSTICK_INPUT;
    Joystick_Data[DIRECTION_ARRAY].m_rawNeutral = NEUTRAL_JOYSTICK_INPUT;
    Joystick_Data[DIRECTION_ARRAY].m_rawMinNeutral = NEUTRAL_JOYSTICK_INPUT - NEUTRAL_ERROR_MARGIN;
    Joystick_Data[DIRECTION_ARRAY].m_rawMaxNuetral = NEUTRAL_JOYSTICK_INPUT + NEUTRAL_ERROR_MARGIN;
    Joystick_Data[DIRECTION_ARRAY].m_rawMinimum = NEUTRAL_JOYSTICK_INPUT - JOYSTICK_RAW_MAX_DEFLECTION;
    Joystick_Data[DIRECTION_ARRAY].m_rawMaximum = NEUTRAL_JOYSTICK_INPUT + JOYSTICK_RAW_MAX_DEFLECTION;
    Joystick_Data[DIRECTION_ARRAY].m_PositiveScale = JOYSTICK_RAW_MAX_DEFLECTION;
    Joystick_Data[DIRECTION_ARRAY].m_NegativeScale = JOYSTICK_RAW_MAX_DEFLECTION;

    EEPROM_readInt16 (EEPROM_SPEED_LOWER_SCALE, &Joystick_Data[SPEED_ARRAY].m_NegativeScale);
    EEPROM_readInt16 (EEPROM_SPEED_UPPER_SCALE, &Joystick_Data[SPEED_ARRAY].m_PositiveScale);

    EEPROM_readInt16 (EEPROM_DIRECTION_LOWER_SCALE, &Joystick_Data[DIRECTION_ARRAY].m_NegativeScale);
    EEPROM_readInt16 (EEPROM_DIRECTION_UPPER_SCALE, &Joystick_Data[DIRECTION_ARRAY].m_PositiveScale);

    // Perform a sanity check to see if the values in the EEPROM are acceptable.
    if ((Joystick_Data[SPEED_ARRAY].m_PositiveScale > JOYSTICK_RAW_MAX_DEFLECTION) 
    || (Joystick_Data[SPEED_ARRAY].m_PositiveScale < JOYSTICK_RAW_MAX_DEFLECTION / 8))
    {
        Joystick_Data[SPEED_ARRAY].m_PositiveScale = JOYSTICK_RAW_MAX_DEFLECTION;
    }

    if ((Joystick_Data[SPEED_ARRAY].m_NegativeScale > JOYSTICK_RAW_MAX_DEFLECTION) 
    || (Joystick_Data[SPEED_ARRAY].m_NegativeScale < JOYSTICK_RAW_MAX_DEFLECTION / 8))
    {
        Joystick_Data[SPEED_ARRAY].m_NegativeScale = JOYSTICK_RAW_MAX_DEFLECTION;
    }
    if ((Joystick_Data[DIRECTION_ARRAY].m_PositiveScale > JOYSTICK_RAW_MAX_DEFLECTION) 
    || (Joystick_Data[DIRECTION_ARRAY].m_PositiveScale < JOYSTICK_RAW_MAX_DEFLECTION / 8))
    {
        Joystick_Data[DIRECTION_ARRAY].m_PositiveScale = JOYSTICK_RAW_MAX_DEFLECTION;
    }

    if ((Joystick_Data[DIRECTION_ARRAY].m_NegativeScale > JOYSTICK_RAW_MAX_DEFLECTION) 
    || (Joystick_Data[DIRECTION_ARRAY].m_NegativeScale < JOYSTICK_RAW_MAX_DEFLECTION / 8))
    {
        Joystick_Data[DIRECTION_ARRAY].m_NegativeScale = JOYSTICK_RAW_MAX_DEFLECTION;
    }
}

