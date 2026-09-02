#include "StepperModule.h"
#include <AccelStepper.h>
#include <avr/interrupt.h>

// ============================================================
// TRIM INDICATOR / NEEDLE 1
// ============================================================

const int TRIM_INDICATOR = 0;

const int TRIM_INDICATOR_HOME_PIN = 14;

// Bij jouw Needle 1 is TRUE de juiste fysieke draairichting.
const bool TRIM_INDICATOR_GESPIEGELD = true;

// Schakelaar wordt iets vóór het echte fysieke nulpunt geraakt.
const long TRIM_INDICATOR_HOME_OFFSET = 70;

// Homing snelheid
const float TRIM_INDICATOR_HOME_SPEED = 500.0;

const long TRIM_INDICATOR_MAX_STEPS = 770;

// ============================================================
// TRIM INDICATOR / NEEDLE 2
// ============================================================

const int TRIM_INDICATOR_2 = 1;

const int TRIM_INDICATOR_2_HOME_PIN = 15;

// Needle 2 draait tegengesteld aan Needle 1
const bool TRIM_INDICATOR_2_GESPIEGELD = false;

// Voorlopig dezelfde offset als Needle 1
const long TRIM_INDICATOR_2_HOME_OFFSET =  10;

const float TRIM_INDICATOR_2_HOME_SPEED = 500.0;
// ============================================================
// TIJDELIJKE KALIBRATIE
// ============================================================
//
// Deze staat nu UIT.
// De kalibratie van 650 stappen is afgerond.
//

const bool TRIM_INDICATOR_CALIBRATION = false;

const long TRIM_INDICATOR_TEST_STEPS = 770;

const float TRIM_INDICATOR_TEST_SPEED = 300.0;
const float TRIM_INDICATOR_TEST_ACCEL = 500.0;

// ============================================================
// TRIM INDICATOR HOME STATE
// ============================================================

enum TrimIndicatorHomeState
{
    TRIM_HOME_IDLE,
    TRIM_HOME_SEARCHING,
    TRIM_HOME_OFFSET,
    TRIM_HOME_DONE
};

TrimIndicatorHomeState trimIndicatorHomeState =
    TRIM_HOME_IDLE;

TrimIndicatorHomeState trimIndicator2HomeState =
    TRIM_HOME_IDLE;

// ============================================================
// TRIM INDICATOR DATA
// ============================================================

// Laatste door SPAD.next ontvangen waarde
float lastTrimIndicatorValue = 0.0;

bool trimIndicatorValueReceived = false;

// Doelpositie in stepper-stappen
long trimIndicatorTarget = 0;

// ===========================================================
// THROTTLE 1 CALIBRATION
// ===========================================================
const bool THROTTLE_1_CALIBRATION = false;

const long THROTTLE_1_TEST_STEPS = 800;

const bool THROTTLE_1_CALIBRATION_DIRECTION = false;

const float THROTTLE_1_TEST_SPEED = 500.0;
const float THROTTLE_1_TEST_ACCEL = 500.0;

const unsigned long THROTTLE_1_HOLD_TIME_MS = 3000;

long throttle1CalibrationTarget = 0;

unsigned long throttle1CalibrationHoldStart = 0;

enum Throttle1CalibrationState
{
    THROTTLE1_CAL_IDLE,
    THROTTLE1_CAL_MOVING_OUT,
    THROTTLE1_CAL_HOLDING,
    THROTTLE1_CAL_RETURNING
};

Throttle1CalibrationState throttle1CalibrationState =
    THROTTLE1_CAL_IDLE;


// ============================================================
// THROTTLE SERVO / A/T DATA
// ============================================================

// true wanneer PMDG de throttle-levers daadwerkelijk door de
// autothrottle-servos laat aansturen.
bool throttleServosActive = false;

// Laatst ontvangen PMDG throttle-posities.
float lastThrottle1Position = 0.0;
float lastThrottle2Position = 0.0;

bool throttle1PositionReceived = false;
bool throttle2PositionReceived = false;

// ============================================================
// THROTTLE RICHTING
// ============================================================
//
// false = positieve stepperrichting is Idle -> Full Throttle
// true  = stepperrichting moet worden gespiegeld
//
// Voor de huidige mechanica laten we beide op false staan.
// De instellingen zijn nu expliciet en later eenvoudig aan te
// passen als een mechanische richting wijzigt.
//
const bool THROTTLE_1_GESPIEGELD = true;
const bool THROTTLE_2_GESPIEGELD = false;

// ============================================================
// THROTTLE 1 MECHANISCHE KALIBRATIE
// ============================================================
//
// fysieke idle = 0 stappen
// fysieke full throttle = 3200 stappen
//
// PMDG meet bij fysieke idle ongeveer 4% en bij full throttle
// ongeveer 85%.
const float THROTTLE_1_PMDG_IDLE = 4.0;
const float THROTTLE_1_PMDG_FULL = 85.0;
const long THROTTLE_1_FULL_STEPS = 3200;

const float THROTTLE_1_AT_SPEED = 3000.0;
const float THROTTLE_1_AT_ACCEL = 1500.0;

// Laatst berekende Throttle 1 stepperpositie.
long throttle1TargetSteps = 0;
long throttle2TargetSteps = 0;

// ============================================================
// THROTTLE 2 MECHANISCHE KALIBRATIE
// ============================================================
const float THROTTLE_2_PMDG_IDLE = 4.0;
const float THROTTLE_2_PMDG_FULL = 85.0;
const long THROTTLE_2_FULL_STEPS = 3200;

const float THROTTLE_2_AT_SPEED = 3000.0;
const float THROTTLE_2_AT_ACCEL = 1500.0;

// ============================================================
// SPEEDBRAKE
// ============================================================
//
// SPAD.next:
//   SIMCONNECT:SPOILERS HANDLE POSITION
//
// Observed:
//   0.00       = DOWN
//   0.60-0.65  = ARMED
//   ~0.76      = highest observed in normal flight
//   1.00       = landing / full deployment
//
// Mechanical calibration:
//   0     = DOWN
//   400   = 26 mm
//   800   = 60 mm
//   1200  = 94 mm
//   1600  = 130 mm = UP
//
// ============================================================

const float SPEED_BRAKE_ARMED_MIN = 0.60;
const float SPEED_BRAKE_ARMED_MAX = 0.65;

const float SPEED_BRAKE_DEPLOY_THRESHOLD = 0.90;
const float SPEED_BRAKE_RETRACT_THRESHOLD = 0.59;

const long SPEED_BRAKE_DOWN_STEPS = 0;
const long SPEED_BRAKE_UP_STEPS = 1600;

const float SPEED_BRAKE_SPEED = 2500.0;
const float SPEED_BRAKE_ACCEL = 1000.0;

float lastSpeedBrakePosition = 0.0;
bool speedBrakePositionReceived = false;
bool speedBrakeDeployed = false;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================
void startTrimIndicatorCalibration();

void startTrimIndicatorHoming();
void updateTrimIndicatorHoming();

void startTrimIndicator2Homing();
void updateTrimIndicator2Homing();

void updateTrimIndicator(float indicatorValue);

void updateThrottleServos(bool active);
void updateThrottle1(float value);
void updateThrottle2(float value);
long throttlePercentToSteps(float value,
                            float idlePercent,
                            float fullPercent,
                            long fullSteps);
void updateSpeedBrake(float value);
long speedBrakePositionToSteps(float value);

void startThrottle1Calibration();
void updateThrottle1Calibration();

// ============================================================
// TRIM WHEEL
// ============================================================

const float TRIM_WHEEL_SPEED = 10500.0;

const float TRIM_WHEEL_ACCEL = 80000.0;
const float TRIM_WHEEL_DECEL = 50000.0;

bool trimStopping = false;

unsigned long trimStopUpdateTime = 0;

const unsigned long TRIM_STOP_TIMEOUT_MS = 80;

float trimAccumulatedDelta = 0.0;

// ============================================================
// TRIM SCHAAL
// ============================================================

const float TRIM_MIN = -0.2251474735;
const float TRIM_MAX = 0.2251474735;

const float TRIM_STEPS_PER_RAD = 234635.0;

const bool TRIM_WHEEL_GESPIEGELD = true;

// ============================================================
// DEADBAND
// ============================================================

const float TRIM_DEADBAND = 0.000100;

// ============================================================
// PINNEN TRIM WHEEL
// ============================================================

const int TRIM_STEP_PIN = 39;
const int TRIM_DIR_PIN = 40;
const int TRIM_ENA_PIN = 46;

// ============================================================
// STEPPERS
// ============================================================

AccelStepper steppers[NUM_STEPPERS] =
    {
        AccelStepper(AccelStepper::DRIVER, 33, 34), // Trim Needle 1
        AccelStepper(AccelStepper::DRIVER, 35, 36), // Trim Needle 2 (tijdelijk niet verbonden)
        AccelStepper(AccelStepper::DRIVER, 31, 32), // Throttle 2
        AccelStepper(AccelStepper::DRIVER, 29, 30), // Throttle 1
        AccelStepper(AccelStepper::DRIVER, 37, 38), // Speed Brake
        AccelStepper(AccelStepper::DRIVER, 39, 40)  // Trim Wheel
    };

// ============================================================
// NEEDLE 2 - PERMANENT STANDALONE OBJECT
// ============================================================

AccelStepper needle2Standalone(
    AccelStepper::DRIVER,
    35,
    36
);

const int enaPins[NUM_STEPPERS] =
    {
        43, // Trim Needle 1
        44, // Trim Needle 2
        42, // Throttle 2
        41, // Throttle 1
        48, // Speed Brake
        46  // Trim Wheel
    };

// ============================================================
// CONFIGURATIE
// ============================================================

const StepperConfig stepperConfig[NUM_STEPPERS] =
    {
        {10000, 1000.0, 500.0, false}, // Trim needle 1
        {10000, 1000.0, 500.0, false}, // Trim needle 2
        {10000, 1000.0, 500.0, false}, // Throttle 2
        {10000, 1000.0, 500.0, false}, // Throttle 1
        {10000, 1000.0, 500.0, false}, // Speed Brake
        {10000, 1000.0, 500.0, false}  // Trim Wheel
};

// ============================================================
// STATUS
// ============================================================

long targetPositions[NUM_STEPPERS] = {0};

bool stepperEnabled[NUM_STEPPERS] =
    {
        true, // Trim Needle 1
        true, // Trim Needle 2
        false,
        false,
        false,
        false};

// ============================================================
// TRIM WHEEL STATUS
// ============================================================

float oldTrim = 0.0;

bool trimInitialized = false;

volatile bool trimTimerRunning = false;

volatile bool trimStepHigh = false;

volatile bool trimTimerDirectionPositive = false;

volatile long trimWheelPosition = 0;

// Huidige snelheid trim wheel
volatile float trimCurrentSpeed = 0.0;

// Laatste trimdata
unsigned long lastTrimUpdateTime = 0;

// Laatste acceleratie-update
unsigned long lastTrimAccelerationUpdate = 0;

// ============================================================
// TIMER2 FREQUENTIE
// ============================================================
//
// Timer2:
// CPU = 16 MHz
// prescaler = 8
// timer clock = 2 MHz
//
// Twee interrupts per volledige STEP.
//
// ============================================================

void setTrimTimerSpeed(float stepsPerSecond)
{
    if (stepsPerSecond < 1.0)
        stepsPerSecond = 1.0;

    if (stepsPerSecond > TRIM_WHEEL_SPEED)
        stepsPerSecond = TRIM_WHEEL_SPEED;

    float interruptFrequency =
        stepsPerSecond * 2.0;

    float ocr =
        (2000000.0 / interruptFrequency) - 1.0;

    if (ocr < 1.0)
        ocr = 1.0;

    if (ocr > 255.0)
        ocr = 255.0;

    noInterrupts();

    OCR2A = (uint8_t)ocr;

    interrupts();
}

// ============================================================
// TIMER2 START
// ============================================================

void startTrimTimer(bool positiveDirection)
{
    noInterrupts();

    digitalWrite(
        TRIM_DIR_PIN,
        positiveDirection ? HIGH : LOW);

    trimTimerDirectionPositive =
        positiveDirection;

    digitalWrite(
        TRIM_STEP_PIN,
        LOW);

    trimStepHigh = false;

    // Start langzaam
    trimCurrentSpeed = 1000.0;

    float interruptFrequency =
        trimCurrentSpeed * 2.0;

    float ocr =
        (2000000.0 / interruptFrequency) - 1.0;

    if (ocr < 1.0)
        ocr = 1.0;

    if (ocr > 255.0)
        ocr = 255.0;

    OCR2A = (uint8_t)ocr;

    // Timer2 reset
    TCCR2A = 0;
    TCCR2B = 0;

    TCNT2 = 0;

    // CTC mode
    TCCR2A |= (1 << WGM21);

    // Prescaler 8
    TCCR2B |= (1 << CS21);

    // Compare A interrupt
    TIMSK2 |= (1 << OCIE2A);

    trimTimerRunning = true;

    interrupts();

    lastTrimAccelerationUpdate = millis();
}

// ============================================================
// TIMER2 STOP
// ============================================================

void stopTrimTimer()
{
    noInterrupts();

    trimTimerRunning = false;

    trimCurrentSpeed = 0.0;

    // Timer stoppen
    TCCR2B &= ~(
        (1 << CS22) |
        (1 << CS21) |
        (1 << CS20));

    // STEP LOW
    PORTG &= ~(1 << PG2);

    trimStepHigh = false;

    interrupts();
}

// ============================================================
// TIMER2 INTERRUPT
// ============================================================

ISR(TIMER2_COMPA_vect)
{
    if (!trimTimerRunning)
    {
        PORTG &= ~(1 << PG2);

        trimStepHigh = false;

        return;
    }

    // STEP HIGH
    if (!trimStepHigh)
    {
        PORTG |= (1 << PG2);

        trimStepHigh = true;

        if (trimTimerDirectionPositive)
        {
            trimWheelPosition++;
        }
        else
        {
            trimWheelPosition--;
        }
    }
    // STEP LOW
    else
    {
        PORTG &= ~(1 << PG2);

        trimStepHigh = false;
    }
}

// ============================================================
// INIT
// ============================================================

void initSteppers()
{
    // --------------------------------------------------------
    // Trim wheel pinnen
    // --------------------------------------------------------

    pinMode(TRIM_STEP_PIN, OUTPUT);
    pinMode(TRIM_DIR_PIN, OUTPUT);
    pinMode(TRIM_ENA_PIN, OUTPUT);

    digitalWrite(TRIM_STEP_PIN, LOW);
    digitalWrite(TRIM_DIR_PIN, LOW);
    digitalWrite(TRIM_ENA_PIN, HIGH);

    // --------------------------------------------------------
    // Timer2 uit
    // --------------------------------------------------------

    TCCR2A = 0;
    TCCR2B = 0;

    TCNT2 = 0;

    TIMSK2 &= ~(1 << OCIE2A);

    // --------------------------------------------------------
    // Alle steppers initialiseren
    // --------------------------------------------------------

    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        if (i == TRIM_INDICATOR_2)
        {
            continue;
        }

        steppers[i].setEnablePin(
            enaPins[i]);

        steppers[i].setPinsInverted(
            false,
            false,
            true);

        if (i == TRIM_WHEEL)
        {
            steppers[i].setMaxSpeed(
                TRIM_WHEEL_SPEED);

            steppers[i].setAcceleration(
                TRIM_WHEEL_ACCEL);
        }
        else
        {
            steppers[i].setMaxSpeed(
                stepperConfig[i].speed);

            steppers[i].setAcceleration(
                stepperConfig[i].acceleration);
        }

        steppers[i].setCurrentPosition(0);

        steppers[i].disableOutputs();
    }

    // --------------------------------------------------------
    // Trim status
    // --------------------------------------------------------

    oldTrim = 0.0;

    trimInitialized = false;

    trimTimerRunning = false;

    trimStepHigh = false;

    trimTimerDirectionPositive = false;

    trimWheelPosition = 0;

    trimCurrentSpeed = 0.0;

    lastTrimUpdateTime = millis();

    lastTrimAccelerationUpdate = millis();

    // --------------------------------------------------------
    // Trim indicator status
    // --------------------------------------------------------

    lastTrimIndicatorValue = 0.0;

    trimIndicatorValueReceived = false;

    trimIndicatorTarget = 0;

    throttleServosActive = false;
    lastThrottle1Position = 0.0;
    lastThrottle2Position = 0.0;
    throttle1PositionReceived = false;
    throttle2PositionReceived = false;
    throttle2TargetSteps = 0;

    lastSpeedBrakePosition = 0.0;
    speedBrakePositionReceived = false;
    speedBrakeDeployed = false;

    // --------------------------------------------------------
    // Needle 2 standalone initialiseren
    // --------------------------------------------------------

    needle2Standalone.setEnablePin(44);
    needle2Standalone.setPinsInverted(
        false,
        false,
        true);
    needle2Standalone.setMaxSpeed(
        stepperConfig[TRIM_INDICATOR_2].speed);
    needle2Standalone.setAcceleration(
        stepperConfig[TRIM_INDICATOR_2].acceleration);
    needle2Standalone.setCurrentPosition(0);
    needle2Standalone.disableOutputs();

    // --------------------------------------------------------
    // Start homing
    // --------------------------------------------------------

    startTrimIndicatorHoming();
    startTrimIndicator2Homing();

   
}

// ============================================================
// START DECELERATION TRIM WHEEL
// ============================================================

void startTrimDeceleration()
{
    if (!trimTimerRunning ||
        trimStopping)
    {
        return;
    }

    trimStopping = true;

    trimStopUpdateTime =
        millis();
}

// ============================================================
// UPDATE STEPPERS
// ============================================================

void updateSteppers()
{
    // ========================================================
    // TRIM INDICATOR 1 HOMING
    // ========================================================

    if (trimIndicatorHomeState != TRIM_HOME_DONE &&
        trimIndicatorHomeState != TRIM_HOME_IDLE)
    {
        updateTrimIndicatorHoming();
    }
// ========================================================
// THROTTLE 1 KALIBRATIE
// ========================================================

if (THROTTLE_1_CALIBRATION)
{
    updateThrottle1Calibration();

    // Tijdens deze test doet de rest van de
    // normale stepperregeling niets.
    return;
}
    // ========================================================
    // TRIM INDICATOR 2 HOMING
    // ========================================================

    if (trimIndicator2HomeState != TRIM_HOME_DONE &&
        trimIndicator2HomeState != TRIM_HOME_IDLE)
    {
        updateTrimIndicator2Homing();
    }

        // ========================================================
    // THROTTLE 1 KALIBRATIE STARTEN
    // ========================================================

    static bool throttle1CalibrationStarted =
        false;

    if (THROTTLE_1_CALIBRATION &&
        !throttle1CalibrationStarted &&
        trimIndicatorHomeState == TRIM_HOME_DONE &&
        trimIndicator2HomeState == TRIM_HOME_DONE)
    {
        startThrottle1Calibration();

        throttle1CalibrationStarted =
            true;
    }

    // ========================================================
    // THROTTLE 1 KALIBRATIE UITVOEREN
    // ========================================================

    if (THROTTLE_1_CALIBRATION)
    {
        updateThrottle1Calibration();

        return;
    }

    // ========================================================
    // TIJDENS HOMING GEEN ANDERE NORMALE STEPPER-AANSTURING
    // ========================================================

    if (trimIndicatorHomeState != TRIM_HOME_DONE ||
        trimIndicator2HomeState != TRIM_HOME_DONE)
    {
        return;
    }

    // ========================================================
    // ALLE STEPPERS
    // ========================================================

    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        // ====================================================
        // THROTTLE 1 TIJDELIJKE KALIBRATIE
        // ====================================================

        if (i == THROTTLE_1 &&
            THROTTLE_1_CALIBRATION)
        {
            // updateThrottle1Calibration() heeft de run() al
            // uitgevoerd. Deze tak voorkomt dat de normale
            // throttle/stepperlogica hem daarna opnieuw bedient.
            continue;
        }

        // ====================================================
        // THROTTLE 2 / A-T
        // ====================================================
        //
        // Throttle 2 heeft geen home-schakelaar.
        //
        // Bij A/T servo = false:
        //   motor wordt vrijgegeven voor handmatige bediening.
        //
        // Bij overgang false -> true:
        //   de huidige PMDG-positie wordt eerst als de actuele
        //   fysieke motorpositie aangenomen. Daardoor ontstaat
        //   geen sprong bij het overnemen.
        //
        // Daarna volgt de motor de PMDG throttlepositie.
        // ====================================================

        // ====================================================
        // SPEEDBRAKE
        // ====================================================
        //
        // 0.00          = DOWN
        // 0.60-0.65     = ARMED
        // <= ongeveer 0.76 = normale vlucht
        // >= 0.90       = DEPLOY -> 1600 stappen
        //
        // ====================================================

        if (i == SPEED_BRAKE)
        {
            if (!speedBrakePositionReceived)
            {
                steppers[SPEED_BRAKE]
                    .disableOutputs();

                continue;
            }

            long target =
                speedBrakePositionToSteps(
                    lastSpeedBrakePosition);

            bool deployed =
                (target == SPEED_BRAKE_UP_STEPS);

            if (deployed != speedBrakeDeployed)
            {
                speedBrakeDeployed =
                    deployed;

                steppers[SPEED_BRAKE]
                    .setMaxSpeed(
                        SPEED_BRAKE_SPEED);

                steppers[SPEED_BRAKE]
                    .setAcceleration(
                        SPEED_BRAKE_ACCEL);

                steppers[SPEED_BRAKE]
                    .enableOutputs();

                steppers[SPEED_BRAKE]
                    .moveTo(target);
            }

            if (steppers[SPEED_BRAKE]
                    .distanceToGo() != 0)
            {
                steppers[SPEED_BRAKE]
                    .enableOutputs();

                steppers[SPEED_BRAKE]
                    .run();
            }
            else
            {
                steppers[SPEED_BRAKE]
                    .disableOutputs();
            }

            continue;
        }

        // ====================================================
        // THROTTLE 1 + THROTTLE 2 / A-T
        // ====================================================
        //
        // Zolang de A/T-servos niet werkelijk actief zijn,
        // blijven beide throttle-motoren vrij.
        //
        // Zodra de A/T-servos actief worden, volgen beide
        // throttles hun eigen PMDG throttlepositie.
        // ====================================================

        if (i == THROTTLE_1 ||
            i == THROTTLE_2)
        {
            static bool throttle1WasActive = false;
            static bool throttle2WasActive = false;

            bool &wasActive =
                (i == THROTTLE_1)
                    ? throttle1WasActive
                    : throttle2WasActive;

            // ------------------------------------------------
            // A/T niet actief -> motor vrij
            // ------------------------------------------------

            if (!throttleServosActive)
            {
                wasActive = false;

                steppers[i]
                    .disableOutputs();

                continue;
            }

            // ------------------------------------------------
            // Wachten op PMDG-data
            // ------------------------------------------------

            if (i == THROTTLE_1 &&
                !throttle1PositionReceived)
            {
                steppers[i]
                    .disableOutputs();

                continue;
            }

            if (i == THROTTLE_2 &&
                !throttle2PositionReceived)
            {
                steppers[i]
                    .disableOutputs();

                continue;
            }

            // ------------------------------------------------
            // A/T neemt over
            // ------------------------------------------------
            //
            // De huidige softwarepositie wordt gelijkgezet aan
            // de actuele PMDG-positie. De motor maakt op het
            // exacte overnamemoment dus geen sprong.
            // ------------------------------------------------

            long currentTarget;

            if (i == THROTTLE_1)
            {
                currentTarget =
                    throttle1TargetSteps;

                if (THROTTLE_1_GESPIEGELD)
                {
                    currentTarget =
                        -currentTarget;
                }

                if (!wasActive)
                {
                    steppers[THROTTLE_1]
                        .setCurrentPosition(
                            currentTarget);

                    steppers[THROTTLE_1]
                        .setMaxSpeed(
                            THROTTLE_1_AT_SPEED);

                    steppers[THROTTLE_1]
                        .setAcceleration(
                            THROTTLE_1_AT_ACCEL);
                }
            }
            else
            {
                currentTarget =
                    throttle2TargetSteps;

                if (THROTTLE_2_GESPIEGELD)
                {
                    currentTarget =
                        -currentTarget;
                }

                if (!wasActive)
                {
                    steppers[THROTTLE_2]
                        .setCurrentPosition(
                            currentTarget);

                    steppers[THROTTLE_2]
                        .setMaxSpeed(
                            THROTTLE_2_AT_SPEED);

                    steppers[THROTTLE_2]
                        .setAcceleration(
                            THROTTLE_2_AT_ACCEL);
                }
            }

            wasActive = true;

            // ------------------------------------------------
            // Werkelijke doelpositie
            // ------------------------------------------------

            long target;

            if (i == THROTTLE_1)
            {
                target =
                    throttle1TargetSteps;

                if (THROTTLE_1_GESPIEGELD)
                {
                    target = -target;
                }
            }
            else
            {
                target =
                    throttle2TargetSteps;

                if (THROTTLE_2_GESPIEGELD)
                {
                    target = -target;
                }
            }

            steppers[i]
                .enableOutputs();

            steppers[i]
                .moveTo(target);

            steppers[i]
                .run();

            continue;
        }

        // ====================================================
        // TRIM INDICATOR / NEEDLE 2 - STANDALONE
        // ====================================================

        if (i == TRIM_INDICATOR_2)
        {
            if (!stepperEnabled[TRIM_INDICATOR_2] ||
                !trimIndicatorValueReceived)
            {
                needle2Standalone.disableOutputs();
                continue;
            }

            long target2 =
                -trimIndicatorTarget;

            target2 +=
                40;

            needle2Standalone.enableOutputs();
            needle2Standalone.moveTo(target2);
            needle2Standalone.run();

            continue;
        }

        // ====================================================
        // Alle andere steppers: disabled
        // ====================================================

        if (!stepperEnabled[i])
        {
            if (i == TRIM_WHEEL &&
                trimTimerRunning)
            {
                stopTrimTimer();
            }

            steppers[i].disableOutputs();
            continue;
        }

        steppers[i].enableOutputs();

        // ====================================================
        // TRIM WHEEL
        // ====================================================

        if (i == TRIM_WHEEL)
        {
            if (!trimTimerRunning)
            {
                continue;
            }

            unsigned long now =
                millis();

            unsigned long elapsed =
                now -
                lastTrimAccelerationUpdate;

            if (elapsed > 0 &&
                !trimStopping)
            {
                lastTrimAccelerationUpdate =
                    now;

                float increase =
                    (TRIM_WHEEL_ACCEL *
                     elapsed) /
                    1000.0;

                noInterrupts();

                trimCurrentSpeed +=
                    increase;

                if (trimCurrentSpeed >
                    TRIM_WHEEL_SPEED)
                {
                    trimCurrentSpeed =
                        TRIM_WHEEL_SPEED;
                }

                float speed =
                    trimCurrentSpeed;

                interrupts();

                setTrimTimerSpeed(speed);
            }

            if (now -
                    lastTrimUpdateTime >=
                TRIM_STOP_TIMEOUT_MS)
            {
                startTrimDeceleration();
            }

            if (trimStopping)
            {
                unsigned long stopNow =
                    millis();

                if (stopNow -
                        trimStopUpdateTime >=
                    5)
                {
                    unsigned long stopElapsed =
                        stopNow -
                        trimStopUpdateTime;

                    trimStopUpdateTime =
                        stopNow;

                    float decrease =
                        (TRIM_WHEEL_DECEL *
                         stopElapsed) /
                        1000.0;

                    noInterrupts();

                    trimCurrentSpeed -=
                        decrease;

                    if (trimCurrentSpeed <= 0.0)
                    {
                        trimCurrentSpeed =
                            0.0;
                    }

                    float speed =
                        trimCurrentSpeed;

                    interrupts();

                    if (speed > 0.0)
                    {
                        setTrimTimerSpeed(speed);
                    }
                    else
                    {
                        stopTrimTimer();
                        trimStopping = false;
                    }
                }
            }

            continue;
        }

        // ====================================================
        // TRIM INDICATOR / NEEDLE 1
        // ====================================================

        if (i == TRIM_INDICATOR)
        {
            if (!trimIndicatorValueReceived)
            {
                continue;
            }

            steppers[TRIM_INDICATOR]
                .moveTo(
                    trimIndicatorTarget);

            steppers[TRIM_INDICATOR]
                .run();

            continue;
        }

        // ====================================================
        // ANDERE STEPPERS
        // ====================================================

        long target =
            targetPositions[i];

        if (stepperConfig[i].gespiegeld)
        {
            target =
                stepperConfig[i].maxPosition -
                target;
        }

        steppers[i]
            .moveTo(target);

        steppers[i]
            .run();
    }
}

// ============================================================
// TARGET
// ============================================================

void setStepperTarget(
    int index,
    long position)
{
    if (index < 0 ||
        index >= NUM_STEPPERS)
    {
        return;
    }

    if (position < 0)
    {
        position = 0;
    }

    if (position >
        stepperConfig[index].maxPosition)
    {
        position =
            stepperConfig[index].maxPosition;
    }

    targetPositions[index] =
        position;
}

// ============================================================
// ENABLE
// ============================================================

void setStepperEnable(
    int index,
    bool enabled)
{
    if (index < 0 ||
        index >= NUM_STEPPERS)
    {
        return;
    }

    stepperEnabled[index] =
        enabled;

    if (index == TRIM_INDICATOR_2)
    {
        if (enabled)
        {
            needle2Standalone.enableOutputs();
        }
        else
        {
            needle2Standalone.disableOutputs();
        }

        return;
    }

    if (enabled)
    {
        steppers[index].enableOutputs();
    }
    else
    {
        if (index == TRIM_WHEEL)
        {
            stopTrimTimer();
        }

        steppers[index].disableOutputs();
    }
}
// ============================================================
// POSITIE
// ============================================================

long getStepperPosition(
    int index)
{
    if (index < 0 ||
        index >= NUM_STEPPERS)
    {
        return 0;
    }

    if (index == TRIM_WHEEL)
    {
        long position;

        noInterrupts();

        position =
            trimWheelPosition;

        interrupts();

        return position;
    }

    if (index == TRIM_INDICATOR_2)
    {
        return needle2Standalone.currentPosition();
    }

    return steppers[index]
        .currentPosition();
}

// ============================================================
// ENABLE STATUS
// ============================================================

bool isStepperEnabled(
    int index)
{
    if (index < 0 ||
        index >= NUM_STEPPERS)
    {
        return false;
    }

    return stepperEnabled[index];
}

// ============================================================
// START THROTTLE 1 KALIBRATIE
// ============================================================

void startThrottle1Calibration()
{
    if (!THROTTLE_1_CALIBRATION)
    {
        return;
    }

    // De huidige mechanische positie wordt softwarematig 0.
    steppers[THROTTLE_1]
        .setCurrentPosition(0);

    steppers[THROTTLE_1]
        .setMaxSpeed(THROTTLE_1_TEST_SPEED);

    steppers[THROTTLE_1]
        .setAcceleration(THROTTLE_1_TEST_ACCEL);

    throttle1CalibrationTarget =
        THROTTLE_1_CALIBRATION_DIRECTION
            ? THROTTLE_1_TEST_STEPS
            : -THROTTLE_1_TEST_STEPS;

    steppers[THROTTLE_1]
        .enableOutputs();

    steppers[THROTTLE_1]
        .moveTo(throttle1CalibrationTarget);

    throttle1CalibrationState =
        THROTTLE1_CAL_MOVING_OUT;
}

// ============================================================
// UPDATE THROTTLE 1 KALIBRATIE
// ============================================================

void updateThrottle1Calibration()
{
    if (!THROTTLE_1_CALIBRATION)
    {
        return;
    }

    // --------------------------------------------------------
    // NAAR TESTPUNT
    // --------------------------------------------------------

    if (throttle1CalibrationState ==
        THROTTLE1_CAL_MOVING_OUT)
    {
        steppers[THROTTLE_1]
            .run();

        if (steppers[THROTTLE_1]
                .distanceToGo() == 0)
        {
            throttle1CalibrationHoldStart =
                millis();

            throttle1CalibrationState =
                THROTTLE1_CAL_HOLDING;
        }

        return;
    }

    // --------------------------------------------------------
    // 3 SECONDEN VAS HOUDEN
    // --------------------------------------------------------

    if (throttle1CalibrationState ==
        THROTTLE1_CAL_HOLDING)
    {
        if (millis() -
                throttle1CalibrationHoldStart >=
            THROTTLE_1_HOLD_TIME_MS)
        {
            steppers[THROTTLE_1]
                .moveTo(0);

            throttle1CalibrationState =
                THROTTLE1_CAL_RETURNING;
        }

        return;
    }

    // --------------------------------------------------------
    // TERUG NAAR SOFTWARE 0
    // --------------------------------------------------------

    if (throttle1CalibrationState ==
        THROTTLE1_CAL_RETURNING)
    {
        steppers[THROTTLE_1]
            .run();

        if (steppers[THROTTLE_1]
                .distanceToGo() == 0)
        {
            steppers[THROTTLE_1]
                .setCurrentPosition(0);

            steppers[THROTTLE_1]
                .disableOutputs();

            throttle1CalibrationState =
                THROTTLE1_CAL_IDLE;
        }

        return;
    }
}

// ============================================================
// THROTTLE SERVO STATUS
// ============================================================
//
// true = A/T servo's sturen daadwerkelijk de thrust levers
//
// ============================================================

void updateThrottleServos(bool active)
{
    throttleServosActive =
        active;
}

// ============================================================
// THROTTLE 1 POSITIE
// ============================================================

long throttlePercentToSteps(
    float value,
    float idlePercent,
    float fullPercent,
    long fullSteps)
{
    if (value <= idlePercent)
    {
        return 0;
    }

    if (value >= fullPercent)
    {
        return fullSteps;
    }

    float normalized =
        (value - idlePercent) /
        (fullPercent - idlePercent);

    long target =
        (long)(normalized *
                   (float)fullSteps +
               0.5);

    if (target < 0)
    {
        target = 0;
    }

    if (target > fullSteps)
    {
        target = fullSteps;
    }

    return target;
}

// ============================================================
// SPEEDBRAKE POSITION -> STEPS
// ============================================================
//
// We sturen voorlopig alleen de veilige automatische
// DOWN <-> UP toestand. ARM en normale vluchtwaarden worden
// niet als volledige deploy geïnterpreteerd.
//
// ============================================================

long speedBrakePositionToSteps(float value)
{
    if (value < 0.0)
    {
        value = 0.0;
    }

    if (value > 1.0)
    {
        value = 1.0;
    }

    if (speedBrakeDeployed)
    {
        if (value < SPEED_BRAKE_RETRACT_THRESHOLD)
        {
            return SPEED_BRAKE_DOWN_STEPS;
        }

        return SPEED_BRAKE_UP_STEPS;
    }

    if (value >= SPEED_BRAKE_DEPLOY_THRESHOLD)
    {
        return SPEED_BRAKE_UP_STEPS;
    }

    return SPEED_BRAKE_DOWN_STEPS;
}

// ============================================================
// UPDATE SPEEDBRAKE
// ============================================================

void updateSpeedBrake(float value)
{
    if (value < 0.0)
    {
        value = 0.0;
    }

    if (value > 1.0)
    {
        value = 1.0;
    }

    lastSpeedBrakePosition =
        value;

    speedBrakePositionReceived =
        true;

    // Het doel wordt in updateSteppers() uitgevoerd.
}

// ============================================================
// THROTTLE 1 POSITIE
// ============================================================

void updateThrottle1(float value)
{
    lastThrottle1Position =
        value;

    throttle1PositionReceived =
        true;

    throttle1TargetSteps =
        throttlePercentToSteps(
            value,
            THROTTLE_1_PMDG_IDLE,
            THROTTLE_1_PMDG_FULL,
            THROTTLE_1_FULL_STEPS);
}

// ============================================================
// THROTTLE 2 POSITIE
// ============================================================

void updateThrottle2(float value)
{
    lastThrottle2Position =
        value;

    throttle2PositionReceived =
        true;

    throttle2TargetSteps =
        throttlePercentToSteps(
            value,
            THROTTLE_2_PMDG_IDLE,
            THROTTLE_2_PMDG_FULL,
            THROTTLE_2_FULL_STEPS);
}

// ============================================================
// TRIM WHEEL
// ============================================================

void updateTrimWheel(
    float trimValue)
{
    unsigned long now =
        millis();

    // ========================================================
    // EERSTE WAARDE
    // ========================================================

    if (!trimInitialized)
    {
        oldTrim =
            trimValue;

        trimInitialized =
            true;

        lastTrimUpdateTime =
            now;

        return;
    }

    // ========================================================
    // WATCHDOG RESET
    // ========================================================

    lastTrimUpdateTime =
        now;

    // ========================================================
    // VERSCHIL
    // ========================================================

    float delta =
        trimValue - oldTrim;

    oldTrim =
        trimValue;

    trimAccumulatedDelta += delta;

    if (trimAccumulatedDelta > -TRIM_DEADBAND &&
        trimAccumulatedDelta < TRIM_DEADBAND)
    {
        return;
    }

    bool positiveDirection =
        (trimAccumulatedDelta > 0.0);

    trimAccumulatedDelta = 0.0;
    // ========================================================
    // ENABLE
    // ========================================================

    steppers[TRIM_WHEEL]
        .enableOutputs();

    // ========================================================
    // DECELERATIE ANNULEREN
    // ========================================================

    trimStopping =
        false;

    // ========================================================
    // TIMER STARTEN
    // ========================================================

    if (!trimTimerRunning)
    {
        startTrimTimer(
            positiveDirection);

        return;
    }

    // ========================================================
    // RICHTING WIJZIGEN
    // ========================================================

    if (trimTimerDirectionPositive !=
        positiveDirection)
    {
        noInterrupts();

        trimTimerDirectionPositive =
            positiveDirection;

        interrupts();

        digitalWrite(
            TRIM_DIR_PIN,
            positiveDirection
                ? HIGH
                : LOW);
    }
}

// ============================================================
// START TRIM INDICATOR HOMING
// ============================================================

void startTrimIndicatorHoming()
{
    trimIndicatorHomeState =
        TRIM_HOME_SEARCHING;

    steppers[TRIM_INDICATOR]
        .enableOutputs();

    steppers[TRIM_INDICATOR]
        .setMaxSpeed(
            TRIM_INDICATOR_HOME_SPEED);

    // --------------------------------------------------------
    // TRUE = juiste richting naar HOME
    // --------------------------------------------------------

    if (TRIM_INDICATOR_GESPIEGELD)
    {
        steppers[TRIM_INDICATOR]
            .setSpeed(
                TRIM_INDICATOR_HOME_SPEED);
    }
    else
    {
        steppers[TRIM_INDICATOR]
            .setSpeed(
                -TRIM_INDICATOR_HOME_SPEED);
    }
}

// ============================================================
// UPDATE TRIM INDICATOR HOMING
// ============================================================

void updateTrimIndicatorHoming()
{
    // ========================================================
    // IDLE
    // ========================================================

    if (trimIndicatorHomeState ==
        TRIM_HOME_IDLE)
    {
        return;
    }

    // ========================================================
    // HOME ZOEKEN
    // ========================================================

    if (trimIndicatorHomeState ==
        TRIM_HOME_SEARCHING)
    {
        // INPUT_PULLUP:
        //
        // HIGH = niet actief
        // LOW  = actief
        //

        if (digitalRead(
                TRIM_INDICATOR_HOME_PIN) == LOW)
        {
            // ------------------------------------------------
            // HOME GEVONDEN
            // ------------------------------------------------

            steppers[TRIM_INDICATOR]
                .setSpeed(0);

            // Schakelaarpositie tijdelijk als 0
            steppers[TRIM_INDICATOR]
                .setCurrentPosition(0);

            // ------------------------------------------------
            // HOME OFFSET
            // ------------------------------------------------

            long offset =
                TRIM_INDICATOR_GESPIEGELD
                    ? TRIM_INDICATOR_HOME_OFFSET
                    : -TRIM_INDICATOR_HOME_OFFSET;

            steppers[TRIM_INDICATOR]
                .moveTo(offset);

            trimIndicatorHomeState =
                TRIM_HOME_OFFSET;

            return;
        }

        // ----------------------------------------------------
        // Nog niet bij HOME
        // ----------------------------------------------------

        steppers[TRIM_INDICATOR]
            .runSpeed();

        return;
    }

    // ========================================================
    // HOME OFFSET UITVOEREN
    // ========================================================

    if (trimIndicatorHomeState ==
        TRIM_HOME_OFFSET)
    {
        steppers[TRIM_INDICATOR]
            .run();

        if (steppers[TRIM_INDICATOR]
                .distanceToGo() == 0)
        {
            // ------------------------------------------------
            // DIT IS HET ECHTE SOFTWARE NULPUNT
            // ------------------------------------------------

            steppers[TRIM_INDICATOR]
                .setCurrentPosition(0);

            // ------------------------------------------------
            // NORMALE INSTELLINGEN HERSTELLEN
            // ------------------------------------------------

            steppers[TRIM_INDICATOR]
                .setMaxSpeed(
                    stepperConfig[TRIM_INDICATOR].speed);

            steppers[TRIM_INDICATOR]
                .setAcceleration(
                    stepperConfig[TRIM_INDICATOR].acceleration);

            steppers[TRIM_INDICATOR]
                .setSpeed(0);

            // ------------------------------------------------
            // HOMING KLAAR
            // ------------------------------------------------

            trimIndicatorHomeState =
                TRIM_HOME_DONE;

            // ------------------------------------------------
            // TIJDELIJKE KALIBRATIE
            // ------------------------------------------------

            if (TRIM_INDICATOR_CALIBRATION)
            {
                startTrimIndicatorCalibration();
            }
            else
            {
                // De outputs mogen uit.
                // updateSteppers() zet ze weer aan
                // zodra er een TI-waarde beschikbaar is.
                steppers[TRIM_INDICATOR]
                    .disableOutputs();
            }

            return;
        }

        return;
    }
}

// ============================================================
// START TRIM INDICATOR 2 HOMING
// ============================================================

void startTrimIndicator2Homing()
{
    trimIndicator2HomeState =
        TRIM_HOME_SEARCHING;

    needle2Standalone
        .enableOutputs();

    needle2Standalone
        .setMaxSpeed(
            TRIM_INDICATOR_2_HOME_SPEED);

    // FALSE = richting naar HOME voor Needle 2
    if (TRIM_INDICATOR_2_GESPIEGELD)
    {
        needle2Standalone
            .setSpeed(
                TRIM_INDICATOR_2_HOME_SPEED);
    }
    else
    {
        needle2Standalone
            .setSpeed(
                -TRIM_INDICATOR_2_HOME_SPEED);
    }
}

// ============================================================
// UPDATE TRIM INDICATOR 2 HOMING
// ============================================================

void updateTrimIndicator2Homing()
{
    // ========================================================
    // HOME ZOEKEN
    // ========================================================

    if (trimIndicator2HomeState ==
        TRIM_HOME_SEARCHING)
    {
        if (digitalRead(
                TRIM_INDICATOR_2_HOME_PIN) == LOW)
        {
            // ------------------------------------------------
            // HOME GEVONDEN
            // ------------------------------------------------

            needle2Standalone
                .setSpeed(0);

            needle2Standalone
                .setCurrentPosition(0);

            // ------------------------------------------------
            // OFFSET
            // ------------------------------------------------

            long offset =
                TRIM_INDICATOR_2_GESPIEGELD
                    ? TRIM_INDICATOR_2_HOME_OFFSET
                    : -TRIM_INDICATOR_2_HOME_OFFSET;

            needle2Standalone
                .moveTo(offset);

            trimIndicator2HomeState =
                TRIM_HOME_OFFSET;

            return;
        }

        // ----------------------------------------------------
        // Nog niet bij HOME
        // ----------------------------------------------------

        needle2Standalone
            .runSpeed();

        return;
    }

    // ========================================================
    // OFFSET UITVOEREN
    // ========================================================

    if (trimIndicator2HomeState ==
        TRIM_HOME_OFFSET)
    {
        needle2Standalone
            .run();

        if (needle2Standalone
                .distanceToGo() == 0)
        {
            // ------------------------------------------------
            // ECHTE SOFTWARE 0
            // ------------------------------------------------

            needle2Standalone
                .setCurrentPosition(0);

            // ------------------------------------------------
            // NORMALE INSTELLINGEN
            // ------------------------------------------------

            needle2Standalone
                .setMaxSpeed(
                    stepperConfig[TRIM_INDICATOR_2].speed);

            needle2Standalone
                .setAcceleration(
                    stepperConfig[TRIM_INDICATOR_2].acceleration);

            needle2Standalone
                .setSpeed(0);

            // ------------------------------------------------
            // HOMING KLAAR
            // ------------------------------------------------

            trimIndicator2HomeState =
                TRIM_HOME_DONE;

            // Voorlopig nog uitschakelen.
            // Needle 2 krijgt nog geen kTrimIndicator.
            needle2Standalone
                .disableOutputs();

            return;
        }

        return;
    }
}
// ============================================================
// TIJDELIJKE KALIBRATIE
// ============================================================

void startTrimIndicatorCalibration()
{
    if (!TRIM_INDICATOR_CALIBRATION)
    {
        return;
    }

    steppers[TRIM_INDICATOR]
        .enableOutputs();

    steppers[TRIM_INDICATOR]
        .setMaxSpeed(
            TRIM_INDICATOR_TEST_SPEED);

    steppers[TRIM_INDICATOR]
        .setAcceleration(
            TRIM_INDICATOR_TEST_ACCEL);

    // --------------------------------------------------------
    // Vanaf fysiek 0 naar de andere kant dan HOME
    // --------------------------------------------------------

    long calibrationTarget =
        TRIM_INDICATOR_GESPIEGELD
            ? -TRIM_INDICATOR_TEST_STEPS
            : TRIM_INDICATOR_TEST_STEPS;

    steppers[TRIM_INDICATOR]
        .moveTo(
            calibrationTarget);
}
// ============================================================
// FYSIEKE INDICATORSTAND -> STEPPERSTAPPEN
// ============================================================
//
// Mechanische kalibratie:
//
// 100 stappen -> 2.5
// 200 stappen -> 3.0
// 300 stappen -> 8.5
// 400 stappen -> 9.8
// 500 stappen -> 12.5
// 600 stappen -> 15.2
// 650 stappen -> 16.5
// 700 stappen -> 18.0
//
// Deze functie interpoleert tussen de gemeten punten.
// ============================================================

long physicalIndicatorToSteps(float physicalPosition)
{
    if (physicalPosition <= 0.0)
    {
        return 0;
    }

    if (physicalPosition <= 2.5)
    {
        float fraction =
            physicalPosition / 2.5;

        return (long)(fraction * 100.0 + 0.5);
    }

    if (physicalPosition <= 3.0)
    {
        float fraction =
            (physicalPosition - 2.5) /
            (3.0 - 2.5);

        return (long)(100.0 +
                      fraction * (200.0 - 100.0) +
                      0.5);
    }

    if (physicalPosition <= 8.5)
    {
        float fraction =
            (physicalPosition - 3.0) /
            (8.5 - 3.0);

        return (long)(200.0 +
                      fraction * (300.0 - 200.0) +
                      0.5);
    }

    if (physicalPosition <= 9.8)
    {
        float fraction =
            (physicalPosition - 8.5) /
            (9.8 - 8.5);

        return (long)(300.0 +
                      fraction * (400.0 - 300.0) +
                      0.5);
    }

    if (physicalPosition <= 12.5)
    {
        float fraction =
            (physicalPosition - 9.8) /
            (12.5 - 9.8);

        return (long)(400.0 +
                      fraction * (500.0 - 400.0) +
                      0.5);
    }

    if (physicalPosition <= 15.2)
    {
        float fraction =
            (physicalPosition - 12.5) /
            (15.2 - 12.5);

        return (long)(500.0 +
                      fraction * (600.0 - 500.0) +
                      0.5);
    }

    if (physicalPosition <= 16.5)
    {
        float fraction =
            (physicalPosition - 15.2) /
            (16.5 - 15.2);

        return (long)(600.0 +
                      fraction * (650.0 - 600.0) +
                      0.5);
    }

    if (physicalPosition <= 18.0)
    {
        float fraction =
            (physicalPosition - 16.5) /
            (18.0 - 16.5);

        return (long)(650.0 +
                      fraction * (700.0 - 650.0) +
                      0.5);
    }

    return 700;
}

// ============================================================
// TRIM INDICATOR
// ============================================================
//
// kTrimIndicator = SIMCONNECT:ELEVATOR TRIM INDICATOR
// Eenheid = radialen
//
// Gekoppelde meetpunten:
//
// TI 0.3421610 -> fysieke 4.5
// TI 0.4149358 -> fysieke 6.0
// TI 0.4940770 -> fysieke 8.0
// TI 0.8643216 -> fysieke 15.0
//
// ============================================================

void updateTrimIndicator(float indicatorValue)
{
    // ========================================================
    // LAATSTE TI-WAARDE OPSLAAN
    // ========================================================

    lastTrimIndicatorValue =
        indicatorValue;

    trimIndicatorValueReceived =
        true;

    // ========================================================
    // TIJDENS HOMING GEEN BEWEGING
    // ========================================================

    if (trimIndicatorHomeState !=
        TRIM_HOME_DONE)
    {
        return;
    }

    // ========================================================
    // TI-MEETPUNTEN
    //
    // TI waarde -> cockpit trimstand
    // ========================================================

    const float tiValues[15] =
        {
            -0.2269650, // 1
            -0.1369530, // 2
            -0.0538750, // 3
            0.0186820,  // 4
            0.0987340,  // 5
            0.1942440,  // 6
            0.2679350,  // 7
            0.3421610,  // 8
            0.4149350,  // 9
            0.4940770,  // 10
            0.5665200,  // 11
            0.6514510,  // 12
            0.7172933,  // 13
            0.7884810,  // 14
            0.8643210   // 15
        };

    // ========================================================
    // STEPPER-MEETPUNTEN
    //
    // Fysieke cockpitstand -> echte stappen
    // ========================================================

    const long stepValues[18] =
        {
            0,   // 0
            164, // 1
            195, // 2
            229, // 3
            267, // 4
            305, // 5
            345, // 6
            379, // 7
            417, // 8
            460, // 9
            505, // 10
            540, // 11
            575, // 12
            610, // 13
            645, // 14
            680, // 15
            725, // 16
            770  // 17
        };

    // ========================================================
    // TRIMSTAND BEPALEN
    // ========================================================

    float trimPosition;

    // Onderste meetpunt
    if (indicatorValue <= tiValues[0])
    {
        // Onder trim 1 trekken we lineair terug
        // naar trim 0.

        float fraction =
            (indicatorValue + 0.3000000) /
            (tiValues[0] + 0.3000000);

        if (fraction < 0.0)
            fraction = 0.0;

        trimPosition =
            fraction * 1.0;
    }

    // Bovenste meetpunt
    else if (indicatorValue >= tiValues[14])
    {
        // Boven trim 15 trekken we door tot trim 17.
        //
        // We hebben geen afzonderlijke TI-metingen voor
        // 16 en 17, dus voorlopig lineair doortrekken.

        float fraction =
            (indicatorValue - tiValues[14]) /
            (1.0 - tiValues[14]);

        if (fraction > 1.0)
            fraction = 1.0;

        trimPosition =
            15.0 +
            fraction * 2.0;
    }

    // Tussen twee bekende TI-meetpunten
    else
    {
        int segment = 0;

        while (segment < 14 &&
               indicatorValue > tiValues[segment + 1])
        {
            segment++;
        }

        float fraction =
            (indicatorValue -
             tiValues[segment]) /
            (tiValues[segment + 1] -
             tiValues[segment]);

        trimPosition =
            (float)(segment + 1) +
            fraction;
    }

    // ========================================================
    // TRIMPOSITIE BEGRENZEN
    // ========================================================

    if (trimPosition < 0.0)
    {
        trimPosition = 0.0;
    }

    if (trimPosition > 17.0)
    {
        trimPosition = 17.0;
    }

    // ========================================================
    // TRIMSTAND -> STEPPERSTAPPEN
    // ========================================================

    if (trimPosition <= 0.0)
    {
        trimIndicatorTarget = 0;
    }
    else if (trimPosition >= 17.0)
    {
        trimIndicatorTarget = 770;
    }
    else
    {
        int lower =
            (int)trimPosition;

        float fraction =
            trimPosition -
            (float)lower;

        trimIndicatorTarget =
            (long)(stepValues[lower] +
                   fraction *
                       (stepValues[lower + 1] -
                        stepValues[lower]) +
                   0.5);
    }

    // ========================================================
    // FYSIEKE RICHTING
    //
    // TRUE = juiste richting voor Needle 1
    // ========================================================

    if (TRIM_INDICATOR_GESPIEGELD)
    {
        trimIndicatorTarget =
            -trimIndicatorTarget;
    }

    // ========================================================
    // STEPPER ACTIVEREN
    // ========================================================

    stepperEnabled[TRIM_INDICATOR] =
        true;

    steppers[TRIM_INDICATOR]
        .enableOutputs();

    // ========================================================
    // DOEL INSTELLEN
    // ========================================================

    steppers[TRIM_INDICATOR]
        .moveTo(
            trimIndicatorTarget);
}
