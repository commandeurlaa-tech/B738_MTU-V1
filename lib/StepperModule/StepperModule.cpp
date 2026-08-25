#include "StepperModule.h"
#include <AccelStepper.h>
#include <avr/interrupt.h>

// ============================================================
// TRIM WHEEL
// ============================================================

const float TRIM_WHEEL_SPEED = 10500.0;

// Acceleratie in steps/sec per seconde
const float TRIM_WHEEL_ACCEL = 80000.0;
const float TRIM_WHEEL_DECEL = 50000.0;

bool trimStopping = false;
unsigned long trimStopUpdateTime = 0;
// Direct stoppen bij loslaten
const unsigned long TRIM_STOP_TIMEOUT_MS = 80;

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

const float TRIM_DEADBAND = 0.000002;

// ============================================================
// PINNEN
// ============================================================

const int TRIM_STEP_PIN = 39;
const int TRIM_DIR_PIN = 40;
const int TRIM_ENA_PIN = 46;

// ============================================================
// STEPPERS
// ============================================================

AccelStepper steppers[NUM_STEPPERS] =
    {
        AccelStepper(AccelStepper::DRIVER, 29, 30),
        AccelStepper(AccelStepper::DRIVER, 31, 32),
        AccelStepper(AccelStepper::DRIVER, 33, 34),
        AccelStepper(AccelStepper::DRIVER, 35, 36),
        AccelStepper(AccelStepper::DRIVER, 37, 38),
        AccelStepper(AccelStepper::DRIVER, 39, 40)};

const int enaPins[NUM_STEPPERS] =
    {
        41,
        42,
        43,
        44,
        45,
        46};

// ============================================================
// CONFIGURATIE
// ============================================================

const StepperConfig stepperConfig[NUM_STEPPERS] =
    {
        {10000, 1000.0, 500.0, false},
        {10000, 1000.0, 500.0, false},
        {10000, 1000.0, 500.0, false},
        {10000, 1000.0, 500.0, false},
        {10000, 1000.0, 500.0, false},
        {10000, 1000.0, 500.0, false}};

// ============================================================
// STATUS
// ============================================================

long targetPositions[NUM_STEPPERS] = {0};

bool stepperEnabled[NUM_STEPPERS] =
    {
        false,
        false,
        false,
        false,
        false,
        false};

// ============================================================
// TRIM STATUS
// ============================================================

float oldTrim = 0.0;

bool trimInitialized = false;

volatile bool trimTimerRunning = false;
volatile bool trimStepHigh = false;
volatile bool trimTimerDirectionPositive = false;

volatile long trimWheelPosition = 0;

// Huidige snelheid van het trimwiel
volatile float trimCurrentSpeed = 0.0;

// Tijdstip laatste trimdata
unsigned long lastTrimUpdateTime = 0;

// Tijdstip laatste acceleratie-update
unsigned long lastTrimAccelerationUpdate = 0;

// ============================================================
// TIMER2 FREQUENTIE INSTELLEN
// ============================================================
//
// Timer2:
// CPU = 16 MHz
// prescaler = 8
// timer clock = 2 MHz
//
// We gebruiken twee interrupts per STEP.
//
// gewenste STEP-frequentie:
// bijvoorbeeld 10.000 steps/sec
//
// interrupt frequentie:
// 20.000 Hz
//
// OCR2A:
// 2.000.000 / 20.000 - 1 = 99
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

    // Richting
    digitalWrite(
        TRIM_DIR_PIN,
        positiveDirection ? HIGH : LOW);

    trimTimerDirectionPositive =
        positiveDirection;

    // STEP laag
    digitalWrite(TRIM_STEP_PIN, LOW);

    trimStepHigh = false;

    // Begin op lage snelheid
    trimCurrentSpeed = 1000.0;

    // Timer instellen op startsnelheid
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

    // Compare interrupt
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

    // STEP gegarandeerd LOW
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
    pinMode(TRIM_STEP_PIN, OUTPUT);
    pinMode(TRIM_DIR_PIN, OUTPUT);
    pinMode(TRIM_ENA_PIN, OUTPUT);

    digitalWrite(TRIM_STEP_PIN, LOW);
    digitalWrite(TRIM_DIR_PIN, LOW);
    digitalWrite(TRIM_ENA_PIN, HIGH);

    // Timer2 uit
    TCCR2A = 0;
    TCCR2B = 0;

    TCNT2 = 0;

    TIMSK2 &= ~(1 << OCIE2A);

    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i].setEnablePin(enaPins[i]);

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

    oldTrim = 0.0;

    trimInitialized = false;

    trimTimerRunning = false;

    trimStepHigh = false;

    trimTimerDirectionPositive = false;

    trimWheelPosition = 0;

    trimCurrentSpeed = 0.0;

    lastTrimUpdateTime = millis();

    lastTrimAccelerationUpdate = millis();
}
// ============================================================
// START DECELERATION
// ============================================================
void startTrimDeceleration()
{
    if (!trimTimerRunning || trimStopping)
        return;

    trimStopping = true;
    trimStopUpdateTime = millis();
}
// ============================================================
// UPDATE
// ============================================================

void updateSteppers()
{
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        if (stepperEnabled[i])
        {
            steppers[i].enableOutputs();

            // =================================================
            // TRIM WHEEL
            // =================================================

            if (i == TRIM_WHEEL)
            {
                // ------------------------------------------------
                // Acceleratie
                // ------------------------------------------------

                if (trimTimerRunning)
                {
                    unsigned long now = millis();

                    unsigned long elapsed =
                        now -
                        lastTrimAccelerationUpdate;

                    if (elapsed > 0 && !trimStopping)
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

                    // ------------------------------------------------
                    // Geen nieuwe data?
                    // ------------------------------------------------

                    if (now -
                            lastTrimUpdateTime >=
                        TRIM_STOP_TIMEOUT_MS)
                    {
                        startTrimDeceleration();
                    }
                    // ------------------------------------------------
                    // ZACHT AFREMMEN NA LOSLATEN
                    // ------------------------------------------------

                    if (trimStopping)
                    {
                        unsigned long stopNow = millis();

                        // Iedere 5 ms snelheid aanpassen
                        if (stopNow - trimStopUpdateTime >= 5)
                        {
                            unsigned long elapsed =
                                stopNow - trimStopUpdateTime;

                            trimStopUpdateTime = stopNow;

                            float decrease =
                                (TRIM_WHEEL_DECEL * elapsed) /
                                1000.0;

                            noInterrupts();

                            trimCurrentSpeed -= decrease;

                            if (trimCurrentSpeed <= 0.0)
                            {
                                trimCurrentSpeed = 0.0;
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
                }
            }

            // =================================================
            // ANDERE STEPPERS
            // =================================================

            else
            {
                long target =
                    targetPositions[i];

                if (stepperConfig[i].gespiegeld)
                {
                    target =
                        stepperConfig[i].maxPosition -
                        target;
                }

                steppers[i].moveTo(target);

                steppers[i].run();
            }
        }

        else
        {
            if (i == TRIM_WHEEL)
            {
                if (trimTimerRunning)
                {
                    stopTrimTimer();
                }
            }

            steppers[i].disableOutputs();
        }
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
        position = 0;

    if (position >
        stepperConfig[index].maxPosition)
    {
        position =
            stepperConfig[index].maxPosition;
    }

    targetPositions[index] = position;
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

    stepperEnabled[index] = enabled;

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

long getStepperPosition(int index)
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

    return steppers[index].currentPosition();
}

// ============================================================
// ENABLE STATUS
// ============================================================

bool isStepperEnabled(int index)
{
    if (index < 0 ||
        index >= NUM_STEPPERS)
    {
        return false;
    }

    return stepperEnabled[index];
}

// ============================================================
// TRIM WHEEL
// ============================================================

void updateTrimWheel(float trimValue)
{
    unsigned long now = millis();

    // ========================================================
    // EERSTE WAARDE
    // ========================================================

    if (!trimInitialized)
    {
        oldTrim = trimValue;

        trimInitialized = true;

        lastTrimUpdateTime = now;

        return;
    }

    // ========================================================
    // WATCHDOG RESET
    // ========================================================
    //
    // Er komt nog steeds trimdata binnen.
    // Dus het wiel mag blijven draaien.
    //

    lastTrimUpdateTime = now;

    // ========================================================
    // VERSCHIL BEREKENEN
    // ========================================================

    float delta =
        trimValue - oldTrim;

    // Nieuwe waarde bewaren
    oldTrim = trimValue;

    // ========================================================
    // DEADBAND
    // ========================================================
    //
    // Heel kleine veranderingen negeren.
    // Dit voorkomt onnodig starten/stoppen.
    //

    if (delta > -TRIM_DEADBAND &&
        delta < TRIM_DEADBAND)
    {
        return;
    }

    // ========================================================
    // RICHTING BEPALEN
    // ========================================================

    bool positiveDirection =
        (delta > 0.0);

    // ========================================================
    // SPIEGELEN
    // ========================================================

    if (TRIM_WHEEL_GESPIEGELD)
    {
        positiveDirection =
            !positiveDirection;
    }

    // ========================================================
    // MOTOR ENABLE
    // ========================================================

    steppers[TRIM_WHEEL].enableOutputs();

    // ========================================================
    // BELANGRIJK:
    // ALS HET WIEL AAN HET AFREMMEN WAS,
    // DAN ANNULEREN WE DE STOP.
    // ========================================================

    trimStopping = false;

    // ========================================================
    // TIMER NOG NIET ACTIEF
    // ========================================================

    if (!trimTimerRunning)
    {
        startTrimTimer(
            positiveDirection);

        return;
    }

    // ========================================================
    // RICHTING VERANDERD
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