#ifndef INPUTMODULE_H
#define INPUTMODULE_H

/**
 * @brief Module for handling hardware inputs (encoders and buttons).
 * This module manages rotary encoders and push buttons, sending state changes to SPAD.neXt.
 */

// Function declarations

/**
 * @brief Checks all rotary encoders for position changes and sends updates.
 * Monitors Enc_1 and Enc_2 for rotation and reports direction to SPAD.neXt.
 */
void CheckAllEncoders();

/**
 * @brief Checks all buttons for state changes and sends updates.
 * Monitors digital pins 9-45 for button presses/releases and reports to SPAD.neXt.
 */
void CheckAllButtons();

#endif // INPUTS_H