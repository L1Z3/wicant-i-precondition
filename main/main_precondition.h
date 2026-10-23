/** @file main_precondition.h
 *  @brief Manual battery preconditioning for E-GMP cars: the settings
 *         group, the tasks that feed precondition.c from can_manager, and
 *         its HTTP routes (main_precondition_http.c). */
#pragma once

#include "esp_err.h"

/** Registers the "precondition" settings group. Settings window only. */
esp_err_t main_precondition_register_settings(void);

/** Turns the CAN bus on when no CAN settings have been saved yet (first
 *  boot, factory reset), so preconditioning works out of the box. AFTER
 *  can_manager_init registers its group, BEFORE settings_manager_start. */
esp_err_t main_precondition_can_defaults(void);

/** GET /api/precondition, POST /api/precondition/toggle and /popup, and the
 *  /precondition.html page. In the *_register_http block, BEFORE
 *  web_ui_v2_register. */
esp_err_t main_precondition_register_http(void);

/** Starts the tasks. AFTER can_manager_start; the frame task subscribes to
 *  the bus once it is up, and again whenever the subscription is lost. */
esp_err_t main_precondition_start(void);
