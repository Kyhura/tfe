/*
 *  Montefiore Robocup
 *  ==================
 *
 *  pwm.c: PWM setup and operations. 
 */


 #include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <stdint.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "pwm.h"
#include "../utils/delay.h"
#include "../utils/parameters.h"
#include "../utils/board.h"
#include "../utils/diag.h"
#include "../utils/flags.h"
#include "../utils/angle_diff.h"


/*** Public functions  ***/

void pwm_setup(void) {

    /* Enable GPIO clocks. */

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_TIM1);

    /* Setup PWM. */

    gpio_mode_setup(DRIVER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SD);
    gpio_set(DRIVER_PORT, SD);
    gpio_mode_setup(DRIVER_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LEFT_IN | RIGHT_IN);
    gpio_set_af(DRIVER_PORT, GPIO_AF6, LEFT_IN);
    gpio_set_af(DRIVER_PORT, GPIO_AF11, RIGHT_IN);

    rcc_periph_reset_pulse(RST_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1,
                    TIM_CR1_DIR_UP);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1); 
    timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC4);
    timer_enable_break_main_output(TIM1);
    timer_set_period(TIM1, 1024);
    timer_set_oc_value(TIM1, TIM_OC1, 1024/2);
    timer_set_oc_value(TIM1, TIM_OC4, 1024/2);
    timer_enable_counter(TIM1);

}

void pwm_update(int16_t new_pwm) {

    bool CCW = false;
    double limit_tol = 5;

    double converted_servo_angle, converted_CCW, converted_CW;

    // By definition of the referential system
    converted_CCW = 0;

    // Convert angle;
    converted_servo_angle = angle_referential_conversion(servo_angle);
    converted_CW = angle_referential_conversion(CW_angle_limit);

    if(fabs(new_pwm) > max_torque) {
        if(new_pwm < 0) {
            board_set_leds(BOARD_LED_CYAN);
            set_flag(MAX_TOR_REACHED_FLAG);
            new_pwm = -max_torque;
        }
        else {
            board_set_leds(BOARD_LED_MAGENTA);
            set_flag(MAX_TOR_REACHED_FLAG);
            new_pwm = max_torque;
        }
    }

    if(new_pwm < 0) CCW = true;

    /** If servo tries actively to go outside the limit **/

    if((CCW && converted_servo_angle <= converted_CCW + limit_tol) ||
        (!CCW && converted_servo_angle >= converted_CW - limit_tol)) {

        board_set_leds(BOARD_LED_RED);
        set_flag(OOR_PID_FLAG);

        // O indicates to the motor to stop
        new_pwm = 0;
    
    }

    // Put it back to the [0;1024] range
    new_pwm = new_pwm + 512;

    timer_set_oc_value(TIM1, TIM_OC1, 1024 - new_pwm);
    timer_set_oc_value(TIM1, TIM_OC4, new_pwm);
}


void pwm_motor_stop() {

    timer_set_oc_value(TIM1, TIM_OC1, 512);
    timer_set_oc_value(TIM1, TIM_OC4, 512);
}

void disable_motor() {

    board_set_leds(BOARD_LED_RED);

    timer_disable_oc_output(TIM1, TIM_OC1);
    timer_disable_oc_output(TIM1, TIM_OC4);
    timer_disable_counter(TIM1);
    
}
