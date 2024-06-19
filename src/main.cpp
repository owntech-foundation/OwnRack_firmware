/*
 * Copyright (c) 2021-2024 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  This file it the main entry point of the
 *         OwnTech Power API. Please check the OwnTech
 *         documentation for detailed information on
 *         how to use Power API: https://docs.owntech.org/
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 */

//--------------OWNTECH APIs----------------------------------
#include "DataAPI.h"
#include "TaskAPI.h"
#include "TwistAPI.h"
#include "SpinAPI.h"
#include "CommunicationAPI.h"
#include "pr.h"
#include "filters.h"
#include "transform.h"
//----------- USER INCLUDE ----------------------

//------------ZEPHYR DRIVERS----------------------
#include "zephyr/kernel.h"
#include "zephyr/console/console.h"
#include <cstdint>
#include <zephyr/timing/timing.h>

//---------- LL drivers --------------------------
#include "stm32_ll_gpio.h"
#include "stm32_ll_lpuart.h"

#define DMA_BUFFER_SIZE 10 // 3 byte for Va/Vb, 3 byte for Vc/(Ia/Ic), 3 byte for Ib and w, and 1 byte for identifiant

uint8_t SPIN_MODE = 0;
                    
#define UID_SLAVE_A 0x3A004D
#define UID_SLAVE_B 0x58002F
#define UID_SLAVE_C 0x400040

#define SLAVE_A 1
#define SLAVE_B 2
#define SLAVE_C 3

#define RS_ID_MASTER 0 
#define RS_ID_SLAVE_A 1 
#define RS_ID_SLAVE_B 2 
#define RS_ID_SLAVE_C 3 
char TXT_ID[] = {'M', 'A', 'B', 'C'};
uint32_t n_receive_calls = 0;
uint8_t stop_recording = 0;
#define CURRENT_LIMIT 9.0
#define VOLTAGE_SCALE 50.0
#define RECEIVE_MODULO 30000
// retrieve identifiant
#define GET_ID(id_and_status) ((id_and_status >> 6) & 0x3)  
#define SET_ID(id_and_status, value) { \
    id_and_status &= 0x3F;\
    id_and_status |= ((value & 0x3) << 6);\
    }

// set bit 5
#define SET_OVERCURRENT(id_and_status) (id_and_status |= 0x20)
// get bit 5
#define IS_OVERCCURRENT(id_and_status) ((id_and_status >> 5) & 1) // check current for safety
#define GET_STATUS(id_and_status) (id_and_status & 1)             // check the master_status (IDLE or POWER )
#define REROLL_COUNTER(id_and_status) ((id_and_status >> 1) & 1)  // reroll counter to monitor the intern variables

#define GET_UPPER_12BITS(buf_3bytes) ((int16_t) ((*(buf_3bytes + 2) << 4) | (*(buf_3bytes + 1) >> 4)))

#define GET_LOWER_12BITS(buf_3bytes) ((int16_t) (((*(buf_3bytes + 1) & 0xf)<<8) | (*(buf_3bytes ) & 0xff)))

#define PUT_UPPER_12BITS(buf_3bytes, data) { \
    *(buf_3bytes+1) = (0xf0 & (data << 4)) | (0xf & *(buf_3bytes+1));\
    *(buf_3bytes+2)  = (data & 0xff0) >> 4; \
    }

#define PUT_LOWER_12BITS(buf_3bytes, data) {\
    *buf_3bytes = data & 0xff; \
    *(buf_3bytes+1) = (0xf0 & *(buf_3bytes+1)) | ((data & 0xf00) >> 8); \
    }

__inline__ int16_t to_12bits(float32_t data, float scale, float32_t offset = 0.0) 
{
    return (int16_t) ((data + offset) * 4095.0 / (scale));
}

float32_t from_12bits(int16_t data, float32_t scale, float offset = 0.0)
{
    return ((scale * (float32_t) data) / 4095.0)  - offset;
}

// to get the processor unique id 96 bits data 
unsigned int *uid_0 = (unsigned int *) 0x1FFF7590;
unsigned int *uid_1 = (unsigned int *) 0x1FFF7594;
unsigned int *uid_2 = (unsigned int *) 0x1FFF7598;

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_control_task();       // code to be executed in real-time at 20kHz
void loop_application_task();   // code to be executed in the fast application task
int8_t AppTask_num;             // Application Task number

//
uint32_t led_refresh_sampling = 6*10000;
uint32_t application_task_counter;
//------------- PR RESONANT -------------------------------------
float32_t w0;
static LowPassFirstOrderFilter vHigh_filter(100e-6, 1e-3);
static float32_t vHigh_filtered;
//--------------USER VARIABLES DECLARATIONS----------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static float32_t Ts = (float32_t ) control_task_period * 1.0e-6F;
uint8_t received_serial_char;

/* Measure variables */
static float32_t V1_low_value;
static float32_t I1_low_value = 0.0;
static float32_t I2_low_value = 0.0;
static float32_t V_high;

static float32_t Vref = 0.0; // voltage reference from master

static float meas_data; // temp storage meas value (ctrl task)

/* duty_cycle*/
float32_t duty_cycle;
float32_t duty_cycle_leg1 = 0.2;
float32_t duty_cycle_leg2 = 0.2;
float32_t duty_cycle_leg1_mem = 0.2;
float32_t duty_cycle_leg2_mem = 0.2;
uint8_t is_pr_regul = 0;
/* Sinewave settings */
float f0 = 10;
float angle;
PrParams pr_params = PrParams(Ts, 0.001, 300.0, 0.0, 0.0, -48.0, 48.0);
Pr prop_res = Pr();


//debug 
uint8_t rms_counter = 0;
uint8_t oldest_index = 0;
float32_t I1_s = 0.;
float32_t I1_sq = 0.;
float32_t var_I1 = 0.;
float32_t I1_saved[16];

float32_t Vref_s = 0.;
float32_t Vref_sq = 0.;
float32_t var_Vref = 0.;
float32_t Vref_saved[16];



struct consigne_struct
{
    uint8_t buf_Vab[3];    // Contains Va and Vb reference
    uint8_t buf_VcIac[3];  // Contains Vc reference and IA OR IC measure
    uint8_t buf_IbW[3];    // Contains Ib and pulsation reference    //
    uint8_t id_and_status; // Contains master_status
};

struct consigne_struct tx_consigne;
struct consigne_struct rx_consigne;
uint8_t *buffer_tx = (uint8_t *)&tx_consigne;
uint8_t *buffer_rx = (uint8_t *)&rx_consigne;

uint8_t master_status;
uint32_t counter_time = 0;
uint32_t counter_led = 0;

three_phase_t Iabc_ref;
float32_t comp_dt = 0.010;
uint32_t counter;

//---------------------------------------------------------------

enum control_state_mode // 
{
    IDLE = 0,
    POWER = 1,
    OVERCURRENT = 2
};
enum control_state_mode control_state = IDLE;
const char *state_msg[] = {"IDLE", "POWER", "OVERCURRENT", NULL};

timing_t time_toc;
timing_t time_tic;
uint64_t total_cycles;
uint64_t total_ns;

uint32_t spin_mode(void) {
    return *uid_0;
}

static uint8_t rs_id;
void reception_function(void)
{
    spin.gpio.setPin(PC13);
    rs_id = GET_ID(rx_consigne.id_and_status);
    tx_consigne = rx_consigne;
    switch (SPIN_MODE) 
    { 
        case (SLAVE_A):
            /*reception of data*/
            if (rs_id == RS_ID_MASTER) {
                PUT_UPPER_12BITS(tx_consigne.buf_VcIac, to_12bits(I1_low_value, 40.0, 20.0));
                SET_ID(tx_consigne.id_and_status, RS_ID_SLAVE_A);
                if (control_state == OVERCURRENT)
                    SET_OVERCURRENT(tx_consigne.id_and_status);
                communication.rs485.startTransmission();
                n_receive_calls = (n_receive_calls + 1) % RECEIVE_MODULO;
            }
            break;  
        case (SLAVE_B):
            if (rs_id == RS_ID_SLAVE_A) 
            {
                PUT_LOWER_12BITS(tx_consigne.buf_IbW, to_12bits(I1_low_value, 40.0, 20.0));
                SET_ID(tx_consigne.id_and_status, RS_ID_SLAVE_B);
                if (control_state == OVERCURRENT)
                    SET_OVERCURRENT(tx_consigne.id_and_status);
                communication.rs485.startTransmission();
                n_receive_calls = (n_receive_calls + 1) % RECEIVE_MODULO;
            }
            break;
        case (SLAVE_C):
            if (rs_id == RS_ID_SLAVE_B)
            {
                PUT_UPPER_12BITS(tx_consigne.buf_VcIac, to_12bits(I1_low_value, 40.0, 20.0));
                SET_ID(tx_consigne.id_and_status, RS_ID_SLAVE_C);
                if (control_state == OVERCURRENT)
                    SET_OVERCURRENT(tx_consigne.id_and_status);
                communication.rs485.startTransmission();
                n_receive_calls = (n_receive_calls + 1) % RECEIVE_MODULO;
            }
            break;
        default:
            break;
    }
    spin.gpio.resetPin(PC13);
}

//--------------SETUP FUNCTIONS-------------------------------

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, twist, data and/or tasks.
 * In this example, we setup the version of the spin board and a background task.
 * The critical task is defined but not started.
 */
void setup_routine()
{
    //------- hardware init ----
    spin.version.setBoardVersion(TWIST_v_1_1_2);

    twist.setVersion(shield_TWIST_V1_2);
    twist.initAllBuck(); // initialize in buck mode leg1 and leg2
    communication.sync.initSlave(TWIST_v_1_1_2);
    timing_init();
    timing_start();
    spin.gpio.configurePin(PC12, OUTPUT);
    spin.gpio.configurePin(PC13, OUTPUT);
    //------ software init -----
    data.enableTwistDefaultChannels();
    switch (spin_mode()) {
        case UID_SLAVE_A:
            data.setParameters(I1_LOW, 4.56, -9367.00);
            data.setParameters(I2_LOW, 5.51, -11351.00);
            SPIN_MODE = SLAVE_A;
        break;
        case UID_SLAVE_B:
            data.setParameters(I1_LOW, 4.679, -9551.00);
            data.setParameters(I2_LOW, 4.523, -9281.00);
            SPIN_MODE = SLAVE_B;
        break;
        case UID_SLAVE_C:
            data.setParameters(I1_LOW, 5.406, -11426.00);
            data.setParameters(I2_LOW, 5.31, -10747.00);
            SPIN_MODE = SLAVE_C;
        break;
    }
	rx_consigne.id_and_status = 0;
    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(consigne_struct), reception_function, SPEED_20M); // RS485 at 20Mbits/s
    AppTask_num = task.createBackground(loop_application_task);
    task.startBackground(AppTask_num);
    task.createCritical(&loop_control_task, control_task_period);
    task.startCritical();
    // PR RESONANT
    prop_res.init(pr_params);
}

//--------------LOOP FUNCTIONS--------------------------------

/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
void loop_application_task()
{
    printk("%c: %s: %8.2f: %d\n", TXT_ID[SPIN_MODE], state_msg[control_state], (float32_t) total_ns*0.001, master_status);
    // if (counter != 0) printk("!!!");
    // Pause between two runs of the task
    task.suspendBackgroundMs(100);
}

/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
__attribute__((section(".ramfunc"))) void loop_control_task()
{
	spin.gpio.setPin(PC12);
    time_tic = timing_counter_get();
    counter_time++;
	counter_led= (counter_led + 1) % led_refresh_sampling;
    //--- MEASURES -------------------------------------------------------------
    meas_data = data.getLatest(I1_LOW);
    if (meas_data != NO_VALUE)
        I1_low_value = meas_data / 1000.0;

    meas_data = data.getLatest(V1_LOW);
    if (meas_data != NO_VALUE)
        V1_low_value = meas_data;

    meas_data = data.getLatest(I2_LOW);
    if (meas_data != NO_VALUE)
        I2_low_value = meas_data / 1000.0;

    meas_data = data.getLatest(V_HIGH);
    if (meas_data != NO_VALUE)
        V_high = meas_data;

    vHigh_filtered = vHigh_filter.calculateWithReturn(V_high);
    //--- GET DATA FROM RS485 --------------------------------------------------
    master_status = GET_STATUS(rx_consigne.id_and_status);
    switch (SPIN_MODE) {
        case SLAVE_A:
            Vref = from_12bits(GET_LOWER_12BITS(rx_consigne.buf_Vab), VOLTAGE_SCALE); 
            break;
        case SLAVE_B:
            Vref = from_12bits(GET_UPPER_12BITS(rx_consigne.buf_Vab), VOLTAGE_SCALE);
            break;
        case SLAVE_C:
            Vref = from_12bits(GET_LOWER_12BITS(rx_consigne.buf_VcIac), VOLTAGE_SCALE);
            break;
    }
    w0 = from_12bits(GET_UPPER_12BITS(rx_consigne.buf_IbW), 500.0, 0.0);
    if (REROLL_COUNTER(rx_consigne.id_and_status)) 
        stop_recording = 1;//counter = 0;
    else 
        stop_recording = 0;
    //--- STATE CALCULATION ----------------------------------------------------
    // if (  ((I1_low_value > CURRENT_LIMIT)
    //     || (I2_low_value > CURRENT_LIMIT) 
    //     || (I1_low_value < -CURRENT_LIMIT)
    //     || (I2_low_value < -CURRENT_LIMIT)) && counter_time > 10)
    //     control_state = OVERCURRENT;

    switch (control_state)
    {
        case IDLE:
			led_refresh_sampling = 6*10000; // 60*100ms
			if (master_status == POWER)
			{
				// twist.startLeg(LEG1);
				// twist.startLeg(LEG2);
				control_state = POWER;
			}
			break;
        case POWER:
			led_refresh_sampling = 3*10000; // 20*100ms
            if (master_status == IDLE) {
                control_state = IDLE;
            }
            break;
        case OVERCURRENT:
			led_refresh_sampling = (int)0.5*10000; // 4*100ms
            // we stay in overcurrent mode
            // if (master_status == IDLE)
            //     control_state = IDLE;
            break;
        default:
            control_state = IDLE;
    } 
    //--- CONTROL --------------------------------------------------------------
    if (control_state == POWER)
    {
        duty_cycle_leg1 = 1.0 / VOLTAGE_SCALE * Vref;

        if (w0 > 25)  { 
            is_pr_regul = 1;
        } else if (w0 < 5) {
            is_pr_regul = 0;
        }

        if (is_pr_regul == 1) { 
            prop_res.setW0(w0);
            duty_cycle_leg2 = 1.0 / VOLTAGE_SCALE * (Vref + prop_res.calculateWithReturn(I1_low_value, I2_low_value));
            //duty_cycle_leg2 = (Vref + pr_resonant(I1_low_value, I2_low_value, w0, 0.0, &pr_params)) / VOLTAGE_SCALE;
        } else if (is_pr_regul == 0) {
            duty_cycle_leg2 = 1.0 / VOLTAGE_SCALE * Vref; 
        }

        twist.setLegDutyCycle(LEG1, duty_cycle_leg1);
        twist.setLegDutyCycle(LEG2, duty_cycle_leg2);
    } else {
        twist.stopLeg(LEG1);
        twist.stopLeg(LEG2);
        Vref = 0.0;
    }

	if (counter_led > (led_refresh_sampling >> 1) ) {
		spin.led.turnOn();
	} else {
		spin.led.turnOff();
	}
	time_toc = timing_counter_get();
	total_cycles = timing_cycles_get(&time_tic, &time_toc);
	total_ns = timing_cycles_to_ns(total_cycles); 
	spin.gpio.resetPin(PC12);

}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}
