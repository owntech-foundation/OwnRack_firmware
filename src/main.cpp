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
 * SPDX-License-Identifier: LGLPV2.1
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

//----------- USER INCLUDE ----------------------
#include "pid.h"
#include "stm32g4xx.h"
#include "transform.h"
#include "filters.h"
#include "ScopeMimicry.h"

//------------ZEPHYR DRIVERS----------------------
#include "zephyr/kernel.h"
#include "zephyr/console/console.h"
#include <zephyr/timing/timing.h>

//---------- LL drivers --------------------------
#include "stm32_ll_gpio.h"
#include "stm32_ll_lpuart.h"
#include "stm32g4xx_ll_cordic.h"

#include <sys/_stdint.h>
#define DATACOUNT 8 // 8 - 1 : Can send 7 bytes of data

#define DMA_BUFFER_SIZE 10 // 3 byte for Va/Vb, 3 byte for Vc/(Ia/Ic), 3 byte for Ib and w, and 1 byte for identifiant
#define RS_ID_MASTER 0 
#define RS_ID_SLAVE_A 1 
#define RS_ID_SLAVE_B 2 
#define RS_ID_SLAVE_C 3 
#define CURRENT_LIMIT 9.0
#define VOLTAGE_SCALE 50.0 

#define GET_ID(id_and_status) ((id_and_status >> 6) & 0x3)        // retrieve identifier
#define SET_ID(id_and_status, value) { \
    id_and_status &= 0x3F;\
    id_and_status |= ((value & 0x3) << 6);\
    }

#define IS_OVERCCURRENT(id_and_status)  ((id_and_status >> 5) & 1) // check current for safety
#define GET_STATUS(id_and_status)       (id_and_status & 1)        // check the status (IDLE MODE or POWER MODE)
#define REROLL_COUNTER(id_and_status)   ((id_and_status >> 1) & 1) // reroll counter to monitor the intern variables
#define SET_STATUS_IDLE(id_and_status)  (id_and_status &= 0xFE)    // assume that id_and_status is only one byte
#define SET_STATUS_POWER(id_and_status) (id_and_status |= 0x1)     // bit 0 set to 1)
#define RESET_COUNTER(id_and_status)    (id_and_status |= 0x2)     // bit 1 set to 1
#define KEEP_COUNT(id_and_status)       (id_and_status &= 0x0fd)   // bit 1 set to 0

#define GET_UPPER_12BITS(buf_3bytes) ((int16_t) ((*(buf_3bytes + 2) << 4) + (*(buf_3bytes + 1) >> 4)))

#define GET_LOWER_12BITS(buf_3bytes) ((int16_t) (((*(buf_3bytes + 1) & 0xf)<<8) + (*(buf_3bytes ) & 0xff)))

#define PUT_UPPER_12BITS(buf_3bytes, data) { \
    *(buf_3bytes+1) = (0xf0 & (data << 4)) | (0xf & *(buf_3bytes+1));\
    *(buf_3bytes+2)  = (data & 0xff0) >> 4; \
    }

#define PUT_LOWER_12BITS(buf_3bytes, data) {\
    *buf_3bytes = data & 0xff; \
    *(buf_3bytes+1) = (0xf0 & *(buf_3bytes+1)) | ((data & 0xf00) >> 8); \
    }

int16_t to_12bits(float32_t data, float32_t scale, float32_t offset=0.0) 
{
    return (int16_t) ((data + offset) * 4095.0 / (scale));
}

float32_t from_12bits(int16_t data, float32_t scale, float32_t offset=0.0)
{
    return (scale * (float32_t) data) / 4095.0  - offset;
}

__inline__ float32_t q31_to_f32(int32_t x)
{
	// 1<<31
	return (float32_t) x / 2147483648.0f;
}

void ot_sqrt(float32_t in, float32_t *out);

float32_t ot_sqrt(float32_t in)
{
	float32_t out;
	__ASM("VSQRT.F32 %[output],%[input]" : [output] "=t"(out) : [input] "t"(in));
	return out;
}

PllDatas pll_datas;
static float32_t w;

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system

//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_communication_task(); // code to be executed in the slow communication task
int8_t CommTask_num;            // Communication Task number
void loop_application_task();   // code to be executed in the fast application task
int8_t AppTask_num;             // Application Task number
void loop_control_task();       // code to be executed in real-time at 20kHz

static uint32_t control_task_period = 100; //[us] period of the control task
static bool pwm_enable = false;            //[bool] state of the PWM (ctrl task)

static LowPassFirstOrderFilter vHigh_filter(100e-6, 1e-3);
// --- SIN-COS SCALING ------------------------------------------
// en haute impedance.
const float SINCOS_AMP = 1.0 / 620.0; // 1.0 / 588.0;
const float SINCOS_OFFSET = 1315;//1576.0;

int32_t cos_theta;
int32_t sin_theta;
float32_t angle_cordic;

//--------------FOR PARK CONTROL --------------------------------
/* Sinewave settings */
static float f0_ref = 0;
static float angle;
static float Iq_ref;
// control
static dqo_t Idq;
static dqo_t Vdq;
static dqo_t Idq_ref;
static three_phase_t Vabc;
static three_phase_t Iabc;
static float32_t Vdq_module;
static float32_t Idq_module;
static Transform transform;
static Pid pi_d;
static Pid pi_q;
static PllAngle pllAngle;
static const float32_t Ts = control_task_period * 1.0e-6F;
static const float32_t Kp = 0.035F;
static const float32_t Ti = 0.001029F;
static const float32_t Td = 0.0F;
static const float32_t N = 1.0F;
static const float32_t lower_bound = -60.0F;
static const float32_t upper_bound = 60.0F;

static float32_t Voffset = 25.0;
static float32_t startup_time;
const static float32_t startup_end_time = 1000.0;

static float32_t I1_low_value_from_slave_a;
static float32_t I1_low_value_from_slave_b;
static float32_t I1_low_value_from_slave_c;

//------------- PR RESONANT -------------------------------------
float32_t w0;

//--------------USER VARIABLES DECLARATIONS----------------------


uint8_t received_serial_char;

/* duty_cycle*/
float32_t duty_cycle;
float32_t duty_cycle_leg1 = 0.2;
float32_t duty_cycle_leg2 = 0.2;
float32_t duty_cycle_leg1_mem = 0.2;
float32_t duty_cycle_leg2_mem = 0.2;

float32_t V_a;
float32_t V_b;
float32_t V_c;
float32_t Vdc;

uint16_t count_delay;

// TODO: test structure bit field for id_and_status ?
struct consigne_struct
{
    uint8_t buf_Vab[3];    // Contains Va and Vb reference
    uint8_t buf_VcIac[3];  // Contains Vc reference and IA OR IC measure
    uint8_t buf_IbW[3];    // Contains Ib and frequency reference    //
    uint8_t id_and_status; // Contains ID [7:6], OVERCURRENT [5], REROLL_COUNTER[1], STATUS [0]
};

struct consigne_struct tx_consigne;
struct consigne_struct rx_consigne;
uint8_t *buffer_tx = (uint8_t *)&tx_consigne;
uint8_t *buffer_rx = (uint8_t *)&rx_consigne;

uint8_t status;
uint32_t counter_time;
uint8_t decimation = 1;
uint8_t delay = 0;
three_phase_t Iabc_ref;
ScopeMimicry scope(1024, 9);
static bool is_downloading = false;
static bool stop_recording = 0;
float32_t sin_value;
float32_t cos_value;

/**
 *  First The ownverter will send reference data in this order.
 *  All the datas are coded in 12 bits with the exception of the status which is 8 bit
 *  and are sent in this order :
 *   +-----------+-----------+-----------+-----------+-----------+
 *   |    Va     |    Vb     |    Vc     |     W0    |   status  |
 *   +-----------+-----------+-----------+-----------+-----------+
 *
 * Then the bridge will sent back measures.
 *   +-----------+-----------+-----------+-----------+-----------+
 *   |    Ia     |    Ib     |    Ic     |     Vdc   |   status  |
 *   +-----------+-----------+-----------+-----------+-----------+
 */
struct datas
{
    uint8_t buf_dataA_dataB[3]; // Contains Va reference OR Ia measure and Vb reference OR Ib measure
    uint8_t buf_dataC_dataD[3]; // Contains Vc reference OR Ic measure and W0 reference OR Vdc measure
    uint8_t status;
};

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
    IDLEMODE = 0,
    POWERMODE = 1,
};

enum control_state_mode {
    IDLE=0,
    POWER=1,
    OVERCURRENT=2,
    STARTUP=3
};

control_state_mode control_state = IDLE;
serial_interface_menu_mode mode_asked = IDLEMODE;
uint32_t cumul_receiv_datas = 0;
uint32_t last_receive_calls = 0;
uint32_t n_receive_calls = 0;
bool sync_problem = 0;

void reception_function(void)
{
    if (GET_ID(rx_consigne.id_and_status) == RS_ID_SLAVE_A)
    {
        // get I value from phase A
        I1_low_value_from_slave_a = from_12bits(GET_UPPER_12BITS(rx_consigne.buf_VcIac), 40.0, 20.0); 
    }
    if (GET_ID(rx_consigne.id_and_status) == RS_ID_SLAVE_B)
    {
        // get I value from phase B
        I1_low_value_from_slave_b = from_12bits(GET_LOWER_12BITS(rx_consigne.buf_IbW), 40.0, 20.0);
    }
    else if (GET_ID(rx_consigne.id_and_status) == RS_ID_SLAVE_C)
    {
        // get I value from phase C
        I1_low_value_from_slave_c = from_12bits(GET_UPPER_12BITS(rx_consigne.buf_VcIac), 40.0, 20.0);
    }
    if (IS_OVERCCURRENT(rx_consigne.id_and_status)) // check if one phase experienced a current > 8A and shut off everything
        control_state = OVERCURRENT;
    n_receive_calls++;
}
bool scope_trigger(void) {
    if (stop_recording == 0 && control_state != OVERCURRENT)
        return false;
    else 
        return true; 
}
void dump_scope_datas(ScopeMimicry &scope)  {
    uint8_t *buffer = scope.get_buffer();
    uint16_t buffer_size = scope.get_buffer_size() >> 2; // we divide by 4 (4 bytes per float data) 
    printk("begin record\n");
    printk("#");
    for (uint16_t k=0;k < scope.get_nb_channel(); k++) {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    printk("# %d\n", scope.get_final_idx());
    for (uint16_t k=0;k < buffer_size; k++) {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
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
    console_init();

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
    
    LL_CORDIC_Config(CORDIC,
                     LL_CORDIC_FUNCTION_PHASE, /* phase function */
                     LL_CORDIC_PRECISION_6CYCLES, /* max precision for q1.31 cosine */
                     LL_CORDIC_SCALE_0, /* no scale */
                     LL_CORDIC_NBWRITE_2, /* x first and y after */
                     LL_CORDIC_NBREAD_1, /* angle to read */
                     LL_CORDIC_INSIZE_32BITS, /* q1.31 format for input data */
                     LL_CORDIC_OUTSIZE_32BITS); 


    spin.version.setBoardVersion(TWIST_v_1_1_2);
    twist.setVersion(shield_ownverter); // Code only compatible for TWIST V1.2 for V1.3 change the version and enable the NGND
    twist.initAllBuck();            // We need it to start HRTIM clock for control task interruption
    communication.sync.initMaster(); // start the synchronisation

    //------ software init -----

    // dataAcquisition.enableTwistDefaultChannels();

    spin.adc.configureTriggerSource(2, hrtim_ev3);
    data.enableShieldChannel(2, ANALOG_SIN);
    data.enableShieldChannel(2, ANALOG_COS);
    
    task.createCritical(&loop_control_task, control_task_period);
    task.startCritical();

    AppTask_num = task.createBackground(loop_application_task);
    task.startBackground(AppTask_num);

    CommTask_num = task.createBackground(loop_communication_task);
    task.startBackground(CommTask_num);

    //TODO: make watchdog in each slave (if no data received between two count: disable pwm)
    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(consigne_struct), reception_function, SPEED_20M); // custom configuration for RS485

    scope.set_trigger(scope_trigger);
    scope.connectChannel(angle_cordic, "angle_cordic");
    scope.connectChannel(angle, "angle_pll");
    scope.connectChannel(V_a, "V_a");
    scope.connectChannel(V_b, "V_b");
    scope.connectChannel(V_c, "V_c");
    scope.connectChannel(Iabc.a, "I_a");
    scope.connectChannel(Iabc.b, "I_b");
    scope.connectChannel(Iabc.c, "I_c");
    scope.connectChannel(w, "w");
    scope.start();
    /* for park control */
    PidParams pid_p(Ts, Kp, Ti, Td, N, lower_bound, upper_bound);
    // pi for d axis
    pi_d.init(pid_p);
    pi_d.reset();
    // pi for q axis
    pi_q.init(pid_p);
    pi_q.reset();
    if (pllAngle.init(Ts, 400.0, 0.05F) == -EINVAL) {
        control_state = OVERCURRENT; 
    }
    
    pllAngle.reset(0.F);
    Vabc.a = 0.0;
    Vabc.b = 0.0;
    Vabc.c = 0.0;
    Vdq.d = 0.0;
    Vdq.q = 0.0;
    Iabc.a = 0.0;
    Iabc.b = 0.0;
    Iabc.c = 0.0;
}
//--------------LOOP FUNCTIONS--------------------------------
void loop_communication_task()
{
        received_serial_char = console_getchar();
        switch (received_serial_char)
    {
        case 'h':
            //----------SERIAL INTERFACE MENU-----------------------
            printk(" ________________________________________\n");
            printk("|     ------- MENU ---------             |\n");
            printk("|     press i : idle mode                |\n");
            printk("|     press p : power mode               |\n");
            printk("|     press u : Iqref UP                 |\n");
            printk("|     press d : Iqref DOWN               |\n");
            printk("|     press j : f0    UP                 |\n");
            printk("|     press k : f0    DOWN               |\n");
            printk("|     press r : toggle recording         |\n");
            printk("|     press s : dump datas recorded      |\n");
            printk("|________________________________________|\n\n");
            //------------------------------------------------------
            break;
        case 'i':
            mode_asked = IDLEMODE;
            cumul_receiv_datas = 0;
            Iq_ref = 0.0;
            break;
        case 'p':
            mode_asked = POWERMODE;
            printk("power\n");
            break;
        case 'u': 
            if (Iq_ref < CURRENT_LIMIT)
            {
                Iq_ref += 0.5;
            }
            break;
        case 'd':
            if (Iq_ref > 0.0)
                Iq_ref -= 0.5;
            break;
        case 'j':
            if (f0_ref < 400.0)
                f0_ref += .1;
            break;
        case 'k':
            if (f0_ref > 0.0)
                f0_ref -= .1;
            break;
        case 'r':
            stop_recording = (stop_recording + 1) & 0x1;
            if (stop_recording == 0) {
                scope.start();
            }
            break;
        case 's':
            is_downloading = true;
        default:
            break;
    }
}
/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */
char status_icon[2][4] = {
    {0xF0, 0x9F, 0x91, 0x8C},
    {0xF0, 0x9F, 0x92, 0xA9}};

void loop_application_task()
{
    int id = sync_problem; 
    printk("%i:", control_state);
    printk("%.2f:", Iq_ref);
    printk("%.2f:", f0_ref);
    printk("%d:", sync_problem);
    printk("%d:", cumul_receiv_datas);
    printk("%d:", last_receive_calls);
    printk("%c%c%c%c:", 
           status_icon[id][0],
           status_icon[id][1],
           status_icon[id][2],
           status_icon[id][3]);
    if (stop_recording) 
        printk("\n");
    else 
        printk("REC\n");
    if (control_state == POWER)
    {
        spin.led.turnOn();
    } else {
        spin.led.turnOff();
    }
    if (is_downloading && control_state != POWER) {
        dump_scope_datas(scope);
        is_downloading = 0;
    }
    // Pause between two runs of the task
    task.suspendBackgroundMs(200);
}
/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be interruped.
 * It is from it that you will control your power flow.
 */
__attribute__((section(".ramfunc"))) void loop_control_task()
{
    uint8_t data_validity;
    float value;
    counter_time++;
    
    value = data.getLatest(ANALOG_SIN, &data_validity);
    if (data_validity == DATA_IS_OK)
    {
        sin_value = SINCOS_AMP * (value - SINCOS_OFFSET);
    }
    value = data.getLatest(ANALOG_COS, &data_validity);
    if (data_validity == DATA_IS_OK)
    {
        cos_value = SINCOS_AMP * (value - SINCOS_OFFSET);
    }
    // ANGLE ACQUISITION, ANGLE CLOSE LOOP
    cos_theta = (int32_t) (cos_value * 2147483648.0);
    sin_theta = (int32_t) (sin_value * 2147483648.0);
    LL_CORDIC_WriteData(CORDIC, cos_theta); // give X data
    LL_CORDIC_WriteData(CORDIC, sin_theta); // give y data
    // angle_cordic = PI * q31_to_f32((int32_t)LL_CORDIC_ReadData(CORDIC)); // retrieve phase of x+iy
    w0 = 2.0F * PI * f0_ref;
    // ANGLE OPEN LOOP
    angle_cordic += w0 * control_task_period * 1.0e-6F;
    angle_cordic  = ot_modulo_2pi(angle);
    
    pll_datas = pllAngle.calculateWithReturn(angle_cordic);
    angle = ot_modulo_2pi((4.0 * (PI - pll_datas.angle)));
    w = pll_datas.w;

    switch (control_state)
    {
        case IDLE:
            if (mode_asked == POWERMODE)
            {
                control_state = STARTUP;
                startup_time = 0.0;
            }
        break;
        case STARTUP:
            startup_time++;
            if (startup_time >= startup_end_time) {
                control_state = POWER;
            }
        break;
        case POWER:
            if (mode_asked == IDLEMODE) {
                control_state = IDLE;
            }
        break;
        case OVERCURRENT:
            if (mode_asked == IDLEMODE) {
                control_state = IDLE;
            }
        break;
        default:
        break;
    }


    if (control_state == IDLE || control_state == OVERCURRENT)
    {
        pi_d.reset(0.0F);
        pi_q.reset(0.0F);
        pllAngle.reset(0.0F);
        SET_STATUS_IDLE(tx_consigne.id_and_status);
        SET_ID(tx_consigne.id_and_status, RS_ID_MASTER);
        if (stop_recording == 1)
           RESET_COUNTER(tx_consigne.id_and_status); 
        else
           KEEP_COUNT(tx_consigne.id_and_status);
        pwm_enable = false;
    }
    else if (control_state == STARTUP) {
        float32_t ratio = startup_time / startup_end_time;
        Vabc.a = ratio * Voffset;
        Vabc.b = ratio * Voffset;
        Vabc.c = ratio * Voffset;

        Iabc.a  = I1_low_value_from_slave_a;
        Iabc.b  = I1_low_value_from_slave_b;
        Iabc.c  = I1_low_value_from_slave_c;

    }
    else if (control_state == POWER)
    {

        Idq_ref.q = Iq_ref;
        Idq_ref.d = 0.0;
        Iabc_ref = transform.to_threephase(Idq_ref, angle);

        Iabc.a  = I1_low_value_from_slave_a;
        Iabc.b  = I1_low_value_from_slave_b;
        Iabc.c  = I1_low_value_from_slave_c;

        Idq = transform.to_dqo(Iabc, angle);

        Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);
        Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);

        Idq_module = ot_sqrt(Idq.d*Idq.d + Idq.q*Idq.q);
        Vdq_module = ot_sqrt(Vdq.d*Vdq.d + Vdq.q*Vdq.q);
        // REMOVE CURRENT CONTROL
        //Vdq.d = 0.0;
        //Vdq.q = Iq_ref;
        Vabc = transform.to_threephase(Vdq, angle);
        Vabc.a += Voffset; 
        Vabc.b += Voffset; 
        Vabc.c += Voffset; 
    }

    if (control_state == STARTUP || control_state == POWER)
    {
        if (!pwm_enable)
            pwm_enable = true;

        /* Writting Va */
        PUT_LOWER_12BITS(tx_consigne.buf_Vab, to_12bits(Vabc.a, VOLTAGE_SCALE));

        /* Writting Vb */
        PUT_UPPER_12BITS(tx_consigne.buf_Vab, to_12bits(Vabc.b, VOLTAGE_SCALE));

        /* Writting Vc */
        PUT_LOWER_12BITS(tx_consigne.buf_VcIac, to_12bits(Vabc.c, VOLTAGE_SCALE));

        /* Writting frequency reference */
        PUT_UPPER_12BITS(tx_consigne.buf_IbW, to_12bits(pll_datas.w, 500.0)); 

        if (stop_recording == 1)
           RESET_COUNTER(tx_consigne.id_and_status); 
        else
           KEEP_COUNT(tx_consigne.id_and_status);

        SET_STATUS_POWER(tx_consigne.id_and_status);
        SET_ID(tx_consigne.id_and_status, RS_ID_MASTER);
    }
    // scope.acquire();
    if (counter_time > 1000) { // Wait to be sure that other cards are power on.
        cumul_receiv_datas += n_receive_calls;
        last_receive_calls = n_receive_calls;
        //if ((n_receive_calls != 3))
        //    sync_problem = 1; 
        n_receive_calls = 0;
        communication.rs485.startTransmission();
    }
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
