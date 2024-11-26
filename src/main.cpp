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
//----------- USER INCLUDE ----------------------
#include "pid.h"
#include "transform.h"
#include "filters.h"
#include "ScopeMimicry.h"
#include "control_factory.h"
//------------ZEPHYR DRIVERS----------------------
#include "zephyr/console/console.h"
#include <zephyr/timing/timing.h>
#include <zephyr/kernel.h>
#include <string.h>
//---------- LL drivers --------------------------
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_tim.h"
#include <stm32_ll_bus.h>


#define DATACOUNT 8 // 8 - 1 : Can send 7 bytes of data
#define DMA_BUFFER_SIZE                                                                            \
	10 // 3 byte for Va/Vb, 3 byte for Vc/(Ia/Ic), 3 byte for Ib and w, and 1 byte for
	   // identifiant
enum rs_id {
	BRIDGE = 1,
	MASTER_A = 2,
	MASTER_B = 3,
	MASTER_C = 4,
	SLAVE_A = 5,
	SLAVE_B = 6,
	SLAVE_C = 7
};

enum rs_id spin_mode;

struct bridge_frame {
	uint16_t Va:12;
	uint16_t Vb:12;
	uint16_t Vc:12;
	uint16_t w:12;
	uint8_t stop_recording:1;
	uint16_t no_data1:11;
	uint16_t no_data2:12;
	uint8_t abx_cmd:4;
	uint8_t id:4;
} __packed;

typedef bridge_frame bridge_frame_t;

bridge_frame_t data_bridge;

struct phase_frame {
	uint16_t V1:12;
	uint16_t V2:12;
	uint16_t I1:12;
	uint16_t I2:12;
	uint16_t VH:12;
	uint16_t IH:12;
	uint8_t status:4;
	uint8_t id:4;
} __packed;

typedef phase_frame phase_frame_t;

static phase_frame_t data_master_a;
static phase_frame_t data_master_b;
static phase_frame_t data_master_c;
static phase_frame_t data_slave;
static phase_frame_t datas_to_send;

static float32_t I2_master_a;
static float32_t I1_master_c;
static float32_t I2_master_b;
static float32_t I2_master_c;

uint8_t buffer_tx[10];
uint8_t buffer_rx[10];
//
enum autobox_msg {
	STOP = 0, 
	START = 1
};
static enum autobox_msg abx_cmd;

#define RS_ID_MASTER  0
#define CURRENT_LIMIT 9.0F
#define VOLTAGE_SCALE 50.0F
#define GET_ID(buffer_rx) ((buffer_rx[9] >> 4) & 0xf)

int16_t to_12bits(float32_t data, float32_t scale, float32_t offset = 0.0)
{
	return (int16_t)((data + offset) * 4095.0F / (scale));
}

inline float32_t from_12bits(int16_t data, float32_t scale, float32_t offset = 0.0)
{
	return (scale * (float32_t)data) / 4095.0F - offset;
}

__inline__ float32_t q31_to_f32(int32_t x)
{
	// 1<<31
	return (float32_t)x / 2147483648.0f;
}

void ot_sqrt(float32_t in, float32_t *out);

float32_t ot_sqrt(float32_t in)
{
	float32_t out;
	__ASM("VSQRT.F32 %[output],%[input]" : [output] "=t"(out) : [input] "t"(in));
	return out;
}


timing_t time_toc;
timing_t time_tic;
uint64_t total_cycles;
uint64_t total_ns;
//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system
//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_communication_task(); // code to be executed in the slow communication task
int8_t CommTask_num;            // Communication Task number
void loop_application_task();   // code to be executed in the fast application task
int8_t AppTask_num;             // Application Task number
void loop_control_task();       // code to be executed in real-time at 20kHz

static uint32_t control_task_period = 110; //[us] period of the control task
static LowPassFirstOrderFilter vHigh_filter(100e-6, 1e-3);

static LowPassFirstOrderFilter Id_filter(100e-6, 10e-3);
static LowPassFirstOrderFilter Iq_filter(100e-6, 10e-3);

static float32_t Id_filtered;
static float32_t Iq_filtered;

//---- HALL SENSOR FILTER ----------------------------------------

/* Hall effect sensors */
#define HALL1 PC6
#define HALL2 PC7
#define HALL3 PD2

static uint8_t HALL1_value;
static uint8_t HALL2_value;
static uint8_t HALL3_value;

static const float32_t Ts = 100.e-6F;

static uint8_t angle_index;
static float32_t hall_angle;
static PllAngle pllangle = controlLibFactory.pllAngle(Ts, 1.0F, 0.04F);
static PllDatas pllDatas;
static float32_t angle_filtered;
static float32_t w_meas;
static float32_t f_meas;
/*
 * one sector for one index value
 * index = H1*2^0 + H2*2^1 + H3*2^2
 */
static int16_t sector[] = {-1, 5, 1, 0, 3, 4, 2};
static float32_t k_angle_offset = 0.0F;

static LowPassFirstOrderFilter w_mes_filter = controlLibFactory.lowpassfilter(Ts, 5.0e-3F);



// --- SIN-COS SCALING ------------------------------------------
// en haute impedance.
const float SINCOS_AMP = 1.0 / 620.0; // 1.0 / 588.0;
const float SINCOS_OFFSET = 1315;     // 1576.0;

int32_t cos_theta;
int32_t sin_theta;
float32_t angle_cordic;

//--------------FOR PARK CONTROL --------------------------------
/* Sinewave settings */
static float f0_ref = 0.0F;
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
PllDatas pll_datas;
static float32_t w;
static PllAngle pllAngle;
// static const float32_t Ts = control_task_period * 1.0e-6F;
static const float32_t Kp = 0.035F;
static const float32_t Ti = 0.001029F;
static const float32_t Td = 0.0F;
static const float32_t N = 1.0F;
static const float32_t lower_bound = -15.0F; // -Vhigh
static const float32_t upper_bound = 15.0F;  // +Vhigh

/* assume the DC Bus is 48V (~50V) */
static float32_t Voffset = 15.0;  //Vhigh divided by 2
static float32_t startup_time;
const static float32_t startup_end_time = 1000.0;

static float32_t I1_low_value_from_slave_a;
static float32_t I1_low_value_from_slave_b;
static float32_t I1_low_value_from_slave_c;
// for open loop test
float32_t w0_ref;
//--------------USER VARIABLES DECLARATIONS----------------------
uint8_t received_serial_char;

/* duty_cycle*/
float32_t duty_cycle;
float32_t duty_cycle_leg1 = 0.2;
float32_t duty_cycle_leg2 = 0.2;

uint32_t counter_time;
three_phase_t Iabc_ref;
ScopeMimicry scope(512, 13);
static bool is_downloading = false;
static bool stop_recording = 0;
uint32_t decimation = 2;
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
struct datas {
	uint8_t buf_dataA_dataB[3]; // Contains Va reference OR Ia measure and Vb reference OR Ib
				    // measure
	uint8_t buf_dataC_dataD[3]; // Contains Vc reference OR Ic measure and W0 reference OR Vdc
				    // measure
	uint8_t status;
};

//---------------------------------------------------------------

enum serial_interface_menu_mode // LIST OF POSSIBLE MODES FOR THE OWNTECH CONVERTER
{
	IDLEMODE = 0,
	POWERMODE = 1,
};

enum control_state_mode {
	IDLE = 0,
	POWER = 1,
	ERROR_STATE = 2,
	STARTUP = 3
};
control_state_mode control_state = IDLE;
serial_interface_menu_mode mode_asked = IDLEMODE;
uint32_t cumul_receiv_datas = 0;
uint32_t last_receive_calls = 0;
uint32_t n_receive_calls = 3;
bool sync_problem = false;
static uint8_t rs_id;
char status_icon[2][4] = {{0xF0, 0x9F, 0x91, 0x8C}, {0xF0, 0x9F, 0x92, 0xA9}};

inline void get_angle_and_pulsation_sin_cos() {
	uint8_t data_validity;
	float32_t value;
	
	value = data.getLatest(ANALOG_SIN, &data_validity);
	
	if(value > value_max) value_max = value;
	if(value < value_min) value_min = value;

	if (data_validity == DATA_IS_OK) {
		sin_value = SINCOS_AMP * (value - SINCOS_OFFSET);
	}
	
	if(sin_value > sin_value_max) sin_value_max = sin_value;
	if(sin_value < sin_value_min) sin_value_min = sin_value;

	value = data.getLatest(ANALOG_COS, &data_validity);
	
	if (data_validity == DATA_IS_OK) {
		cos_value = SINCOS_AMP * (value - SINCOS_OFFSET);
	}
	// ANGLE ACQUISITION, ANGLE CLOSE LOOP
	cos_theta = (int32_t)(cos_value * 2147483648.0F);
	sin_theta = (int32_t)(sin_value * 2147483648.0F);
	
	LL_CORDIC_WriteData(CORDIC, cos_theta); // give X data
	
	LL_CORDIC_WriteData(CORDIC, sin_theta); // give y data

	
	angle_cordic = PI * q31_to_f32((int32_t)LL_CORDIC_ReadData(CORDIC)); // retrieve phase of
	
	// x+iy
	// w0_ref = 2.0F * PI * f0_ref;
	// ANGLE OPEN LOOP
	
	// angle_cordic += w0_ref * control_task_period * 1.0e-6F;
	
	// angle_cordic = ot_modulo_2pi(angle_cordic);
	
	pll_datas = pllAngle.calculateWithReturn(angle_cordic);
	
	angle_filtered = ot_modulo_2pi(pll_datas.angle);
	w_meas = pll_datas.w;
	f_meas = pll_datas.w/(2*PI * -10);
}

/**
 * @brief a periodmeter function which estimate pulsation
 * for the sector variable (one integer value correspond to π/3).
 *
 * @param sector assume sector is integer in [0, 5]
 * @param time in [s]
 * @return pulsation (float)
 */
float32_t pulsation_estimator(int16_t sector, float32_t time)
{
	static float32_t w_estimate_intern = 0.0F;
	static int16_t prev_sector;
	static float32_t prev_time = 0.0F;
	int16_t delta_sector;
	float32_t sixty_degre_step_time;
	delta_sector = sector - prev_sector;
	prev_sector = sector;
	if (delta_sector == 1 || delta_sector == -5) {
		// positive speed
		sixty_degre_step_time = (time - prev_time);
		w_estimate_intern = (PI / 3.0) / sixty_degre_step_time;
		prev_time = time;
	}
	if (delta_sector == -1 || delta_sector == 5) {
		sixty_degre_step_time = (time - prev_time);
		w_estimate_intern = (-PI / 3.0) / sixty_degre_step_time;
		prev_time = time;
	}
	return w_estimate_intern;
}

inline void get_position_and_speed()
{
	// we build angle index using HALL values.
	HALL1_value = spin.gpio.readPin(HALL1);
	HALL2_value = spin.gpio.readPin(HALL2);
	HALL3_value = spin.gpio.readPin(HALL3);

	angle_index = HALL3_value * 4 + HALL2_value * 2 + HALL1_value * 1;

	hall_angle = ot_modulo_2pi(PI / 3.0 * sector[angle_index] + PI * k_angle_offset / 24.0);  //offset may be used to fine tune angle measurement

	// w_estimate = pulsation_estimator(sector[angle_index], counter_time * Ts);

	pllDatas = pllangle.calculateWithReturn(hall_angle);

	angle_filtered = pllDatas.angle;
	w_meas = w_mes_filter.calculateWithReturn(pllDatas.w);

	// ANGLE OPEN LOOP
	w0_ref = 2.0F * PI * f0_ref;
	
	angle_filtered += w0_ref * control_task_period * 1.0e-6F;
	
	angle_filtered = ot_modulo_2pi(angle_filtered);
	w_meas = w0_ref;



}


inline void get_rs485_datas() 
{
	I2_master_a = from_12bits(data_master_a.I2, 50.0, 25.0);
	I1_master_c = from_12bits(data_master_c.I1, 50.0, 25.0);
	I2_master_c = from_12bits(data_master_c.I2, 50.0, 25.0);
	I2_master_b = from_12bits(data_master_b.I2, 50.0, 25.0);

	I1_low_value_from_slave_a = from_12bits(data_master_a.I1, 50.0, 25.0);
	I1_low_value_from_slave_b = from_12bits(data_master_b.I1, 50.0, 25.0);
	I1_low_value_from_slave_c = from_12bits(data_master_c.I1, 50.0, 25.0);
}

inline void define_tx_datas() 
{
	/* Writting Va */
	data_bridge.Va = to_12bits(Vabc.a, VOLTAGE_SCALE);
	/* Writting Vb */
	data_bridge.Vb = to_12bits(Vabc.b, VOLTAGE_SCALE);
	/* Writting Vc */
	data_bridge.Vc = to_12bits(Vabc.c, VOLTAGE_SCALE);
	/* Writting frequency reference */
	// data_bridge.w = (0xFFF & (counter_time>>6));//to_12bits(pll_datas.w, 500.0);
	data_bridge.w = to_12bits(w_meas, 500.0);
	data_bridge.stop_recording = scope.has_trigged();
	data_bridge.id = BRIDGE;
	memcpy(buffer_tx, &data_bridge, sizeof(data_bridge));
}

void reception_function(void)
{
	// spin.gpio.setPin(PC7);
	rs_id = GET_ID(buffer_rx);
	switch (rs_id) {
		case MASTER_A:
			data_master_a = *(phase_frame *) buffer_rx;
		break;
	case MASTER_B:
			data_master_b = *(phase_frame *) buffer_rx;
		break;
	case MASTER_C:
			data_master_c = *(phase_frame *) buffer_rx;
		break;
	}
	n_receive_calls++;
	// spin.gpio.resetPin(PC7);
}
bool scope_trigger(void)
{
	if ((stop_recording == 1 && control_state == POWER) || control_state == ERROR_STATE)
	{
		return true;
	}
	else 
	{
		return false;
	}
}
void dump_scope_datas(ScopeMimicry &scope)
{
	uint8_t *buffer = scope.get_buffer();
	uint16_t buffer_size =
		scope.get_buffer_size() >> 2; // we divide by 4 (4 bytes per float data)
	printk("begin record\n");
	printk("#");
	for (uint16_t k = 0; k < scope.get_nb_channel(); k++) {
		printk("%s,", scope.get_channel_name(k));
	}
	printk("\n");
	printk("# %d\n", scope.get_final_idx());
	for (uint16_t k = 0; k < buffer_size; k++) {
		printk("%08x\n", *((uint32_t *)buffer + k));
		task.suspendBackgroundUs(100);
	}
	printk("end record\n");
}

void launch_transmission(void) {
 // Check whether update interrupt is pending
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6)) {
        // Clear the update interrupt flag
        LL_TIM_ClearFlag_UPDATE(TIM6);
		// spin.gpio.setPin(PC7);
		communication.rs485.startTransmission();
		// spin.gpio.resetPin(PC7);
    }
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
	// variable, software initialisation
	spin.gpio.configurePin(HALL1, INPUT);
	spin.gpio.configurePin(HALL2, INPUT);
	spin.gpio.configurePin(HALL3, INPUT);

	scope.set_trigger(scope_trigger);
	scope.set_delay(0.9);
	scope.connectChannel(angle_cordic, "angle_cordic");
	scope.connectChannel(angle_filtered, "angle_hal");
	scope.connectChannel(Vabc.a, "V_a");
	scope.connectChannel(Vabc.b, "V_b");
	scope.connectChannel(Idq.d, "I_d");
	scope.connectChannel(Iabc.a, "I_a");
	scope.connectChannel(Iabc.b, "I_b");
	scope.connectChannel(Idq.q, "I_q");
	scope.connectChannel(I2_master_a, "I2_master_a");
	scope.connectChannel(I1_master_c, "I1_master_c");
	scope.connectChannel(I2_master_c, "I2_master_c");
	scope.connectChannel(I2_master_b, "I2_master_b");
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
	if (pllAngle.init(Ts, 10.0, 0.05F) == -EINVAL) {
		// control_state = OVERCURRENT;
	}
	pllAngle.reset(0.F);
	pllangle.reset(0.F);  // attention, lowercase a (to be changed)
	Vabc.a = 0.0;
	Vabc.b = 0.0;
	Vabc.c = 0.0;
	Vdq.d = 0.0;
	Vdq.q = 0.0;
	Iabc.a = 0.0;
	Iabc.b = 0.0;
	Iabc.c = 0.0;

	//------- hardware init ----
	console_init();

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);

	LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_PHASE, /* phase function */
			 LL_CORDIC_PRECISION_6CYCLES,      /* max precision for q1.31 cosine */
			 LL_CORDIC_SCALE_0,                /* no scale */
			 LL_CORDIC_NBWRITE_2,              /* x first and y after */
			 LL_CORDIC_NBREAD_1,               /* angle to read */
			 LL_CORDIC_INSIZE_32BITS,          /* q1.31 format for input data */
			 LL_CORDIC_OUTSIZE_32BITS);

	spin.version.setBoardVersion(TWIST_v_1_1_4);
	twist.setVersion(shield_TWIST_V1_4); // Code only compatible for TWIST V1.2 for V1.3 change
					    // the version and enable the NGND
	twist.initAllBuck(); // We need it to start HRTIM clock for control task interruption
	timing_init();
	timing_start();
	// spin.gpio.configurePin(PC8, OUTPUT);
	// spin.gpio.configurePin(PC7, OUTPUT);
	//------ software init -----
	communication.sync.initMaster(); // start the synchronisation

	// data.enableTwistDefaultChannels();

	spin.adc.configureTriggerSource(2, hrtim_ev3);
	data.enableShieldChannel(2, ANALOG_SIN);
	data.enableShieldChannel(2, ANALOG_COS);

    // Peripheral clock enable
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
    LL_TIM_SetCounterMode(TIM6, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetPrescaler(TIM6, (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC/1e7) - 1);
    LL_TIM_SetOnePulseMode(TIM6, LL_TIM_ONEPULSEMODE_SINGLE);
    // LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
    LL_TIM_DisableMasterSlaveMode(TIM6);
    LL_TIM_SetUpdateSource(TIM6, LL_TIM_UPDATESOURCE_REGULAR);
	/* 50us */
	LL_TIM_SetAutoReload(TIM6, 55*10);
    LL_TIM_EnableIT_UPDATE(TIM6);



 //    IRQ_DIRECT_CONNECT(TIM6_DAC_IRQn, 0, launch_transmission, IRQ_ZERO_LATENCY);
	// irq_enable(TIM6_DAC_IRQn);
	AppTask_num = task.createBackground(loop_application_task);
	task.startBackground(AppTask_num);

	CommTask_num = task.createBackground(loop_communication_task);
	task.startBackground(CommTask_num);

	// TODO: make watchdog in each slave (if no data received between two count: disable pwm)
	communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
				      reception_function,
				      SPEED_20M); // custom configuration for RS485
	task.createCritical(&loop_control_task, control_task_period);
	task.startCritical();


}
//--------------LOOP FUNCTIONS--------------------------------
void loop_communication_task()
{
	received_serial_char = console_getchar();
	switch (received_serial_char) {
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
		f0_ref = 0.0;
		break;
	case 'p':
		mode_asked = POWERMODE;
		// printk("power\n");
		break;
	case 'u':
		if (Iq_ref < CURRENT_LIMIT) {
			Iq_ref += 0.1;
		}
		break;
	case 'd':
		if (Iq_ref > 0.0) {
			Iq_ref -= 0.1;
		}
		break;
	case 'j':
		if (f0_ref < 400.0) {
			f0_ref += .1;
		}
		break;
	case 'k':
		if (f0_ref > 0.0) {
			f0_ref -= .1;
		}
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
void loop_application_task()
{
	static uint32_t counter_app;
	counter_app++;
	int id = sync_problem;
	printk("\033[2J");
	printk("\033[H");
	printk("ctrl state   = %d\n", control_state);
	printk("3? 		     = %d\n", last_receive_calls);
	printk("Iq_ref 	     = %6.2f [A]\n", (double)Iq_ref);
	printk("    f0       = %6.2f [Hz]\n", (double)f0_ref);
	printk(" w_sin_cos   = %6.2f [rad/s]\n", (double)w_meas);
	printk(" f_sin_cos   = %6.2f [Hz]\n", (double)f_meas);
	printk("Vh a 	     = %.2f [V]\n", (double) from_12bits(data_master_a.VH, 100.0));
	printk("Vh b  	     = %.2f [V]\n", (double) from_12bits(data_master_b.VH, 100.0));
	printk("Vh c 	     = %.2f [V]\n", (double) from_12bits(data_master_c.VH, 100.0));
	printk("stop_rec     = %d\n", stop_recording);
	printk("scope st     = %d\n", scope.has_trigged());
	printk("comm_pb_a    = %d\n", data_master_a.V2);
	printk("comm_pb_b    = %d\n", data_master_b.V2);
	printk("comm_pb_c    = %d\n", data_master_c.V2);
	printk("n_frame_pb_a = %d\n", data_master_a.V1);
	printk("n_frame_pb_b = %d\n", data_master_b.V1);
	printk("n_frame_pb_c = %d\n", data_master_c.V1);
	printk("status_a     = %d\n", data_master_a.status);
	printk("status_b     = %d\n", data_master_b.status);
	printk("status_c     = %d\n", data_master_c.status);
	printk("Id          = %f\n", Id_filtered);
	printk("Iq          = %f\n", Iq_filtered);

	printk("\n");
	if (is_downloading && control_state != POWER) {
		dump_scope_datas(scope);
		is_downloading = 0;
	}

	switch (control_state) {
	case IDLE:
		spin.led.turnOff();
		if (mode_asked == POWERMODE) {
			startup_time = 0.0F;
			control_state = STARTUP;
		}
		break;
	case STARTUP:
		if (startup_time >= startup_end_time) {
			control_state = POWER;
		}
		break;
	case POWER:
		spin.led.turnOn();
		if (mode_asked == IDLEMODE) {
			control_state = IDLE;
		}
		break;
	case ERROR_STATE:
		spin.led.toggle();
		if (mode_asked == IDLEMODE) 
		{
			// task.stopCritical();
			// task.startCritical();
			control_state = IDLE;
		}
		break;
	default:
		break;
	}
	// Pause between two runs of the task
	task.suspendBackgroundMs(500);
}
/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be
 * interruped. It is from it that you will control your power flow.
 */
__attribute__((section(".ramfunc"))) void loop_control_task()
{
	float32_t ratio;
	time_tic = timing_counter_get();
	/* to re-launch the timer 6 */
	
	// spin.gpio.resetPin(PC8);
	counter_time++;
	// spin.gpio.setPin(PC8);

	get_angle_and_pulsation_sin_cos();
	// get_position_and_speed();
	// w_meas = 0;
	
	get_rs485_datas();
	
	switch (control_state) {
	case IDLE:
	case ERROR_STATE:
		pi_d.reset(0.0F);
		pi_q.reset(0.0F);
		pllAngle.reset(0.0F);
		data_bridge.abx_cmd = STOP;
		break;
	case STARTUP:
		startup_time++;
		ratio = startup_time / startup_end_time;
		if (ratio > 1.0F) ratio = 1.0F;
		Vabc.a = ratio * Voffset;
		Vabc.b = ratio * Voffset;
		Vabc.c = ratio * Voffset;
		/* for recording */
		Iabc.a = I1_low_value_from_slave_a;
		Iabc.b = I1_low_value_from_slave_b;
		Iabc.c = I1_low_value_from_slave_c;
		data_bridge.abx_cmd = START;
		break;
	case POWER:

		Idq_ref.q = Iq_ref;
		Idq_ref.d = 0.0;
		Iabc_ref = transform.to_threephase(Idq_ref, angle_filtered); // used for debugging

		Iabc.a = I1_low_value_from_slave_a;
		Iabc.c = I1_low_value_from_slave_c;
		Iabc.b = - (I1_low_value_from_slave_a + I1_low_value_from_slave_c);

		Idq = transform.to_dqo(Iabc, angle_filtered); //used for control
		Id_filtered = Id_filter.calculateWithReturn(Idq.d);
		Iq_filtered = Iq_filter.calculateWithReturn(Idq.q);

		Vdq.d = pi_d.calculateWithReturn(Idq_ref.d, Idq.d);
		Vdq.q = pi_q.calculateWithReturn(Idq_ref.q, Idq.q);

		Idq_module = ot_sqrt(Idq.d * Idq.d + Idq.q * Idq.q);
		Vdq_module = ot_sqrt(Vdq.d * Vdq.d + Vdq.q * Vdq.q);
		// REMOVE CURRENT CONTROL
		// Vdq.d = 0.0;
		// Vdq.q = Iq_ref * 0.2;
		Vabc = transform.to_threephase(Vdq, angle_filtered);
		Vabc.a += Voffset;
		Vabc.b += Voffset;
		Vabc.c += Voffset;
		data_bridge.abx_cmd = START;
		break;
	}
	define_tx_datas();
	if (counter_time % decimation == 0)
	{
		scope.acquire();
	}
	if (counter_time > 100) { // Wait to be sure that other cards are power on.
		cumul_receiv_datas += n_receive_calls;
		last_receive_calls = n_receive_calls;
		if (n_receive_calls != 3)
		{
			control_state = ERROR_STATE;
			sync_problem = true;
		}
		communication.rs485.startTransmission();
		n_receive_calls = 0;
	}
	time_toc = timing_counter_get();
	total_cycles = timing_cycles_get(&time_tic, &time_toc);
	total_ns = timing_cycles_to_ns(total_cycles);
	// spin.gpio.resetPin(PC8);
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
