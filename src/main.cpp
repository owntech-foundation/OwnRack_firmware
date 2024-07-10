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
#include "pr.h"
#include "filters.h"
#include "transform.h"
#include "ScopeMimicry.h"
//------------ZEPHYR DRIVERS----------------------
#include "zephyr/console/console.h"
#include <zephyr/timing/timing.h>
#include <string.h>
//---------- LL drivers --------------------------
#include "stm32_ll_gpio.h"
#include "stm32_ll_lpuart.h"

/* 3 byte for Va/Vb, 
 * 3 byte for Vc/(Ia/Ic), 
 * 3 byte for Ib and w, 
 * 1 byte for identifiant */
#define DMA_BUFFER_SIZE 10

#define UID_MASTER_A 0x3A004D
#define UID_SLAVE_A1 0x350030
#define UID_MASTER_B 0x58002F
#define UID_SLAVE_B1 0x340051
#define UID_MASTER_C 0x400040
#define UID_SLAVE_C1 0x3A004F

enum {
	MASTER,
	SLAVE
} control_mode;

enum rs_id {
	BRIDGE = 1,
	MASTER_A = 2,
	SLAVE_A1 = 3,
	SLAVE_A2 = 4,
	MASTER_B = 5,
	SLAVE_B1 = 6,
	SLAVE_B2 = 7,
	MASTER_C = 8,
	SLAVE_C1 = 9,
	SLAVE_C2 = 10
};
enum rs_id spin_mode;
static bool is_master_phase;

char TXT_ID[11][10] = {"_", "Bridge", "MASTER_A", "SLAVE_A1", "SLAVE_A2", "MASTER_B", "SLAVE_B1", "SLAVE_B2", "MASTER_C", "SLAVE_C1", "SLAVE_C2"};
#define CURRENT_LIMIT         9.0F
#define VOLTAGE_SCALE         50.0F
// retrieve identifiant
#define GET_ID(buffer_rx) ((buffer_rx[9] >> 4) & 0xf)

__inline__ int16_t to_12bits(float32_t data, float scale, float32_t offset = 0.0)
{
	return (int16_t)((data + offset) * 4095.0F / (scale));
}

float32_t from_12bits(int16_t data, float32_t scale, float offset = 0.0)
{
	return ((scale * (float32_t)data) / 4095.0F) - offset;
}
// register address to get the processor unique id 96 bits data
unsigned int *uid_0 = (unsigned int *)0x1FFF7590;
unsigned int *uid_1 = (unsigned int *)0x1FFF7594;
unsigned int *uid_2 = (unsigned int *)0x1FFF7598;

//--------------SETUP FUNCTIONS DECLARATION-------------------
void setup_routine(); // Setups the hardware and software of the system
//-------------LOOP FUNCTIONS DECLARATION----------------------
void loop_communication_task(); // code to be executed in the slow communication task
int8_t CommTask_num;            // Communication Task number
void loop_control_task();     // code to be executed in real-time at 20kHz
void loop_application_task(); // code to be executed in the fast application task
int8_t AppTask_num;           // Application Task number

//
uint32_t led_refresh_sampling = 6 * 10000;
uint32_t application_task_counter;
//------------- PR RESONANT -------------------------------------
static float32_t w0;
static LowPassFirstOrderFilter vHigh_filter(100e-6, 1e-3);
static float32_t vHigh_filtered;
//--------------USER VARIABLES DECLARATIONS----------------------

static uint32_t control_task_period = 100; //[us] period of the control task
static float32_t Ts = (float32_t)control_task_period * 1.0e-6F;
uint8_t received_serial_char;

/* Measure variables */
static float32_t V2_low_value;
static float32_t V1_low_value;
static float32_t I1_low_value = 0.0;
static float32_t I2_low_value = 0.0;
static float32_t I1_offset;
static float32_t I2_offset;
static float32_t V_high;
static float32_t I_high;
static float meas_data; // temp storage meas value (ctrl task)
static uint16_t I1_low_for_rs;
static float32_t Vref = 0.0; // voltage reference from master
static float32_t Iref = 0.0; // voltage reference from master
static float32_t duty_cycle_leg1 = 0.2;
static float32_t duty_cycle_leg2 = 0.2;
/* Sinewave settings */
float f0 = 10.0F;
float angle;
PrParams pr_params = PrParams(Ts, 0.005, 2000.0, 0.0, 0.0, -48.0, 48.0);
Pr prop_res_leg1 = Pr();
Pr prop_res_leg2 = Pr();
ScopeMimicry scope(512, 7);
static uint32_t decimation = 1;
bool is_downloading;
typedef struct {
	uint16_t Va:12;
	uint16_t Vb:12;
	uint16_t Vc:12;
	uint16_t w:12;
	uint8_t stop_recording:1;
	uint16_t no_data1:11;
	uint16_t no_data2:12;
	uint8_t abx_cmd:4;
	uint8_t id:4;
} __packed bridge_frame_t;
bridge_frame_t data_bridge;

typedef struct {
	uint16_t V1:12;
	uint16_t V2:12;
	uint16_t I1:12;
	uint16_t I2:12;
	uint16_t VH:12;
	uint16_t IH:12;
	uint8_t status:4;
	uint8_t id:4;
} __packed phase_frame_t;

phase_frame_t data_master_a;
phase_frame_t data_master_b;
phase_frame_t data_master_c;
phase_frame_t data_slave;
phase_frame_t datas_to_send;

uint8_t buffer_tx[10];
uint8_t buffer_rx[10];

uint32_t n_frame_received = 3;
uint32_t n_frame_received_pb = 0;
static float32_t comm_problem = 0.0F;
uint8_t stop_recording = 0;
uint32_t counter_time = 0;
uint32_t counter_led = 0;

three_phase_t Iabc_ref;
float32_t comp_dt = 0.010;
uint32_t counter;

//---------------------------------------------------------------

enum control_state_mode //
{
	INIT = 0,
	OFFSET = 1,
	IDLE = 2,
	POWER = 3,
	ERROR_STATE = 4

};
enum error_codes {
	NOERR = 0,
	OVERCURRENT = 1,
	COMM_PB_IN_IDLE = 2,
	COMM_PB_IN_POWER = 3,
} ;
enum error_codes error_code = NOERR;
enum control_state_mode control_state = IDLE;
const char *state_msg[] = {"IDLE", "POWER", "ERROR_STATE", NULL};

const uint32_t OFFSET_COUNT = 500;

enum autobox_msg {
	STOP = 0, 
	START = 1
};
static enum autobox_msg abx_cmd;
timing_t time_toc;
timing_t time_tic;
uint64_t total_cycles;
uint64_t total_ns;

bool mytrigger() {
	return true;
	// return data_bridge.stop_recording;
}
uint32_t spin_id(void)
{
	return *uid_0;
}
void reception_function(void)
{
	uint8_t rs_id;
	spin.gpio.setPin(PC7);
	rs_id = GET_ID(buffer_rx);
	if (rs_id == BRIDGE)
	{
		data_bridge = *(bridge_frame_t *) buffer_rx;
		/*reception of data*/
		if (spin_mode == MASTER_A) 
		{
			memcpy(buffer_tx, &datas_to_send, sizeof(datas_to_send));
			communication.rs485.startTransmission();
			data_slave = datas_to_send;
		}
	}
	else
	{
		switch (spin_mode) {
			case SLAVE_A1:
			case SLAVE_A2:
				if (rs_id == MASTER_A) {
					data_slave = *(phase_frame_t *) buffer_rx;
				}
			break;
			case MASTER_B:
				if (rs_id == MASTER_A) {
					memcpy(buffer_tx, &datas_to_send, sizeof(datas_to_send));
					communication.rs485.startTransmission();
					data_slave = datas_to_send;
				}
			break;
			case SLAVE_B1:
			case SLAVE_B2:
				if (rs_id == MASTER_B) {
					data_slave = *(phase_frame_t *) buffer_rx;
				}
			break;
			case MASTER_C:
				if (rs_id == MASTER_B) {
					memcpy(buffer_tx, &datas_to_send, sizeof(datas_to_send));
					communication.rs485.startTransmission();
					data_slave = datas_to_send;
				}
			break;
			case SLAVE_C1:
			case SLAVE_C2:
				if (rs_id == MASTER_C) {
					data_slave = *(phase_frame_t *) buffer_rx;
				}
			break;
		}
	}
		
	n_frame_received++;
	spin.gpio.resetPin(PC7);
}
inline void get_measures()
{
	meas_data = data.getLatest(I1_LOW);
	if (meas_data != NO_VALUE) {
		I1_low_value = meas_data + I1_offset;
	}
	meas_data = data.getLatest(V1_LOW);
	if (meas_data != NO_VALUE) {
		V1_low_value = meas_data;
	}
	meas_data = data.getLatest(V2_LOW);
	if (meas_data != NO_VALUE) {
		V2_low_value = meas_data;
	}

	meas_data = data.getLatest(I2_LOW);
	if (meas_data != NO_VALUE) {
		I2_low_value = meas_data + I2_offset;
	}
	meas_data = data.getLatest(V_HIGH);
	if (meas_data != NO_VALUE) {
		V_high = meas_data;
	}
	meas_data = data.getLatest(I_HIGH);
	if (meas_data != NO_VALUE) {
		I_high = meas_data;
	}

	vHigh_filtered = vHigh_filter.calculateWithReturn(V_high);

	if (counter_time < OFFSET_COUNT)
	{
		I1_offset = -0.04F * (I1_low_value - I1_offset) + 0.96F * I1_offset;
		I2_offset = -0.04F * (I2_low_value - I2_offset) + 0.96F * I2_offset;
	}

}
inline void get_datas_from_rs485()
{
	if (counter_time > 150)
	{
		if ((n_frame_received != 3) && is_master_phase)
		{
			comm_problem++;
			n_frame_received_pb = n_frame_received;
		}
		else if (n_frame_received != 4 && !is_master_phase)
		{
			comm_problem++;
			n_frame_received_pb = n_frame_received;
		}
	/* reset n_frame_received */
		n_frame_received = 0;
	}
	abx_cmd = static_cast<autobox_msg>(data_bridge.abx_cmd);

	switch (spin_mode) {
		case MASTER_A:
		case SLAVE_A1:
		case SLAVE_A2:
			Vref = from_12bits(data_bridge.Va, VOLTAGE_SCALE);
			Iref = from_12bits(data_slave.I1, 50.0, 25.0);
		break;
		case MASTER_B:
		case SLAVE_B1:
		case SLAVE_B2:
			Vref = from_12bits(data_bridge.Vb, VOLTAGE_SCALE);
			Iref = from_12bits(data_slave.I1, 50.0, 25.0);
		break;
		case MASTER_C:
		case SLAVE_C1:
		case SLAVE_C2:
			Vref = from_12bits(data_bridge.Vc, VOLTAGE_SCALE);
			Iref = from_12bits(data_slave.I1, 50.0, 25.0);
		break;
	}
	w0 = from_12bits(data_bridge.w, 500.0, 0.0);
	/* if stop_recording == 0 we restart always the scope */
	if (data_bridge.stop_recording == 0)
	{
		scope.start();
	}
}
inline void compute_control_state()
{
	switch (control_state) {
		case INIT:
			if (V_high > 12.0F)
			{
				control_state = OFFSET;
			}
			break;
		case OFFSET:
			if (counter_time > OFFSET_COUNT)
			{
				control_state = IDLE;
			}
			break;
		case IDLE:
			led_refresh_sampling = 6 * 10000; // 60*100ms
			if (comm_problem > 10.0F && abx_cmd == STOP)
			{
				control_state = ERROR_STATE;
				if (error_code == NOERR)
				{
					error_code = COMM_PB_IN_IDLE;
				}
			} else if (abx_cmd == START) 
			{
				twist.startLeg(LEG1);
				twist.startLeg(LEG2);
				control_state = POWER;
			}
		break;
		case POWER:
			led_refresh_sampling = 1 * 10000; // 20*100ms
			if (abx_cmd == STOP) 
			{
				control_state = IDLE;
			}
			if (comm_problem > 10.0F)
			{
				control_state = ERROR_STATE;
				if (error_code == NOERR )
				{
					error_code = COMM_PB_IN_POWER;
				}
			}
		break;
		case ERROR_STATE:
			led_refresh_sampling = (int)(0.2 * 10000); // 4*100ms
			if (abx_cmd == STOP) {
				comm_problem = 0;
				control_state = IDLE;
			}
		break;
		default:
			control_state = IDLE;
	}
}
inline void manage_overcurrent() 
{
	if (((I1_low_value > CURRENT_LIMIT) || (I2_low_value > CURRENT_LIMIT) ||
	     (I1_low_value < -CURRENT_LIMIT) || (I2_low_value < -CURRENT_LIMIT)) &&
	    counter_time > 10 && control_state != OFFSET) {
		control_state = ERROR_STATE;
		if (error_code == NOERR)
		{
			error_code = OVERCURRENT;
		}
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
	//------- hardware init ----
	twist.disconnectAllCapacitor();
	switch (spin_id()) 
	{
		case UID_MASTER_A:
		case UID_MASTER_B:
		case UID_MASTER_C:
			spin.version.setBoardVersion(TWIST_v_1_1_2);
			twist.setVersion(shield_TWIST_V1_2);
			twist.initAllBuck(); // initialize in buck mode leg1 and leg2
			communication.sync.initSlave(TWIST_v_1_1_2);
			is_master_phase = true;
		break;
		default:
			spin.version.setBoardVersion(TWIST_v_1_1_4);
			twist.setVersion(shield_TWIST_V1_4);
			twist.initAllBuck(); // initialize in buck mode leg1 and leg2
			communication.sync.initSlave(TWIST_v_1_1_4);
			is_master_phase = false;
	}
	communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
				      reception_function, SPEED_20M); // RS485 at 20Mbits/s
	timing_init();
	timing_start();
	spin.gpio.configurePin(PC8, OUTPUT);
	spin.gpio.configurePin(PC7, OUTPUT);
	//------ software init -----
	scope.connectChannel(I1_low_value, "I1");
	scope.connectChannel(I2_low_value, "I2");
	scope.connectChannel(duty_cycle_leg1, "duty1");
	scope.connectChannel(duty_cycle_leg2, "duty2");
	scope.connectChannel(comm_problem, "comm_problem");
	scope.connectChannel(Vref, "Vref");
	scope.connectChannel(Iref, "Iref");
	scope.set_trigger(mytrigger);
	scope.set_delay(0.5);
	scope.start();
	data.enableTwistDefaultChannels();
	switch (spin_id()) {
		case UID_MASTER_A:
			spin_mode = MASTER_A;
			control_mode = MASTER;
			/* TWIST V1.2 not calibrated */
			data.setParameters(I1_LOW, 0.005, -10.0);
			data.setParameters(I2_LOW, 0.005, -10.0);
			data.setParameters(V_HIGH, 0.030, 0.0);
		break;
		case UID_SLAVE_A1:
			spin_mode = SLAVE_A1;
			control_mode = SLAVE;
			// not sure of calibration
			data.setParameters(I1_LOW, 0.005, -10.0);
			data.setParameters(I2_LOW, 0.005, -10.0);
			data.setParameters(V_HIGH, 0.030, 0.0);
		break;
		case UID_MASTER_B:
			spin_mode = MASTER_B;
			control_mode = MASTER;
			/* TWIST V1.2 not calibrated */
			data.setParameters(I1_LOW, 0.005, -10.0);
			data.setParameters(I2_LOW, 0.005, -10.0);
			data.setParameters(V_HIGH, 0.030, 0.0);
		break;
		case UID_SLAVE_B1:
			spin_mode = SLAVE_B1;
			control_mode = SLAVE;
			// not sure of calibration
			data.setParameters(I1_LOW, 0.005, -10.0);
			data.setParameters(I2_LOW, 0.005, -10.0);
			data.setParameters(V_HIGH, 0.030, 0.0);
		break;
		case UID_MASTER_C:
			spin_mode = MASTER_C;
			control_mode = MASTER;
			/* TWIST V1.2 not calibrated */
			data.setParameters(I1_LOW, 0.005, -10.0);
			data.setParameters(I2_LOW, 0.005, -10.0);
			data.setParameters(V_HIGH, 0.030, 0.0);
		break;
		case UID_SLAVE_C1:
			spin_mode = SLAVE_C1;
			control_mode = SLAVE;
			// not sure of calibration
			data.setParameters(I1_LOW, 0.005, -10.0);
			data.setParameters(I2_LOW, 0.005, -10.0);
			data.setParameters(V_HIGH, 0.030, 0.0);
		break;
	}
	// PR RESONANT
	prop_res_leg1.init(pr_params);
	prop_res_leg2.init(pr_params);
	n_frame_received = 3;
	comm_problem = 0.0F;
	I1_offset = 0.0;
	I2_offset = 0.0;
	
	/* Configure and Launch Tasks */
	CommTask_num = task.createBackground(loop_communication_task);
	task.startBackground(CommTask_num);
	AppTask_num = task.createBackground(loop_application_task);
	task.startBackground(AppTask_num);
	task.createCritical(&loop_control_task, control_task_period);
	task.startCritical();
}
//--------------LOOP FUNCTIONS--------------------------------
/**
 * This is the code loop of the background task
 * It is executed second as defined by it suspend task in its last line.
 * You can use it to execute slow code such as state-machines.
 */

void loop_communication_task() 
{
	received_serial_char = console_getchar();
	switch (received_serial_char) {
		case 'i':
			communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
								 reception_function, SPEED_20M); // RS485 at 20Mbits/s
			break;		
		case 'r':
			is_downloading = true;
		break;
	}
}

void loop_application_task()
{
	uint8_t *datas_to_send_pt;
	void *toto;
	toto = &datas_to_send;
	datas_to_send_pt = static_cast<uint8_t *>(toto);
	printk("\033[2J");
	printk("\033[H");
	printk("%s\n", TXT_ID[spin_mode]);
	printk("record = %d\n", data_bridge.stop_recording);
	printk("trigged %d\n", scope.has_trigged());
	printk("control %d\n", control_state);
	printk("comm_problem: %f\n", (double)comm_problem);
	printk("n_frame_received = %d\n", n_frame_received_pb);
	printk("I1_offset = %f\n", (double) I1_offset);
	printk("I2_offset = %f\n", (double) I2_offset);
	printk("I1_low_value = %f\n", (double) I1_low_value);
	printk("Vhigh = %f\n", (double) V_high);
	printk("is_master_phase = %d\n",  is_master_phase);
	printk("error_code = %d\n", error_code);
	printk("%d\n", counter_time);

	if (is_downloading) {
		scope.reset_dump();
		printk("begin record\n");
		while(scope.get_dump_state() != finished) {
			printk("%s", scope.dump_datas());
			task.suspendBackgroundUs(100);
		}
		printk("end record\n");
		is_downloading = false;
	}	
	task.suspendBackgroundMs(250);
}
/**
 * This is the code loop of the critical task
 * It is executed every 500 micro-seconds defined in the setup_software function.
 * You can use it to execute an ultra-fast code with the highest priority which cannot be
 * interruped. It is from it that you will control your power flow.
 */
__attribute__((section(".ramfunc"))) void loop_control_task()
{
	spin.gpio.setPin(PC8);
	time_tic = timing_counter_get();
	counter_time++;
	get_datas_from_rs485();
	get_measures();
	manage_overcurrent();
	compute_control_state();
	//--- CONTROL --------------------------------------------------------------
	if (control_state == POWER) {
		if (control_mode == MASTER) 
		{
			duty_cycle_leg1 = 1.0F / VOLTAGE_SCALE * Vref;
		}
		else
		{
			prop_res_leg1.setW0(w0);
			duty_cycle_leg1 =
				1.0F / VOLTAGE_SCALE *
				(Vref + prop_res_leg1.calculateWithReturn(Iref, I1_low_value));
		}
		prop_res_leg2.setW0(w0);
		duty_cycle_leg2 =
			1.0F / VOLTAGE_SCALE *
			(Vref + prop_res_leg2.calculateWithReturn(Iref, I2_low_value));
		twist.setLegDutyCycle(LEG1, duty_cycle_leg1);
		twist.setLegDutyCycle(LEG2, duty_cycle_leg2);
	} 
	else
	{
		duty_cycle_leg1 = 0.1;
		duty_cycle_leg2 = 0.1;
		twist.stopLeg(LEG1);
		twist.stopLeg(LEG2);
		Vref = 0.0;
	}

	datas_to_send.I1 = to_12bits(I1_low_value, 50.0, 25.0);
	datas_to_send.I2 = to_12bits(I2_low_value, 50.0, 25.0);
	datas_to_send.V1 = ((uint16_t) n_frame_received_pb) & 0x3ff;
	datas_to_send.V2 = ((uint16_t ) comm_problem) & 0x3ff;
	// datas_to_send.V2 = to_12bits(V2_low_value, 50.0, 25.0);
	datas_to_send.VH = to_12bits(V_high, 100.0);
	datas_to_send.IH = to_12bits(I_high, 50.0, 25.0);
	datas_to_send.id = spin_mode & 0xf;
	datas_to_send.status = control_state & 0xf;

	if (counter_time%decimation == 0) 
	{
		scope.acquire();
	}
	counter_led = (counter_led + 1) % led_refresh_sampling;
	if (counter_led > (led_refresh_sampling >> 1)) {
		spin.led.turnOn();
	} else {
		spin.led.turnOff();
	}
	time_toc = timing_counter_get();
	total_cycles = timing_cycles_get(&time_tic, &time_toc);
	total_ns = timing_cycles_to_ns(total_cycles);
	spin.gpio.resetPin(PC8);
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
