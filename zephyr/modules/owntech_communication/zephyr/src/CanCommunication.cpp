/*
 * Copyright (c) 2022-2024 LAAS-CNRS
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
 * @date   2024
 *
 * @author Clément Foucher <clement.foucher@laas.fr>
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 */


/* Header */
#include "CanCommunication.h"
#include "data_objects.h"

/* Zephyr driver*/
#include <zephyr/drivers/gpio.h>

/////
// Extern variable defined in this module

extern uint16_t broadcast_time;
extern uint16_t control_time;


uint16_t CanCommunication::getCanNodeAddr()
{
    return can_node_addr;
}

bool CanCommunication::getCtrlEnable()
{
    return ctrl_enable;
}

float32_t CanCommunication::getCtrlReference()
{
    return reference_value;
}

uint16_t CanCommunication::getBroadcastPeriod()
{
    return broadcast_time;
}

uint16_t CanCommunication::getControlPeriod()
{
    return control_time;
}


void CanCommunication::setCanNodeAddr(uint16_t addr)
{
    can_node_addr = addr;
}

void CanCommunication::setCtrlEnable(bool enable)
{
    ctrl_enable = enable;
}

void CanCommunication::setCtrlReference(float32_t reference)
{
    reference_value = reference;
}

void CanCommunication::setBroadcastPeriod(uint16_t time_100_ms)
{
    broadcast_time = time_100_ms;
}

void CanCommunication::setControlPeriod(uint16_t time_100_ms)
{
    control_time = time_100_ms;
}

void CanCommunication::enableCan()
{
	const struct gpio_dt_spec can_standby_spec = GPIO_DT_SPEC_GET(CAN_STANDBY_DEVICE, gpios);
	gpio_pin_configure_dt(&can_standby_spec, GPIO_OUTPUT_INACTIVE);
}