# Three phase control with three boards

**Warning** : This code as it is is compatible ONLY with twist V1.2, to support TWIST V1.3 you need to use function `power.setShieldVersion(shield_TWIST_V1_3)` and enable the NGND to get measures.


## Overview

![](Image/schema_twist.jpg)

### Requeirements

- Three TWIST (V1.2 recommended, you can use V1.3 but modifications are required in the code)
- A **64V** DC voltage source
- The output of the twist must be connected to a stator's inductors (you can use a BLDC motor)

## To flash three boards at once

Open the file `/platformio/packages/tool-openocd@2.1100.211028/scripts/interface/stlink.cfg` and change the content to :

```tcl
#
# STMicroelectronics ST-LINK/V1, ST-LINK/V2, ST-LINK/V2-1, STLINK-V3 in-circuit
# debugger/programmer
#

adapter driver hla
hla_layout stlink
if { [info exists ::env(SERIAL) ] } {
        puts "SERIAL found"
        puts "$::env(SERIAL)"
        hla_serial  $::env(SERIAL)
}
hla_device_desc "ST-LINK"
hla_vid_pid 0x0483 0x3744 0x0483 0x3748 0x0483 0x374b 0x0483 0x374d 0x0483 0x374e 0x0483 0x374f 0x0483 0x3752 0x0483 0x3753 0x0483 0x3754

# Optionally specify the serial number of ST-LINK/V2 usb device.  ST-LINK/V2
# devices seem to have serial numbers with unreadable characters.  ST-LINK/V2
# firmware version >= V2.J21.S4 recommended to avoid issues with adapter serial
# number reset issues.
# eg.
#hla_serial "\xaa\xbc\x6e\x06\x50\x75\xff\x55\x17\x42\x19\x3f"

```

:warning: If you can't find the folder `tool-openocd@2.1100.211028` select the folder `tool-openocd` instead.

Go to the folder `.platformio/python3` and open the command line interface and Execute the command `./python -m ensurepip` then `./python pip install pyocd`.

Two python scripts (change_main_compile_cmd.py and three_phase_targets.py) will automatically flash the code.

Here is a general schematic of how to use the control
![](Image/Schema.jpg)

We use three twist, each of them have a specific role and control a phase :

- phase A : The master, it sends reference to the slave
- phase B : A slave, it sends measure to the master
- phase C : A slave, it sends measure to the master

Each of the twist **MUST** be connected to a unic stlink, which is defined by its identifiant (ID). All the stlinks can be connected to the computer (with a USB HUB for example).

To know th ID of your stlink, execute `pyocd list` in  a command line interface. This identifiant is necessary to know wich board has which role, and what code to flash.

When you know the ID of the three stlink you are going to use, you are going to make some modifications on the following files :

- In `three_phase_target.py` :

```py
stlinks_phase = {
        'phase_a': 'ID_FOR_MASTER_STLINK',
        'phase_b': 'ID_FOR_SLAVE_B_STLINK',
        'phase_c': 'ID_FOR_SLAVE_C_STLINK',
        }
```
replace ID_FOR_XXXX_STLINK by the ID of the stlink for MASTER, SLAVE_B and SLAVE_C.

- In `change_main_compile_cmd.py` :

```py
serial_mapping = {'phase_a': 'ID_FOR_MASTER_STLINK',
                  'phase_b': 'ID_FOR_SLAVE_B_STLINK',
                  'phase_c': 'ID_FOR_SLAVE_C_STLINK'
                  }
```

Do the exact same thing in this file.

When you have done all these configurations, you can upload code on each TWIST with just one command. With platformIO, you'll have to be on the `threephase` environment. To flash the code, **Use only the build commande** (✔ symbol) :

![](Image/ThrePhase_environment.PNG)

PlatformIO will then check if the corresponding stlink is present, and flash the section of the code depending on the board's role.

## Using the three phase control

Before using the code check that the currents measures are calibrated. In `src/main.cpp` from line 271, we are deifining the gain and offset for each board :

```C
#if SPIN_MODE == SLAVE_B // calibration for N1
    dataAcquisition.setParameters(I1_LOW, 4.56, -9367.00);
    dataAcquisition.setParameters(I2_LOW, 5.51, -11351.00);
#endif

#if SPIN_MODE == MASTER // calibration for N0
    dataAcquisition.setParameters(I1_LOW, 5.406, -11426.00);
    dataAcquisition.setParameters(I2_LOW, 5.31, -10747.00);
#endif

#if SPIN_MODE == SLAVE_C // calibration for N4
    dataAcquisition.setParameters(I1_LOW, 4.679, -9551.00);
    dataAcquisition.setParameters(I2_LOW, 4.523, -9281.00);
#endif
```

You can modify these values to get the correct measure.

After uploadiing the code on each board, go to the `serial monitor` of the **MASTER** board. Here are the following commands to use :

- `i` : Idle mode. This will stop power flowing in the stator.
- `p` : Power mode. This will start supplying power to the stator.
- `u` : Up mode. This will increase the frequency of the three phase voltage supplied to the stator. If a motor is connected, it will speed up.
- `d`. This will decrease the frequency of the three phase voltage supplied to the stator. If a motor is connected, it will speed down.

## Communication protocol

Each of the three boards will communicate with each other, first the MASTER will send voltage reference to SLAVE_B and SLAVE_C. Then SLAVE_B and SLAVE_C will answer back to MASTER by sending back their current measures.

![](Image/protocol_communication.svg)

Each time the boards are sending **10 bytes** of data, at a speed of **10 Mbits/s**. Here are the datas send during the exchange :

- **Va_ref, Vb_ref, Vc_ref** : These are voltage reference that will be used to compute duty_cycle. duty_cycle = Vx_ref/V_high where V_high is the DC supply voltage which equald to **64V**
- **Ia, Ib and Ic** : These are the measured current from each board.
- **ω** : This is the angular frequency, which equals to 2*π*f with f the frequency of the alternate ouput voltage.

**status** is a 8 bits data used to transmit several information :

![](Image/status_information.svg)

- **ID** : it is a 2 bits data wich indicates who is transmiting data.
    - If ID = 1, MASTER is sending the data
    - If ID = 2, SLAVE_B is sending the data
    - If ID = 3, SLAVE_C is sending the data

- **OVERCURRENT** : For safety, if this bit is 1 it indicates if one of the board has experienced a high current (> 8A). If this is the case, the MASTER board will shutt everything off.
- **COUNTER MODE** : If this bit is high, it sets the variable `counter` to 0. It restart the buffer for monitoring the MCU intern variables.
- **IDLE MODE** :
    - If this bit is low, all the board are in IDLE MODE and no power is flowing.
    - If this bit is high, all the boards are in POWER MODE and power is flowing in the stator. Also the led is ON.

## Expected result

You have a python script `script.py` that allows you to watch the internal variable. See [this tutorial to understand how to use it](https://gitlab.laas.fr/afarahhass/Test-Controle/-/tree/main_RecordVariable).

Below you cans see what you can expect by watching the current value on each phase :

![](Image/Expected_result.PNG)

Modified :

2023.20.26 : [Ayoub Farah Hassan](mailto:ayoub.farah-hassan@laas.fr)