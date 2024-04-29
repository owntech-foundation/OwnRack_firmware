import os
import subprocess
Import("env")

# Each phase represent a board, asociated to a stlink ID. To know the stlink ID use `pyocd list` command line.
# N3: 0011001B5553501020393256
# sans etiquette: 000A0010544B500220343637
# N6: 003500105553501020393256 
# N4: 000D00304741500520383733

import json
with open("serial_mapping.json", "r") as f:
    serial_mapping = json.load(f)

stlinks_phase = {**serial_mapping}

def check_conection(phase):

    # define the target stlink ID
    target_stlink_id = stlinks_phase[phase]

    # Execute `pyocd list` to check all the connected probes
    result = subprocess.run(['pyocd', 'list'], capture_output=True, text=True)
    output = result.stdout

    # Check if target ID is in the output, and flash the code
    if target_stlink_id in output:
        print("\033[1;95m for ", phase, ", the target stlink ID was found")
        os.environ['target'] = phase
        os.system("pio run -t upload")
    else :
        print("\033[1;95m for ", phase,"the target stlink was not found")

target_names = [f'phase_{k}' for k in ['a', 'b', 'c']]

# Custom target definition, check https://docs.platformio.org/en/latest/scripting/custom_targets.html
for t in target_names:
    env.AddCustomTarget(
            name=t,
            dependencies=None,
            actions= [check_conection(t)],
            title=f"\033[1;95m compile {t}",
            description=f"check if the target stlink was found for {t} and flash the code.",
            always_build=True,
            )


