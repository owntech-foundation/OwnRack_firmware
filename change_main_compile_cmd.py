import os
Import("env")
# TODO : put serial_mapping in json file.
import json
with open("serial_mapping.json", "r") as f:
    serial_mapping = json.load(f)

# Macro to define for each target for the target phase_a -> macro to define: SLAVE_A 
spin_mode = {'phase_a': 'SLAVE_A',
             'phase_b': 'SLAVE_B',
             'phase_c': 'SLAVE_C'
             }

def extra_config_for_main(node):
    if "main" in node.name:
        print("deleting main")
        main_obj = '.pio/build/spin/src/main.o'
        if os.path.exists(main_obj):
            os.remove(main_obj)  # to force recompile the main file.
        if 'target' in os.environ:
            target = os.environ['target']
            print("\033[1;95m target en cours:", target)
            env['CPPDEFINES'].append(f'SPIN_MODE={spin_mode[target]}')
            os.environ['SERIAL'] = serial_mapping[target]
        return env.Object(
            node,
            CPPDEFINES=env["CPPDEFINES"])
    else:
        return node

env.AddBuildMiddleware(extra_config_for_main)

