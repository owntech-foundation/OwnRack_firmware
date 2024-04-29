""" get_recording """
import sys
import json
import time
import struct
import argparse
import numpy as np
import pandas as pd
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from pyocd.debug.elf.symbols import ELFSymbolProvider
RECORD_SIZE = 2048
CURVES = [
        {'name': 'I1_low', 'format': 'f',},
        {'name': 'V_low', 'format': 'f',},
        {'name': 'Vhigh_value', 'format': 'f',},
        {'name': 'I2_low', 'format': 'f',},
        {'name': 'I_com', 'format': 'f',},
        {'name': 'duty_cycle', 'format': 'f',},
        {'name': 'Vref', 'format': 'f',},
        {'name': 'n_receive_calls', 'format': 'f',},
        {'name': 'w', 'format': 'f',},
        ]


# N3: 0011001B5553501020393256
# sans etiquette: 000A0010544B500220343637
# N6: 003500105553501020393256 
# N4: 000D00304741500520383733
with open("serial_mapping.json", "r") as f:
    serial_mapping = json.load(f)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(prog="get_recording", 
                                     description="get record_array variable with pyocd",)

    parser.add_argument('cards', choices=['all_phases', 'phase_a', 'phase_b',
                                          'phase_c'], default='all_phases')
    arg_result = parser.parse_args(sys.argv[1:])
    options = {'connect_mode': 'attach', 'target_override': 'stm32g474retx'}

    if arg_result.cards == "all_phases":
        phases = ["phase_a", "phase_b", "phase_c"]
    else:
        phases = [arg_result.cards]

    for phase in phases:
        probe_id = serial_mapping[phase]
        print(phase, probe_id)
        session = ConnectHelper.session_with_chosen_probe(unique_id=probe_id, options=options)
        session.open()
        session.target.elf = f"ELF_FILE/{session.probe.unique_id}/firmware.elf"
        provider = ELFSymbolProvider(session.target.elf)
        addr = provider.get_symbol_value("record_array")
        if addr is None :
            print("problem address")

        print(f"record addr = {addr:x}")
        tic = time.time()
        datas = session.target.read_memory_block32(addr, len(CURVES)*RECORD_SIZE)
        toc = time.time()
        print(f"time : {toc-tic}")
        datas = np.reshape(datas, (-1, len(CURVES)))
        results = {}
        for k, curve in enumerate(CURVES):
            results[curve['name']] = [struct.unpack(curve['format'], struct.pack('I', d))[0] for d in datas[:, k]]

        df = pd.DataFrame(results)
        df.to_csv(f'last_results_{phase}.csv')
        session.close()
