Import("env")
import os
import shutil


def after_upload(source, target, env):


    serial_phase = os.environ['SERIAL']

    new_elf_path = f"./ELF_FILE/{serial_phase}"
    old_elf_path = f"{source[0]}"
    print(f"\033[1;95m ---------- MOVING ELF File : {new_elf_path} -------")

    if os.path.exists(new_elf_path) :
        shutil.copy(old_elf_path, new_elf_path)
    else :

        os.makedirs(new_elf_path)
        shutil.copy(old_elf_path, new_elf_path)

env.AddPostAction("upload", after_upload)
