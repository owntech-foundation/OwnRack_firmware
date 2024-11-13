# BRIDGE flashing script

This repo is for uploading the code in the bridge. It initialize the exchange between the slave and sends the reference needed.

You need to use the `SPIN` environment in PLATFORMio.

## Comments

This following syntax allows the creation of bit structures.

```c
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

```

it means the variables `uintXX_t Name:#` has a certain uint which only has the number `#` of bits. The `__packed` PRAGMA says that this entire structure is one single slot, with no spaces between variables. 


The code has been updated to work with both USB upload and STLink communication. 

In `pio_extra.ini` you will find the following code that greps the vid and pid of the stlink

```ini
    # Find console automatically
    ; monitor_port = hwgrep://2fe3:0100
    monitor_port = hwgrep://0483:374f

```
## HAL sensors integration

The code for HALL sensors from the ownverter FOC example was integrated into the main.
