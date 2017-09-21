ESP32 Busy Board
====================

flash:
```
python D:\works\esp32\code\esp-idf\components\esptool_py\esptool\esptool.py --chip esp32 --port COM7 --baud 115200 --before default_reset --after hard_reset write_flash -u --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 D:\works\esp32\code\esp32-busyboard\build\bootloader\bootloader.bin 0x10000 D:\works\esp32\code\esp32-busyboard\build\busyboard.bin 0x8000 D:\works\esp32\code\esp32-busyboard\build\partitions_singleapp.bin
```
