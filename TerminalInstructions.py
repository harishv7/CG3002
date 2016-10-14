##Terminal instructions

import os

os.system("sudo systemctl stop serial-getty@ttyAMA0.service")
os.system("sudo chmod 777 /dev/ttyAMA0")
