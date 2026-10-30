"""Run with sudo on Nano to restore and persist its CH340 UART access for liwei."""
from pathlib import Path
import subprocess

rule=Path('/etc/udev/rules.d/70-robot-fpga-uart.rules')
content=('SUBSYSTEM=="tty", KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", '
         'ATTRS{idProduct}=="7523", RUN+="/usr/bin/setfacl -m u:liwei:rw /dev/%k"\n')
if rule.exists() and rule.read_text()!=content:
    raise RuntimeError('Existing UART rule differs; refusing overwrite')
rule.write_text(content)
subprocess.check_call(['udevadm','control','--reload-rules'])
port=Path('/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
assert port.resolve().name.startswith('ttyUSB') and port.exists()
subprocess.check_call(['/usr/bin/setfacl','-m','u:liwei:rw',str(port.resolve())])
print('CH340 UART permission restored; matching replug rule installed. No ARM or DRIVE.')
