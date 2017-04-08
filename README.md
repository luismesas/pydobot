[![CircleCI](https://circleci.com/gh/luismesas/pydobot.svg?style=svg)](https://circleci.com/gh/luismesas/pydobot)

Python library for Dobot Magician
===

Based on Communication Protocol V1.0.4 (_latest version [here](http://dobot.cc/download-center/dobot-magician.html)_)


Installation
---

```
pip install pydobot
```

Samples
---

```python
import time
from glob import glob

from pydobot import Dobot

available_ports = glob('/dev/cu*usb*')  # mask for OSX Dobot port
if len(available_ports) == 0:
    print('no port found for Dobot Magician')
    exit(1)

device = Dobot(port=available_ports[0])

time.sleep(0.5)
device.speed(100)
device.go(250.0, 0.0, 25.0)
device.speed(10)
device.go(250.0, 0.0, 0.0)
time.sleep(2)
device.close()

```