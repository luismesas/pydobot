[![CircleCI](https://circleci.com/gh/luismesas/pydobot.svg?style=svg)](https://circleci.com/gh/luismesas/pydobot)

Python library for Dobot Magician
===

Based on Communication Protocol V1.1.4 (_latest version [here](https://www.dobot.cc/downloadcenter.html?sub_cat=72#sub-download)_)


Installation
---

```
pip install pydobot
```

Samples
---

```python
from serial.tools import list_ports

from pydobot import Dobot

port = list_ports.comports()[0].device
device = Dobot(port=port, verbose=True)

(x, y, z, r, j1, j2, j3, j4) = device.pose()
device.go(x + 100, y, z, r)
device.go(x, y, z, r)

device.close()
```