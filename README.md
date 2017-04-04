Python 3 library for Dobot Magician
===

Based on Communication Protocol V1.0.4 (_latest version [here](http://dobot.cc/download-center/dobot-magician.html)_)


Samples
---
OSX

```python
from dobot import DobotMagician
import time

arm = DobotMagician(port='/dev/cu.wchusbserial1410') # OSX
arm.start()

time.sleep(1)
arm.move(250.0, 0.0, -25.0)
time.sleep(2)
arm.move(250.0, 0.0, 25.0)
time.sleep(1)
arm.close()

```