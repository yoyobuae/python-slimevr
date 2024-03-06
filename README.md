# Python SlimeVR

Simple library for sending tracker data to SlimeVR server. Implementation is modelled after the SlimeVR firmware source code.

Example usage:
```
import time
from slimevr.connection import Connection
from slimevr.constants import Quaternion
from slimevr.constants import DATA_TYPE_NORMAL

angle = 0.0
while True:
    time.sleep(0.01)
    connection.update()
    connection.sendRotationData(0, Quaternion.fromAxisAngle(1.0, 1.0, 1.0, angle), DATA_TYPE_NORMAL, 0)
    angle += 0.5
```
This will connect to a local SlimeVR server running on the same network, and then send rotation data for a single tracker at around 100 Hz.
