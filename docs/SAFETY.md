# Safety checklist

SHARKY is an experimental moving machine. Treat unexpected motion as possible
whenever the servo rail is energised.

Before the first powered test:

- inspect the frame for cracks, loose fasteners, trapped wires, and binding
  joints;
- confirm supply voltage, polarity, fuse, wire gauge, and common ground;
- verify each servo horn at its known neutral before attaching its linkage;
- compare the physical channel map and joint directions with the controller;
- keep the robot raised, restrained, and clear of faces, fingers, cables, and
  table edges;
- provide a physical way to remove servo power immediately;
- start with the controller disarmed and test one joint at a time;
- never charge or test an improvised lithium battery setup unattended.

The software includes bounds, an explicit arm state, output disable support, and
a command watchdog. These are layers of protection, not a substitute for a
physical cut-off or correct hardware.

## What activates motion?

Cloning, viewing media, installing dependencies, and compiling the packages do
not move SHARKY. The full launch file starts the device and motion nodes, but the
gait controller remains disarmed. PWM outputs are enabled only after an explicit
arm action; arming also commands the stand pose.

If you do not intend to test hardware, leave servo power disconnected and do
not launch `sharky_full.launch.py`.
