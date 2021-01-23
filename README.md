# Inverted Pendulum

## Simulation of inverted pendulum with PID controller or fuzzy controller

![pendulum_start](./pendulum/pics/Pendulum.gif)

intsall:
- pygame
- box2d
- box2d-py
- scikit-fuzzy

two different control systems included:
- run Pendulum_PID.py for PID control loop
- run Pendulum_Fuzzy.py for fuzzy control loop 

keyboard commands:
- n: create new world
- m: manual mode
- a: automatic mode

___________________________________________________________________________________________
### PID control loop

PID control loop uses PID library from [PyConSys](https://github.com/mck-sbs/PyConSys)
___________________________________________________________________________________________
### fuzzy control loop
fuzzy control loop uses scikit-fuzzy

Input fuzzy set:
![input_set](./pendulum/pics/input_set.png)

Output fuuzy set:
![input_set](./pendulum/pics/output_set.png)

Simple inference:
```
self.rule1 = ctrl.Rule(self.fuzz_pend1['dismal'], self.fuzz_motor['dismal'])
self.rule2 = ctrl.Rule(self.fuzz_pend1['poor'], self.fuzz_motor['poor'])
self.rule3 = ctrl.Rule(self.fuzz_pend1['mediocre'], self.fuzz_motor['mediocre'])
self.rule4 = ctrl.Rule(self.fuzz_pend1['average'], self.fuzz_motor['average'])
self.rule5 = ctrl.Rule(self.fuzz_pend1['decent'], self.fuzz_motor['decent'])
self.rule6 = ctrl.Rule(self.fuzz_pend1['good'], self.fuzz_motor['good'])
self.rule7 = ctrl.Rule(self.fuzz_pend1['excellent'], self.fuzz_motor['excellent'])
self.pendulum_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3, self.rule4, self.rule5, self.rule6, self.rule7])
```


Defuzzification:
![input_set](./pendulum/pics/defuzzification.png)
___________________________________________________________________________________________


From the course "Mechatronische Systeme (Mechatronic Systems)", technical college SBS Herzogenaurach-Höchstadt.

___________________________________________________________________________________________

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

(c) 2020, Metin Karatas (m.karatas@sbs-herzogenaurach.de)

