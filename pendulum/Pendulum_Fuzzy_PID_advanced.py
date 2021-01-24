# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2CircleShape)
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
from pyconsys.PIDControl import PIDControl

class BodyPendulum(Framework):
    name = "Inverted Pendulum"
    description = "(n) new world"

    def __init__(self):
        super(BodyPendulum, self).__init__()

        self.createWorld()
        self.createFuzzy()
        self._angle_old = 0
        self._pendulum_old_x = 0

    def createFuzzy(self):
        self.fuzz_pend1 = ctrl.Antecedent(np.arange(-1.5, 1.6, 0.1), 'join1')
        self.fuzz_pend1_v = ctrl.Antecedent(np.arange(-10, 10, 0.1), 'pend1_v')
        self.fuzz_motor = ctrl.Consequent(np.arange(-150, 150, 1), 'motor')

        self.fuzz_position = ctrl.Antecedent(np.arange(-13, 13, 0.1), 'position')
        self.fuzz_vv2 = ctrl.Antecedent(np.arange(-10, 10, 0.1), 'v')
        self.fuzz_motor2 = ctrl.Consequent(np.arange(-100, 100, 1), 'motor2')


        self.fuzz_pend1['poor'] = fuzz.zmf(self.fuzz_pend1.universe, -0.2, -0.1)
        self.fuzz_pend1['mediocre'] = fuzz.trimf(self.fuzz_pend1.universe, [-0.2, -0.1, 0])
        self.fuzz_pend1['average'] = fuzz.trimf(self.fuzz_pend1.universe, [-0.1, 0, 0.1])
        self.fuzz_pend1['decent'] = fuzz.trimf(self.fuzz_pend1.universe, [0, 0.1, 0.2])
        self.fuzz_pend1['good'] = fuzz.smf(self.fuzz_pend1.universe, 0.1, 0.2)

        self.fuzz_pend1_v['poor'] = fuzz.zmf(self.fuzz_pend1_v.universe, -2, -1)
        self.fuzz_pend1_v['mediocre'] = fuzz.trimf(self.fuzz_pend1_v.universe, [-2, -1, 0])
        self.fuzz_pend1_v['average'] = fuzz.trimf(self.fuzz_pend1_v.universe, [-1, 0, 1])
        self.fuzz_pend1_v['decent'] = fuzz.trimf(self.fuzz_pend1_v.universe, [0, 1, 2])
        self.fuzz_pend1_v['good'] = fuzz.smf(self.fuzz_pend1_v.universe, 1, 2)

        self.fuzz_motor['poor'] = fuzz.zmf(self.fuzz_motor.universe, -50, -20)
        self.fuzz_motor['average'] = fuzz.trimf(self.fuzz_motor.universe, [-50, 0, 50])
        self.fuzz_motor['good'] = fuzz.smf(self.fuzz_motor.universe, 20, 50)

        self.fuzz_position['poor'] = fuzz.zmf(self.fuzz_position.universe, -5, -2)
        self.fuzz_position['average'] = fuzz.trimf(self.fuzz_position.universe, [-5, 0, 5])
        self.fuzz_position['good'] = fuzz.smf(self.fuzz_position.universe, 2, 5)

        self.fuzz_motor2['poor'] = fuzz.zmf(self.fuzz_motor2.universe, -50, -20)
        self.fuzz_motor2['average'] = fuzz.trimf(self.fuzz_motor2.universe, [-50, 0, 50])
        self.fuzz_motor2['good'] = fuzz.smf(self.fuzz_motor2.universe, 20, 50)

        self.fuzz_vv2['poor'] = fuzz.zmf(self.fuzz_vv2.universe, -5, -2.5)
        self.fuzz_vv2['average'] = fuzz.trimf(self.fuzz_vv2.universe, [-5, 0, 5])
        self.fuzz_vv2['good'] = fuzz.smf(self.fuzz_vv2.universe, 2.5, 5)


        # self.fuzz_pend1['average'].view()
        # self.fuzz_pend1_v['average'].view()
        # self.fuzz_motor['average'].view()
        # self.fuzz_position['average'].view()
        # self.fuzz_vv2['average'].view()
        # self.fuzz_motor2['average'].view()

        self.rule1 = ctrl.Rule(self.fuzz_pend1['poor'] & self.fuzz_pend1_v['poor'] , self.fuzz_motor['poor'])
        self.rule2 = ctrl.Rule(self.fuzz_pend1['poor'] & self.fuzz_pend1_v['mediocre'], self.fuzz_motor['poor'])
        self.rule3 = ctrl.Rule(self.fuzz_pend1['poor'] & self.fuzz_pend1_v['average'], self.fuzz_motor['poor'])
        self.rule4 = ctrl.Rule(self.fuzz_pend1['poor'] & self.fuzz_pend1_v['decent'], self.fuzz_motor['poor'])
        self.rule5 = ctrl.Rule(self.fuzz_pend1['poor'] & self.fuzz_pend1_v['good'], self.fuzz_motor['poor'])

        self.rule6 = ctrl.Rule(self.fuzz_pend1['mediocre'] & self.fuzz_pend1_v['poor'] , self.fuzz_motor['poor'])
        self.rule7 = ctrl.Rule(self.fuzz_pend1['mediocre'] & self.fuzz_pend1_v['mediocre'], self.fuzz_motor['poor'])
        self.rule8 = ctrl.Rule(self.fuzz_pend1['mediocre'] & self.fuzz_pend1_v['average'], self.fuzz_motor['poor'])
        self.rule9 = ctrl.Rule(self.fuzz_pend1['mediocre'] & self.fuzz_pend1_v['decent'], self.fuzz_motor['average'])
        self.rule10 = ctrl.Rule(self.fuzz_pend1['mediocre'] & self.fuzz_pend1_v['good'], self.fuzz_motor['average'])

        self.rule11 = ctrl.Rule(self.fuzz_pend1['average'] & self.fuzz_pend1_v['poor'] , self.fuzz_motor['poor'])
        self.rule12 = ctrl.Rule(self.fuzz_pend1['average'] & self.fuzz_pend1_v['mediocre'], self.fuzz_motor['poor'])
        self.rule13 = ctrl.Rule(self.fuzz_pend1['average'] & self.fuzz_pend1_v['average'], self.fuzz_motor['average'])
        self.rule14 = ctrl.Rule(self.fuzz_pend1['average'] & self.fuzz_pend1_v['decent'], self.fuzz_motor['good'])
        self.rule15 = ctrl.Rule(self.fuzz_pend1['average'] & self.fuzz_pend1_v['good'], self.fuzz_motor['good'])

        self.rule16 = ctrl.Rule(self.fuzz_pend1['decent'] & self.fuzz_pend1_v['poor'] , self.fuzz_motor['average'])
        self.rule17 = ctrl.Rule(self.fuzz_pend1['decent'] & self.fuzz_pend1_v['mediocre'], self.fuzz_motor['average'])
        self.rule18 = ctrl.Rule(self.fuzz_pend1['decent'] & self.fuzz_pend1_v['average'], self.fuzz_motor['good'])
        self.rule19 = ctrl.Rule(self.fuzz_pend1['decent'] & self.fuzz_pend1_v['decent'], self.fuzz_motor['good'])
        self.rule20 = ctrl.Rule(self.fuzz_pend1['decent'] & self.fuzz_pend1_v['good'], self.fuzz_motor['good'])

        self.rule21 = ctrl.Rule(self.fuzz_pend1['good'] & self.fuzz_pend1_v['poor'] , self.fuzz_motor['good'])
        self.rule22 = ctrl.Rule(self.fuzz_pend1['good'] & self.fuzz_pend1_v['mediocre'], self.fuzz_motor['good'])
        self.rule23 = ctrl.Rule(self.fuzz_pend1['good'] & self.fuzz_pend1_v['average'], self.fuzz_motor['good'])
        self.rule24 = ctrl.Rule(self.fuzz_pend1['good'] & self.fuzz_pend1_v['decent'], self.fuzz_motor['good'])
        self.rule25 = ctrl.Rule(self.fuzz_pend1['good'] & self.fuzz_pend1_v['good'], self.fuzz_motor['good'])

        self.pendulum_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3, self.rule4, self.rule5,
                                                 self.rule6, self.rule7, self.rule8, self.rule9, self.rule10,
                                                 self.rule11, self.rule12, self.rule13, self.rule14, self.rule15,
                                                 self.rule16, self.rule17, self.rule18, self.rule19, self.rule20,
                                                 self.rule21, self.rule22, self.rule23, self.rule24, self.rule25])
        self.pendulum_fuzz = ctrl.ControlSystemSimulation(self.pendulum_ctrl)



        self.rule110 = ctrl.Rule(self.fuzz_vv2['poor'] & self.fuzz_position['poor'], self.fuzz_motor2['good'])
        self.rule111 = ctrl.Rule(self.fuzz_vv2['poor'] & self.fuzz_position['average'], self.fuzz_motor2['good'])
        self.rule112 = ctrl.Rule(self.fuzz_vv2['poor'] & self.fuzz_position['good'], self.fuzz_motor2['average'])
        self.rule113 = ctrl.Rule(self.fuzz_vv2['average'] & self.fuzz_position['poor'], self.fuzz_motor2['good'])
        self.rule114 = ctrl.Rule(self.fuzz_vv2['average'] & self.fuzz_position['average'], self.fuzz_motor2['average'])
        self.rule115 = ctrl.Rule(self.fuzz_vv2['average'] & self.fuzz_position['good'], self.fuzz_motor2['poor'])
        self.rule116 = ctrl.Rule(self.fuzz_vv2['good'] & self.fuzz_position['poor'], self.fuzz_motor2['average'])
        self.rule117 = ctrl.Rule(self.fuzz_vv2['good'] & self.fuzz_position['average'], self.fuzz_motor2['poor'])
        self.rule118 = ctrl.Rule(self.fuzz_vv2['good'] & self.fuzz_position['good'], self.fuzz_motor2['poor'])

        self.pendulum_ctrl2 = ctrl.ControlSystem([self.rule110,self.rule111, self.rule112, self.rule113, self.rule114,
                                                  self.rule115, self.rule116, self.rule117, self.rule118])

        self.pendulum_fuzz2 = ctrl.ControlSystemSimulation(self.pendulum_ctrl2)

        #self.rule1.view()

    def createWorld(self):
        self._isLiving = True
        self._auto = True

        self.ground = self.world.CreateBody(
            shapes=b2EdgeShape(vertices=[(-25, 0), (25, 0)])
        )

        self.carBody = self.world.CreateDynamicBody(
            position=(0, 3),
            fixtures=b2FixtureDef(
                shape=b2PolygonShape(box=(5, 1)), density=1)

        )

        self.carLwheel = self.world.CreateDynamicBody(
            position=(-3, 1),
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=1), density=2, friction=1)

        )

        self.carRwheel = self.world.CreateDynamicBody(
            position=(3, 1),
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=1), density=2, friction=1)

        )

        self.pendulum = self.world.CreateDynamicBody(
            position=(0, 13),
            fixtures=b2FixtureDef(
                shape=b2PolygonShape(box=(0.5, 10)), density=1),

        )

        self.pendelumJoin = self.world.CreateRevoluteJoint(
            bodyA=self.carBody,
            bodyB=self.pendulum,
            anchor=(0, 3),
            maxMotorTorque=1,
            enableMotor=True
        )

        self.pendelumRJoin =self.world.CreateRevoluteJoint(
            bodyA=self.carBody,
            bodyB=self.carRwheel,
            anchor=(3, 1),
            maxMotorTorque=1,
            enableMotor=True,
        )

        self.pendelumLJoin = self.world.CreateRevoluteJoint(
            bodyA=self.carBody,
            bodyB=self.carLwheel,
            anchor=(-3, 1),
            maxMotorTorque=1,
            enableMotor=True,
        )



    def destroyWorld(self):
        self.world.DestroyBody(self.carBody)
        self.world.DestroyBody(self.carLwheel)
        self.world.DestroyBody(self.carRwheel)
        self.world.DestroyBody(self.pendulum)
        self._isLiving = False


    def Keyboard(self, key):
        if key == Keys.K_a:
            if self._isLiving:
                self.pendelumLJoin.motorSpeed = 0
                self.pendelumLJoin.maxMotorTorque = 1000
                self.pendelumRJoin.motorSpeed = 0
                self.pendelumRJoin.maxMotorTorque = 1000
                self._auto = True

        elif key == Keys.K_n:
            if self._isLiving:
                self.destroyWorld()
                self.createWorld()


    def Step(self, settings):
        super(BodyPendulum, self).Step(settings)
        vv = ((self.pendulum.angle - self._angle_old) / 50) * 10000
        self._angle_old = self.pendulum.angle

        vv2 = ((self.carBody.position.x - self._pendulum_old_x) / 50) * 5000
        self._pendulum_old_x = self.carBody.position.x

        self.pendelumLJoin.maxMotorTorque = 1000
        self.pendelumRJoin.maxMotorTorque = 1000

        self.pendulum_fuzz.input['join1'] = self.pendulum.angle
        self.pendulum_fuzz.input['pend1_v'] = vv
        self.pendulum_fuzz.compute()

        self.pendulum_fuzz2.input['position'] = self.carBody.position.x
        self.pendulum_fuzz2.input['v'] = vv2
        self.pendulum_fuzz2.compute()

        xa = self.pendulum_fuzz2.output['motor2'] + self.pendulum_fuzz.output['motor']

        self.pendelumLJoin.motorSpeed = xa
        self.pendelumRJoin.motorSpeed = xa


if __name__ == "__main__":
    main(BodyPendulum)