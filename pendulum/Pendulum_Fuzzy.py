# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2CircleShape)
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

class BodyPendulum(Framework):
    name = "Inverted Double Pendulum"
    description = "(m) manual, (a) automatic, (n) new"

    def __init__(self):
        super(BodyPendulum, self).__init__()

        self.createWorld()
        self.createFuzzy()

    def createFuzzy(self):
        self.fuzz_pend1 = ctrl.Antecedent(np.arange(-1, 1.1, 0.1), 'join1')
        self.fuzz_motor = ctrl.Consequent(np.arange(-300, 300, 2), 'motor')
        self.fuzz_pend1.automf(7)
        self.fuzz_motor.automf(7)

        #self.fuzz_pend1['average'].view()
        #self.fuzz_motor['average'].view()


        self.rule1 = ctrl.Rule(self.fuzz_pend1['dismal'], self.fuzz_motor['dismal'])
        self.rule2 = ctrl.Rule(self.fuzz_pend1['poor'], self.fuzz_motor['poor'])
        self.rule3 = ctrl.Rule(self.fuzz_pend1['mediocre'], self.fuzz_motor['mediocre'])
        self.rule4 = ctrl.Rule(self.fuzz_pend1['average'], self.fuzz_motor['average'])
        self.rule5 = ctrl.Rule(self.fuzz_pend1['decent'], self.fuzz_motor['decent'])
        self.rule6 = ctrl.Rule(self.fuzz_pend1['good'], self.fuzz_motor['good'])
        self.rule7 = ctrl.Rule(self.fuzz_pend1['excellent'], self.fuzz_motor['excellent'])

        self.pendulum_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3, self.rule4, self.rule5, self.rule6, self.rule7])
        self.pendulum_fuzz = ctrl.ControlSystemSimulation(self.pendulum_ctrl)

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

        elif key == Keys.K_m:
            self._auto = False
            if self._isLiving:
                self.pendelumLJoin.motorSpeed = 0
                self.pendelumLJoin.maxMotorTorque = 1
                self.pendelumRJoin.motorSpeed = 0
                self.pendelumRJoin.maxMotorTorque = 1
        elif key == Keys.K_n:
            if self._isLiving:
                self.destroyWorld()
                self.createWorld()


    def Step(self, settings):
        super(BodyPendulum, self).Step(settings)

        self.pendelumLJoin.maxMotorTorque = 1000
        self.pendelumRJoin.maxMotorTorque = 1000
        self.pendulum_fuzz.input['join1'] = self.pendulum.angle
        self.pendulum_fuzz.compute()
        self.pendelumLJoin.motorSpeed = self.pendulum_fuzz.output['motor']
        self.pendelumRJoin.motorSpeed = self.pendulum_fuzz.output['motor']

        #self.fuzz_motor.view(sim=self.pendulum_fuzz)

if __name__ == "__main__":
    main(BodyPendulum)