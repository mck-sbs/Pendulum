# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2CircleShape)
from pyconsys.Control import Control
from pyconsys.PIDControl import PIDControl


class BodyPendulum(Framework):
    name = "Inverted Pendulum (PyConSys)"
    description = "(m) manual, (a) automatic, (d) delete, (c) create"
    speed = 3

    def __init__(self):
        super(BodyPendulum, self).__init__()

        self.createWorld()

    def createWorld(self):
        self._isLiving = True
        self._auto = False
        self._pid_control = PIDControl(100, 50, 2) #kp, ki, kd


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
            #motorSpeed=10
        )

        self.pendelumLJoin = self.world.CreateRevoluteJoint(
            bodyA=self.carBody,
            bodyB=self.carLwheel,
            anchor=(-3, 1),
            maxMotorTorque=1,
            enableMotor=True,
            #motorSpeed=10
        )

        # self.world.CreatePrismaticJoint(
        #     bodyA=self.carBody,
        #     bodyB=self.ground,
        #     anchor=self.carBody.worldCenter,
        #     axis=(1, 0),
        #     enableMotor=True
        #
        # )

    def destroyWorld(self):
        self.world.DestroyBody(self.carBody)
        self.world.DestroyBody(self.carLwheel)
        self.world.DestroyBody(self.carRwheel)
        self.world.DestroyBody(self.pendulum)
        self._isLiving = False
        # self.world.DestroyBody(self.pendelumLJoin)
        # self.world.DestroyBody(self.pendelumRJoin)
        # self.world.DestroyBody(self.pendelumJoin)

    def Keyboard(self, key):
        if key == Keys.K_a:
            if self._isLiving:
                #self.pendelumLJoin.motorSpeed = 10
                #self.pendelumLJoin.maxMotorTorque = 1000
                #self.pendelumRJoin.motorSpeed = 10
                #self.pendelumRJoin.maxMotorTorque = 1000
                self._auto = True

        elif key == Keys.K_m:
            self._auto = False
            if self._isLiving:
                self.pendelumLJoin.motorSpeed = 0
                self.pendelumLJoin.maxMotorTorque = 1
                self.pendelumRJoin.motorSpeed = 0
                self.pendelumRJoin.maxMotorTorque = 1000
        elif key == Keys.K_d:
            if self._isLiving:
                self.destroyWorld()
        elif key == Keys.K_c:
            if not self._isLiving:
                self.createWorld()


    def Step(self, settings):
        super(BodyPendulum, self).Step(settings)

        w = 0
        e = (w - self.pendulum.angle)*-1
        y = self._pid_control.get_xa(e)

        if self._auto and self._isLiving:
            self.pendelumLJoin.maxMotorTorque = 1000
            self.pendelumRJoin.maxMotorTorque = 1000
            self.pendelumLJoin.motorSpeed = y
            self.pendelumRJoin.motorSpeed = y

        #print(self.carBody.position.x)
        #print(self.pendulum.angle)
