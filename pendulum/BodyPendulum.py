from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2CircleShape)


class BodyPendulum(Framework):
    name = "Pendulum (PyConSys)"
    description = "(m) manual, (a) automatic"
    speed = 3

    def __init__(self):
        super(BodyPendulum, self).__init__()

        self.ground = self.world.CreateBody(
            shapes=b2EdgeShape(vertices=[(-25, 0), (25, 0)])
        )

        self.carBody = self.world.CreateDynamicBody(
            position=(0, 3),
            fixtures=b2FixtureDef(
                shape=b2PolygonShape(box=(5, 1)), density=1),

        )

        self.carLwheel = self.world.CreateDynamicBody(
            position=(-3, 1),
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=1), density=1, friction=1)

        )

        self.carRwheel = self.world.CreateDynamicBody(
            position=(3, 1),
            fixtures=b2FixtureDef(
                shape=b2CircleShape(radius=1), density=1, friction=1)

        )

        self.pendulum = self.world.CreateDynamicBody(
            position=(0, 13),
            fixtures=b2FixtureDef(
                shape=b2PolygonShape(box=(0.5, 10)), density=1),

        )

        self.world.CreateRevoluteJoint(
            bodyA=self.carBody,
            bodyB=self.pendulum,
            anchor=(0, 3),
            maxMotorTorque=50,
            enableMotor=True
        )

        self.world.CreateRevoluteJoint(
            bodyA=self.carBody,
            bodyB=self.carRwheel,
            anchor=(3, 1),
            maxMotorTorque=1000,
            enableMotor=True,
            motorSpeed=10
        )

        self.world.CreateRevoluteJoint(
            bodyA=self.carBody,
            bodyB=self.carLwheel,
            anchor=(-3, 1),
            maxMotorTorque=1000,
            enableMotor=True,
            motorSpeed=10
        )
        # self.world.CreatePrismaticJoint(
        #     bodyA=self.carBody,
        #     bodyB=self.ground,
        #     anchor=self.carBody.worldCenter,
        #     axis=(1, 0),
        #     enableMotor=True
        #
        # )

    def Keyboard(self, key):
        pass

    def Step(self, settings):
        super(BodyPendulum, self).Step(settings)
        #print(self.carBody.position.x)
        print(self.pendulum.angle)
