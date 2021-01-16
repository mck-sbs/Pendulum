from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2_dynamicBody,
                   b2_kinematicBody, b2_staticBody)


class BodyPendulum(Framework):
    name = "Pendulum (PyConSys)"
    description = "Inverse Pendulum (PyConSys)"
    speed = 3  # platform speed

    def __init__(self):
        super(BodyPendulum, self).__init__()

        # The ground
        ground = self.world.CreateBody(
            shapes=b2EdgeShape(vertices=[(-25, 0), (25, 0)])
        )

        # The attachment
        self.attachment = self.world.CreateDynamicBody(
            position=(0, 30),
            fixtures=b2FixtureDef(
                shape=b2PolygonShape(box=(0.5, 2)), density=2.0),
        )

        self.attachment = self.world.CreateDynamicBody(
            position=(0, 3),
            fixtures=b2FixtureDef(
                shape=b2PolygonShape(box=(0.5, 2)), density=2.0),
        )


    def Keyboard(self, key):
        pass

    def Step(self, settings):
        super(BodyPendulum, self).Step(settings)


