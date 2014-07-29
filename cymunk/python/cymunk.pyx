include "constraint.pxi"
include "core.pxi"
include "body.pxi"
include "shape.pxi"
include "space.pxi"

from types import ModuleType


# Pymunk API capability hook
# to emulate vec2d module
class Vec2dModule(ModuleType):
    pass

vec2d = Vec2dModule('cymunk.vec2d')
vec2d.Vec2d = Vec2d


# Pymunk API capability hook
# to emulate constraint module
class ConstraintsModule(ModuleType):
    pass

constraint = ConstraintsModule('cymunk.constraint')
constraint.Constraint = Constraint
constraint.PivotJoint = PivotJoint
constraint.PinJoint = PinJoint
constraint.GrooveJoint = GrooveJoint
constraint.SlideJoint = SlideJoint
constraint.DampedSpring = DampedSpring
constraint.DampedRotarySpring = DampedRotarySpring
constraint.GearJoint = GearJoint
