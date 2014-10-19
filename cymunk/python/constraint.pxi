"""A constraint is something that describes how two bodies interact with 
each other. (how they constrain each other). Constraints can be simple 
joints that allow bodies to pivot around each other like the bones in your 
body, or they can be more abstract like the gear joint or motors. 

This submodule contain all the constraints that are supported by pymunk.

Chipmunk has a good overview of the different constraint on youtube which 
works fine to showcase them in pymunk as well. 
http://www.youtube.com/watch?v=ZgJJZTS0aMM

.. raw:: html
    
    <iframe width="420" height="315" style="display: block; margin: 0 auto;"
    src="http://www.youtube.com/embed/ZgJJZTS0aMM" frameborder="0" 
    allowfullscreen></iframe>
    
"""   

constraint_handlers = {}

cdef void _call_constraint_presolve_func(cpConstraint *constraint,
    cpSpace *space):
    global constraint_handlers
    py_space = <object><void *>space.data
    py_constraint = <object><void *>constraint.data
    constraint_dict = constraint_handlers[py_constraint]
    constraint_dict['pre_solve'](py_constraint, py_space)

cdef void _call_constraint_postsolve_func(cpConstraint *constraint, 
    cpSpace *space):
    global constraint_handlers
    py_space = <object><void *>space.data
    py_constraint = <object><void *>constraint.data
    constraint_dict = constraint_handlers[py_constraint]
    constraint_dict['post_solve'](py_constraint, py_space)


cdef class Constraint:
    """Base class of all constraints. 
    
    You usually don't want to create instances of this class directly, but 
    instead use one of the specific constraints such as the PinJoint.
    """

    def __init__(self):
        self._constraint = NULL
        self.automanaged = 1

    def __dealloc__(self):
        global constraint_handlers
        del constraint_handlers[self]
        if self.automanaged:
            cpConstraintFree(self._constraint)

    property max_force:
        """The maximum force that the constraint can use to act on the two 
        bodies. Defaults to infinity"""
        def __get__(self):
            return self._constraint.maxForce
        def __set__(self, f):
            self._constraint.maxForce = f
    
    property error_bias:
        """The rate at which joint error is corrected.

        Defaults to pow(1.0 - 0.1, 60.0) meaning that it will correct 10% of 
        the error every 1/60th of a second."""
        
        def __get__(self):
            return self._constraint.errorBias
        def __set__(self, error_bias):
            self._constraint.errorBias = error_bias
            
    property max_bias:
        """The maximum rate at which joint error is corrected. Defaults 
            to infinity"""
            
        def __get__(self):
            return self._constraint.maxBias
        def __set__(self, max_bias):
            self._constraint.maxBias = max_bias
            
    property impulse:
        """Get the last impulse applied by this constraint."""
        
        def __get__(self):
            cdef float _res
            _res = cpConstraintGetImpulse(self._constraint)
            return _res
        
    property a:
        """The first of the two bodies constrained"""
        
        def __get__(self):
            return self._a

    property b:
        """The second of the two bodies constrained"""
        
        def __get__(self):
            return self._b

    property pre_solve:
        def __set__(self, func):
            self._set_py_presolve_handler(func)
            self._constraint.preSolve = _call_constraint_presolve_func

    property post_solve:
        def __set__(self, func):
            self._set_py_postsolve_handler(func)
            self._constraint.postSolve = _call_constraint_postsolve_func

    def activate_bodies(self):
        """Activate the bodies this constraint is attached to"""
        self._a.activate()
        self._b.activate()
    
    def _set_bodies(self, a, b):
        self._a = a
        self._b = b

    def _set_py_presolve_handler(self, presolve_func):
        global constraint_handlers
        constraint_handlers[self]['pre_solve'] = presolve_func

    def _set_py_postsolve_handler(self, postsolve_func):
        global constraint_handlers
        constraint_handlers[self]['post_solve'] = postsolve_func


cdef class GrooveJoint(Constraint):
    def __init__(self, Body a, Body b, tuple groove_a, tuple groove_b, tuple anchor2):
        self._constraint = cpGrooveJointNew(a._body, b._body, cpv(groove_a[0], 
            groove_a[1]), cpv(groove_b[0], groove_b[1]), cpv(anchor2[0], anchor2[1]))
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        self._groovejoint = <cpGrooveJoint *>self._constraint
        global constraint_handlers
        constraint_handlers[self] = {}

    property groove_a:
        def __get__(self):
            return self._groovejoint.grv_a
        def __set__(self, tuple new_groove_a):
            self._groovejoint.grv_a = cpv(new_groove_a[0], new_groove_a[1])

    property groove_b:
        def __get__(self):
            return self._groovejoint.grv_b
        def __set__(self, tuple new_groove_b):
            self._groovejoint.grv_b = cpv(new_groove_b[0], new_groove_b[1])

    property anchor2:
        def __get__(self):
            return self._groovejoint.anchr2
        def __set__(self, tuple new_anchor):
            self._groovejoint.anchr2 = cpv(new_anchor[0], new_anchor[1])


cdef class PinJoint(Constraint):

    def __init__(self, Body a, Body b, tuple anchor1, tuple anchor2):
        self._constraint = cpPinJointNew(a._body, b._body, cpv(anchor1[0], 
            anchor1[1]), cpv(anchor2[0], anchor2[1]))
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        self._pinjoint = <cpPinJoint *>self._constraint
        global constraint_handlers
        constraint_handlers[self] = {}

    property anchor1:
        def __get__(self):
            return self._pinjoint.anchr1
        def __set__(self, tuple new_anchor):
            self._pinjoint.anchr1 = cpv(new_anchor[0], new_anchor[1])

    property anchor2:
        def __get__(self):
            return self._pinjoint.anchr2
        def __set__(self, tuple new_anchor):
            self._pinjoint.anchr2 = cpv(new_anchor[0], new_anchor[1])


cdef class DampedSpring(Constraint):

    def __init__(self, Body a, Body b, tuple anchor1, 
        tuple anchor2, float rest_length, float stiffness, float damping):
        self._constraint = cpDampedSpringNew(a._body, b._body, cpv(anchor1[0], 
            anchor1[1]), cpv(anchor2[0], anchor2[1]), rest_length, stiffness,
            damping)
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        self._dampedspring = <cpDampedSpring *>self._constraint
        global constraint_handlers
        constraint_handlers[self] = {}

    property anchor1:
        def __get__(self):
            return self._dampedspring.anchr1
        def __set__(self, tuple new_anchor):
            self._dampedspring.anchr1 = cpv(new_anchor[0], new_anchor[1])

    property anchor2:
        def __get__(self):
            return self._dampedspring.anchr2
        def __set__(self, tuple new_anchor):
            self._dampedspring.anchr2 = cpv(new_anchor[0], new_anchor[1])

    property rest_length:
        def __get__(self):
            return self._dampedspring.restLength
        def __set__(self, float new_rest_length):
            self._dampedspring.restLength = new_rest_length

    property stiffness:
        def __get__(self):
            return self._dampedspring.stiffness
        def __set__(self, float new_stiffness):
            self._dampedspring.stiffness = new_stiffness

    property damping:
        def __get__(self):
            return self._dampedspring.damping
        def __set__(self, float new_damping):
            self._dampedspring.damping = new_damping

cdef class DampedRotarySpring(Constraint):

    def __init__(self, Body a, Body b, float rest_angle, float stiffness, 
        float damping):
        self._constraint = cpDampedRotarySpringNew(a._body, b._body, 
            rest_angle, stiffness,
            damping)
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        self._dampedspring = <cpDampedRotarySpring *>self._constraint
        global constraint_handlers
        constraint_handlers[self] = {}

    property rest_angle:
        def __get__(self):
            return self._dampedspring.restAngle
        def __set__(self, float new_rest_angle):
            self._dampedspring.restAngle = new_rest_angle

    property stiffness:
        def __get__(self):
            return self._dampedspring.stiffness
        def __set__(self, float new_stiffness):
            self._dampedspring.stiffness = new_stiffness

    property damping:
        def __get__(self):
            return self._dampedspring.damping
        def __set__(self, float new_damping):
            self._dampedspring.damping = new_damping


cdef class RotaryLimitJoint(Constraint):

    def __init__(self, Body a, Body b, float min, float max):
        self._constraint = cpRotaryLimitJointNew(a._body, b._body, min, max)
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        self._rotaryLimitJoint = <cpRotaryLimitJoint *>self._constraint
        global constraint_handlers
        constraint_handlers[self] = {}

    property min:
        def __get__(self):
            return self._rotaryLimitJoint.min
        def __set__(self, float new_min):
            self._rotaryLimitJoint.min = new_min

    property max:
        def __get__(self):
            return self._rotaryLimitJoint.max
        def __set__(self, float new_max):
            self._rotaryLimitJoint.max = new_max


cdef class SlideJoint(Constraint):

    def __init__(self, Body a, Body b, tuple anchor1, 
        tuple anchor2, float _min, float _max):
        self._constraint = cpSlideJointNew(a._body, b._body, cpv(anchor1[0], 
            anchor1[1]), cpv(anchor2[0], anchor2[1]), _min, _max)
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        self._slidejoint = <cpSlideJoint *>self._constraint
        global constraint_handlers
        constraint_handlers[self] = {}

    property anchor1:
        def __get__(self):
            return self._slidejoint.anchr1
        def __set__(self, tuple new_anchor):
            self._slidejoint.anchr1 = cpv(new_anchor[0], new_anchor[1])

    property anchor2:
        def __get__(self):
            return self._slidejoint.anchr2
        def __set__(self, tuple new_anchor):
            self._slidejoint.anchr2 = cpv(new_anchor[0], new_anchor[1])

    property min:
        def __get__(self):
            return self._slidejoint.min
        def __set__(self, float new_min):
            self._slidejoint.min = new_min

    property max:
        def __get__(self):
            return self._slidejoint.max
        def __set__(self, float new_max):
            self._slidejoint.max = new_max


cdef class PivotJoint(Constraint):
    
    """Simply allow two objects to pivot about a single point."""
    
    def __init__(self, Body a, Body b, *args):
        """a and b are the two bodies to connect, and pivot is the point in
        world coordinates of the pivot. Because the pivot location is given in
        world coordinates, you must have the bodies moved into the correct
        positions already. 
        Alternatively you can specify the joint based on a pair of anchor 
        points, but make sure you have the bodies in the right place as the 
        joint will fix itself as soon as you start simulating the space. 
        
        That is, either create the joint with PivotJoint(a, b, pivot) or 
        PivotJoint(a, b, anchr1, anchr2).
        
            a : `Body`
                The first of the two bodies
            b : `Body`
                The second of the two bodies
            args : [Vec2d] or [Vec2d,Vec2d]
                Either one pivot point, or two anchor points
        """
        
        cdef cpVect pivot
        cdef list anchors
        cdef int i
        
        anchors = []
        if len(args) == 1:
            if isinstance(args[0], Vec2d):
                pivot = args[0].v
            elif isinstance(args[0], tuple):
                pivot = cpv(args[0][0], args[0][1])
            else:
                raise Exception('Argument must be Vec2d or tuple')
            self._constraint = cpPivotJointNew(a._body, b._body, pivot)

        elif len(args) == 2:
            for i in range(2):
                if isinstance(args[i], Vec2d):
                    anchors.append(cpv(args[i].x, args[i].y))
                elif isinstance(args[i], tuple):
                    anchors.append(cpv(args[i][0], args[i][1]))
                else:
                    raise Exception('Argument must be Vec2d or tuple')
            self._constraint = cpPivotJointNew2(a._body, b._body, anchors[0], anchors[1])
        else:
            raise Exception("You must specify either one pivot point or two anchor points")
            
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        self._pivotjoint = <cpPivotJoint *>self._constraint
        global constraint_handlers
        constraint_handlers[self] = {}
    
    property anchor1:
        def __get__(self):
            return self._pivotjoint.anchr1
        def __set__(self, tuple new_anchor):
            self._pivotjoint.anchr1 = cpv(new_anchor[0], new_anchor[1])

    property anchor2:
        def __get__(self):
            return self._pivotjoint.anchr2
        def __set__(self, tuple new_anchor):
            self._pivotjoint.anchr2 = cpv(new_anchor[0], new_anchor[1])


cdef class GearJoint(Constraint):
    
    def __init__(self, Body a, Body b, float phase, float ratio):
        self._constraint = cpGearJointNew(a._body, b._body, phase, ratio)
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        self._gearjoint = <cpGearJoint *>self._constraint
        global constraint_handlers
        constraint_handlers[self] = {}

    property phase:
        def __get__(self):
            return self._gearjoint.phase
        def __set__(self, float value):
            self._gearjoint.phase = value

    property ratio:
        def __get__(self):
            return self._gearjoint.ratio
        def __set__(self, float value):
            self._gearjoint.ratio = value