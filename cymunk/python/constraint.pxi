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

from cpython.ref cimport PyObject

cdef void _call_constraint_presolve_func(cpConstraint *constraint, cpSpace *space):
    global constraint_handlers
    py_space = <object><void *>space.data
    py_constraint = <object><void *>constraint.data
    constraint_dict = constraint_handlers[py_constraint]
    constraint_dict['pre_solve'](py_constraint, py_space)

cdef void _call_constraint_postsolve_func(cpConstraint *constraint, cpSpace *space):
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
            
        #self._pjc = cp.cast(self._constraint, ct.POINTER(cp.cpPivotJoint)).contents
        self._set_bodies(a,b)
        self._constraint.data = <cpDataPointer><void *>self
        global constraint_handlers
        constraint_handlers[self] = {}
    
#    def _get_anchr1(self):
#        return self._pjc.anchr1
#    def _set_anchr1(self, anchr):
#        self._pjc.anchr1 = anchr
#    anchr1 = property(_get_anchr1, _set_anchr1)
#    
#    def _get_anchr2(self):
#        return self._pjc.anchr2
#    def _set_anchr2(self, anchr):
#        self._pjc.anchr2 = anchr
#    anchr2 = property(_get_anchr2, _set_anchr2)
