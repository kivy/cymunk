cdef extern from "chipmunk/chipmunk.h":
    
    void     cpConstraintDestroy (cpConstraint *constraint)
    void     cpConstraintFree (cpConstraint *constraint)
    
    inline cpFloat cpConstraintGetImpulse(cpConstraint *constraint)
    
    # Callback function type that gets called before solving a joint.
    ctypedef void(* cpConstraintPreSolveFunc )(cpConstraint *constraint, cpSpace *space)
    
    # Callback function type that gets called after solving a joint. 
    ctypedef void(* cpConstraintPostSolveFunc )(cpConstraint *constraint, cpSpace *space) 
    
    ctypedef struct cpConstraint:
        cpBody *     a
        cpBody *     b 
        cpFloat     maxForce
        cpFloat     errorBias
        cpFloat     maxBias
        cpConstraintPreSolveFunc preSolve
        cpConstraintPostSolveFunc postSolve
        cpDataPointer     data

    ctypedef struct cpPivotJoint:
        cpConstraint constraint
        cpVect anchr1
        cpVect anchr2

    ctypedef struct cpSlideJoint:
        cpConstraint constraint
        cpVect anchr1
        cpVect anchr2
        cpFloat min
        cpFloat max

    ctypedef struct cpDampedSpring:
        cpConstraint constraint
        cpVect anchr1
        cpVect anchr2
        cpFloat restLength
        cpFloat stiffness
        cpFloat damping

    ctypedef struct cpPinJoint:
        cpConstraint constraint
        cpVect anchr1
        cpVect anchr2
        cpFloat dist

    ctypedef struct cpGrooveJoint:
        cpConstraint constraint
        cpVect grv_a
        cpVect grv_b
        cpVect anchr2

    ctypedef struct cpGearJoint:
        cpConstraint constraint
        cpFloat phase
        cpFloat ratio

    cpConstraint* cpPinJointNew(cpBody *a, cpBody *b, cpVect anchr1, 
        cpVect anchr2)
    cpConstraint* cpPivotJointNew(cpBody *a, cpBody *b, cpVect pivot)
    cpConstraint* cpPivotJointNew2(cpBody *a, cpBody *b, cpVect anchr1, 
        cpVect anchr2)
    cpConstraint* cpSlideJointNew(cpBody *a, cpBody *b, cpVect anchr1, 
        cpVect anchr2, cpFloat min, cpFloat max)
    cpConstraint* cpDampedSpringNew(cpBody *a, cpBody *b, cpVect anchr1, 
        cpVect anchr2, cpFloat restLength, cpFloat stiffness, cpFloat damping)
    cpConstraint* cpGrooveJointNew(cpBody *a, cpBody *b, cpVect groove_a, 
        cpVect groove_b, cpVect anchr2)
    cpConstraint* cpGearJointNew(cpBody *a, cpBody *b, cpFloat phase, 
        cpFloat ratio)

cdef class Constraint:
    cdef cpConstraint *_constraint
    cdef object _a
    cdef object _b
    cdef int automanaged

cdef class GrooveJoint(Constraint):
    cdef cpGrooveJoint *_groovejoint
    cdef tuple groove_b
    cdef tuple groove_a
    cdef tuple anchor2

cdef class PinJoint(Constraint):
    cdef cpPinJoint *_pinjoint
    cdef tuple anchor1
    cdef tuple anchor2

cdef class PivotJoint(Constraint):
    cdef cpPivotJoint *_pivotjoint
    cdef tuple anchor1
    cdef tuple anchor2

cdef class SlideJoint(Constraint):
    cdef cpSlideJoint *_slidejoint
    cdef tuple anchor1
    cdef tuple anchor2
    cdef float min
    cdef float max

cdef class DampedSpring(Constraint):
    cdef cpDampedSpring *_dampedspring
    cdef tuple anchor1
    cdef tuple anchor2
    cdef float rest_length
    cdef float stiffness
    cdef float damping

cdef class GearJoint(Constraint):
    cdef cpGearJoint *_gearjoint
    cdef float phase
    cdef float ratio