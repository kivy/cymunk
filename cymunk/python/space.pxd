cdef extern from "chipmunk/chipmunk.h":
    ctypedef unsigned int cpTimestamp

    ctypedef struct cpSpace:
        # Number of iterations to use in the impulse solver to solve contacts
        int iterations
        # Gravity to pass to rigid bodies when integrating velocity
        cpVect gravity
        # Damping rate expressed as the fraction of velocity bodies retain each second
        cpFloat damping
        # Speed threshold for a body to be considered idle
        cpFloat idleSpeedThreshold
        # Time a group of bodies must remain idle in order to fall asleep
        cpFloat sleepTimeThreshold
        # Amount of encouraged penetration between colliding shapes
        cpFloat collisionSlop
        # Determines how fast overlapping shapes are pushed apart
        cpFloat collisionBias
        # Number of frames that contact information should persist
        cpTimestamp collisionPersistence
        # Rebuild the contact graph during each step
        cpBool enableContactGraph
        # User definable data pointer
        cpDataPointer data
        # The designated static body for this space
        cpBody *staticBody

    cdef cpSpace* cpSpaceAlloc()
    cdef cpSpace* cpSpaceInit(cpSpace *space)
    cdef cpSpace* cpSpaceNew()


    cdef void cpSpaceDestroy(cpSpace *space)
    cdef void cpSpaceFree(cpSpace *space)

    ctypedef void(* cpConstraintPreSolveFunc)(cpConstraint *constraint, cpSpace *space)
    ctypedef void(* cpConstraintPostSolveFunc)(cpConstraint *constraint, cpSpace *space)

    ctypedef struct cpConstraint:
        # The first body connected to this constraint
        cpBody *a
        # The second body connected to this constraint
        cpBody *b
        # The maximum force that this constraint is allowed to use
        cpFloat maxForce
        # The rate at which joint error is corrected
        cpFloat errorBias
        # The maximum rate at which joint error is corrected
        cpFloat maxBias
        # Function called before the solver runs
        cpConstraintPreSolveFunc preSolve
        # Function called after the solver runs
        cpConstraintPostSolveFunc postSolve
        # User definable data pointer
        cpDataPointer data

    cpShape *cpSpaceAddShape(cpSpace *space, cpShape *shape)
    cpShape *cpSpaceAddStaticShape(cpSpace *space, cpShape *shape)
    cpBody *cpSpaceAddBody(cpSpace *space, cpBody *body)
    cpConstraint *cpSpaceAddConstraint(cpSpace *space, cpConstraint *constraint)

    void cpSpaceRemoveShape(cpSpace *space, cpShape *shape)
    void cpSpaceRemoveStaticShape(cpSpace *space, cpShape *shape)
    void cpSpaceRemoveBody(cpSpace *space, cpBody *body)
    void cpSpaceRemoveConstraint(cpSpace *space, cpConstraint *constraint)
    ctypedef void(* cpSpaceBBQueryFunc)(cpShape *shape, void *data)
    void cpSpaceBBQuery(cpSpace *space, cpBB bb, cpLayers layers, cpGroup group, cpSpaceBBQueryFunc func, void *data)
    

    void cpSpaceReindexStatic(cpSpace *space)
    void cpSpaceReindexShape(cpSpace *space, cpShape *shape)
    void cpSpaceStep(cpSpace *space, cpFloat dt)

    ctypedef struct cpArbiter:
        # Calculated value to use for the elasticity coefficient
        cpFloat e
        # Calculated value to use for the friction coefficient
        cpFloat u
        # Calculated value to use for applying surface velocities
        cpVect surface_vr

    ctypedef cpBool(* cpCollisionBeginFunc)(cpArbiter *arb, cpSpace *space, void *data)
    ctypedef cpBool(* cpCollisionPreSolveFunc)(cpArbiter *arb, cpSpace *space, void *data)
    ctypedef cpBool(* cpCollisionPostSolveFunc)(cpArbiter *arb, cpSpace *space, void *data)
    ctypedef cpBool(* cpCollisionSeparateFunc)(cpArbiter *arb, cpSpace *space, void *data)

    void cpSpaceAddCollisionHandler(
        cpSpace *space,
        cpCollisionType a, cpCollisionType b,
        cpCollisionBeginFunc begin,
        cpCollisionPreSolveFunc preSolve,
        cpCollisionPostSolveFunc postSolve,
        cpCollisionSeparateFunc separate,
        void *data
    )

    void cpSpaceSetDefaultCollisionHandler(
            cpSpace *space,
            cpCollisionBeginFunc begin,
            cpCollisionPreSolveFunc preSolve,
            cpCollisionPostSolveFunc postSolve,
            cpCollisionSeparateFunc separate,
            void *data
    )

