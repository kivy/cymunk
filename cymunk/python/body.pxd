cdef extern from "chipmunk/chipmunk.h":
    cpBody *cpBodyAlloc()
    cpBody *cpBodyInit(cpBody *body, cpFloat m, cpFloat i)
    cpBody* cpBodyNewStatic()
    cpBody *cpBodyNew(cpFloat m, cpFloat i)

    void cpBodyDestroy(cpBody *body)
    void cpBodyFree(cpBody *body)

    void cpBodySetMass(cpBody *body, cpFloat m)
    void cpBodySetMoment(cpBody *body, cpFloat i)
    void cpBodySetAngle(cpBody *body, cpFloat a)

    void cpBodyUpdateVelocity(cpBody *body, cpVect gravity, cpFloat damping, cpFloat dt)
    void cpBodyUpdatePosition(cpBody *body, cpFloat dt)
    void cpBodyApplyImpulse(cpBody *body, cpVect j, cpVect r)
    void cpBodyResetForces(cpBody *body)
    void cpBodyApplyForce(cpBody *body, cpVect f, cpVect r)

    ctypedef void (*cpBodyVelocityFunc)(cpBody *body, cpVect gravity, cpFloat damping, cpFloat dt)
    cpBodyVelocityFunc velocity_func


    void cpBodyActivate(cpBody *body)
    void cpBodySleep(cpBody *body)
    void cpBodySleepWithGroup(cpBody *body, cpBody *group)

    cpBool cpBodyIsSleeping(cpBody *body)
    cpBool cpBodyIsRogue(cpBody *body)
    cpBool cpBodyIsStatic(cpBody *body)

    cpVect cpBodyLocal2World(cpBody *body, cpVect v)
    cpVect cpBodyWorld2Local(cpBody *body, cpVect v)


cdef class Body:
    cdef cpBody* _body
    cdef int automanaged

