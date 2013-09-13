ctypedef int bool
ctypedef int datapointer

cdef extern from "chipmunk/chipmunk.h":
    cdef void cpInitChipmunk()

    ctypedef unsigned long long cpHashValue
    ctypedef float cpFloat
    cpFloat INFINITY
    ctypedef struct cpVect:
        cpFloat x,y
    ctypedef datapointer cpDataPointer
    ctypedef bool cpBool
    ctypedef int cpGroup
    ctypedef int cpLayers
    ctypedef int cpCollisionType
    ctypedef struct cpBB:
        cpFloat l, b, r ,t
    cpVect cpv (cpFloat x, cpFloat y)

    ctypedef unsigned int cpTimestamp


    ctypedef struct cpBody:
        # Mass of the body
        cpFloat m
        # Mass inverse
        cpFloat m_inv
        # Moment of inertia of the body
        cpFloat i
        # Moment of inertia inverse
        cpFloat i_inv
        # Position of the rigid body's center of gravity
        cpVect p
        # Velocity of the rigid body's center of gravity
        cpVect v
        # Force acting on the rigid body's center of gravity
        cpVect f
        # Rotation of the body around it's center of gravity in radians
        cpFloat a
        # Angular velocity of the body around it's center of gravity in radians/second
        cpFloat w
        # Torque applied to the body around it's center of gravity
        cpFloat t
        # Cached unit length vector representing the angle of the body
        cpVect rot
        # User definable data pointer
        cpDataPointer data
        # Maximum velocity allowed when updating the velocity
        cpFloat v_limit
        # Maximum rotational rate (in radians/second) allowed when updating the angular velocity
        cpFloat w_limit


    ctypedef struct cpShape:
        # The rigid body this collision shape is attached to
        cpBody *body
        # The current bounding box of the shape
        cpBB bb
        # Sensor flag
        cpBool sensor
        # Coefficient of restitution (elasticity)
        cpFloat e
        # Coefficient of friction
        cpFloat u
        # Surface velocity used when solving for friction
        cpVect surface_v
        # User definable data pointer
        cpDataPointer data
        # Collision type of this shape used when picking collision handlers
        cpCollisionType collision_type
        # Group of this shape. Shapes in the same group don't collide
        cpGroup group
        # Hash
        cpHashValue hashid_private


    ctypedef struct cpSegmentQueryInfo:
        # The shape that was hit, NULL if no collision occured
        cpShape *shape
        # The normalized distance along the query segment in the range [0, 1]
        cpFloat t
        # The normal of the surface hit
        cpVect n

    ctypedef struct cpContactPointSet:
        # The number of contact points in the set
        int count

    ctypedef struct cpBB:
        cpFloat l, b, r, t

    ctypedef struct cpArbiter:
        # Calculated value to use for the elasticity coefficient
        cpFloat e
        # Calculated value to use for the friction coefficient
        cpFloat u
        # Calculated value to use for applying surface velocities
        cpVect surface_vr

        #cpTimestamp stamp

    cpFloat cpMomentForCircle(cpFloat m, cpFloat r1, cpFloat r2, cpVect offset)
    cpFloat cpMomentForSegment(cpFloat m, cpVect a, cpVect b)
    cpFloat cpMomentForPoly(cpFloat m, int numVerts, cpVect *verts, cpVect offset)
    cpFloat cpMomentForBox(cpFloat m, cpFloat width, cpFloat height)
    void cpResetShapeIdCounter()

    cpBB cpBBNew(cpFloat l, cpFloat b, cpFloat r, cpFloat t)
    cpBool cpBBIntersects(cpBB a, cpBB b)
    cpBool cpBBContainsBB(cpBB bb, cpBB other)
    cpBool cpBBContainsVect(cpBB bb, cpVect v)
    #cpBB cpBBMerge(cpBB a, cpBB b)
    #cpBB cpBBExpand(cpBB bb, cpVect v)
    cpVect cpBBClampVect(cpBB bb, cpVect v)
    cpVect cpBBWrapVect(cpBB bb, cpVect v)


    cpContactPointSet cpArbiterGetContactPointSet(cpArbiter *arb)
    void cpArbiterGetShapes(cpArbiter *arb, cpShape **a, cpShape **b)
    cpVect cpArbiterTotalImpulse(cpArbiter *arb)
    cpVect cpArbiterTotalImpulseWithFriction(cpArbiter *arb)
    cpBool cpArbiterIsFirstContact(cpArbiter *arb)
    cpVect cpArbiterGetNormal(cpArbiter *arb, int i)
    cpVect cpArbiterGetPoint(cpArbiter *arb, int i)
    cpFloat cpArbiterGetDepth(cpArbiter *arb, int i)
    cpFloat cpArbiterTotalKE(const cpArbiter *arb)



include "core.pxd"
include "body.pxd"
include "shape.pxd"
include "space.pxd"
include "constraint.pxd"
