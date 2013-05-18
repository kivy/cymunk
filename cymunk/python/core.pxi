
#: Version of Cymunk
__version__ = '0.1'

# init the library, whatever we will do.
cpInitChipmunk()

def moment_for_circle(mass, inner_radius, outer_radius, offset=(0, 0)):
    '''
    Calculate the moment of inertia for a circle
    '''
    return cpMomentForCircle(mass, inner_radius, outer_radius, cpv(offset[0], offset[1]))


def moment_for_segment(mass, a, b):
    '''
    Calculate the moment of inertia for a segment
    '''
    return cpMomentForSegment(mass, cpv(a.x, a.y), cpv(b.x, b.y))


#def moment_for_poly(mass, vertices,  offset=(0, 0)):
#    verts = (Vec2d * len(vertices))
#    verts = verts(Vec2d(0, 0))
#    for (i, vertex) in enumerate(vertices):
#        verts[i].x = vertex[0]
#        verts[i].y = vertex[1]
#    return cpMomentForPoly(mass, len(verts), verts, offset)


def moment_for_box(mass, width, height):
    '''
    Calculate the moment of inertia for a box
    '''
    return cpMomentForBox(mass, width, height)


def reset_shapeid_counter():
    '''
    Reset the internal shape counter

    cymunk keeps a counter so that every new shape is given a unique hash value
    to be used in the spatial hash. Because this affects the order in which the
    collisions are found and handled, you should reset the shape counter every
    time you populate a space with new shapes. If you don't, there might be
    (very) slight differences in the simulation.
    '''
    cpResetShapeIdCounter()


cdef class Vec2d:
    cdef cpVect v

    def __cinit__(self, float x, float y):
        self.v = cpv(x, y)

    property x:
        def __get__(self):
            return self.v.x
        def __set__(self, value):
            self.v.x = value

    property y:
        def __get__(self):
            return self.v.y
        def __set__(self, value):
            self.v.y = value

    def __getitem__(self, index):
        if index == 0:
            return self.v.x
        elif index == 1:
            return self.v.y
        raise IndexError('Invalid index %r, must be 0 or 1' % index)

    def __setitem__(self, index, value):
        if index == 0:
            self.v.x = value
        elif index == 1:
            self.v.y = value
        else:
            raise IndexError('Invalid index %r, must be 0 or 1' % index)

    def __repr__(self):
        return '<cymunk.Vec2d x=%f y=%f>' % (self.v.x, self.v.y)

cdef class Contact:
    '''
    Contact informations
    '''
    cdef Vec2d _point
    cdef Vec2d _normal
    cdef float _dist

    def __cinit__(self, point, normal, dist):
        self._point = point
        self._normal = normal
        self._dist = dist

    def __repr__(self):
        return 'Contact(%r, %r, %r)' % (
            self.position, self.normal, self.distance)

    property position:
        '''
        Contact position
        '''
        def __get__(self):
            return self._point

    property normal:
        '''
        Contact normal
        '''
        def __get__(self):
            return self._normal

    property distance:
        '''
        Contact distance
        '''
        def __get__(self):
            return self._dist


cdef class BB:
    cdef cpBB _bb
    cdef float l
    cdef float b
    cdef float r
    cdef float t

    def __cinit__(self, float l, float b, float r, float t):
        self._bb = cpBBNew(l, b, r, t)

    def __repr__(self):
        return 'BB(%s, %s, %s, %s)' % (self._bb.l, self._bb.b, self._bb.r, self._bb.t)

    # def __eq__(self, other):
    #     return self.l == other.l and self.b == other.b and \
    #         self.r == other.r and self.t == other.l

    #def __ne__(self, other):
    #    return not self.__eq__(other)

    #def intersects(self, other):
    #    return cpBBIntersects(self._bb, other._bb)

    #def contains(self, other):
    #    return cpBBContainsBB(self._bb, other._bb)

    #def contains_vect(self, v):
    #    return cpBBContainsVect(self._bb, cpv(v.x, v.y))

    #def merge(self, other):
    #    return BB(cpBBMerge(self._bb, other._bb))

    #def expand(self, v):
    #    return BB(cpBBExpand(self._bb, cpv(v.x, v.y)))

    property _bb:
        def __get__(self):
            return self._bb

    property l:
        def __get__(self):
            return self._bb.l

    property b:
        def __get__(self):
            return self._bb.b

    property r:
        def __get__(self):
            return self._bb.r

    property t:
        def __get__(self):
            return self._bb.t

    #def clamp_vect(self, v):
    #    return cpBBClampVect(self._bb, cpv(v.x, v.y))

    #def wrap_vect(self, v):
    #    return cpBBWrapVect(self._bb, cpv(v.x, v.y))


cdef class Arbiter:
    '''
    Arbiters are collision pairs between shapes that are used with the collision
    callbacks.

    .. warning::

        Because arbiters are handled by the space you should never hold onto a
        reference to an arbiter as you don't know when it will be destroyed! Use
        them within the callback where they are given to you and then forget
        about them or copy out the information you need from them.
    '''

    cdef cpArbiter* _arbiter
    cdef Space _space
    cdef list _contacts

    def __cinit__(self, space):
        self._arbiter = NULL
        self._space = space
        self._contacts = None

    property contacts:
        '''
        Information on the contact points between the objects. Return [`Contact`]
        '''
        def __get__(self):
            cdef int i
            cdef cpContactPointSet point_set
            cdef cpVect point, normal
            if self._contacts is None:
                point_set = cpArbiterGetContactPointSet(self._arbiter)
                self._contacts = []
                for i in xrange(point_set.count):
                    point = cpArbiterGetPoint(self._arbiter, i)
                    normal = cpArbiterGetNormal(self._arbiter, i)
                    self._contacts.append(Contact(
                        Vec2d(point.x, point.y),
                        Vec2d(normal.x, normal.y),
                        cpArbiterGetDepth(self._arbiter, i)))
                return self._contacts

    property shapes:
        '''
        Shapes associated to the contact, in the same order as the collision
        callback
        '''
        def __get__(self):
            cdef cpShape* shapeA_p = NULL
            cdef cpShape* shapeB_p = NULL
            cpArbiterGetShapes(self._arbiter, &shapeA_p, &shapeB_p)
            a = self._space._get_shape(shapeA_p)
            b = self._space._get_shape(shapeB_p)
            return a, b

 
    property elasticity:
        '''
        Elasticity
        '''
        def __get__(self):
            return self._arbiter.e
        def __set__(self, value):
            self._arbiter.e = value

    property friction:
        '''
        Friction
        '''
        def __get__(self):
            return self._arbiter.u
        def __set__(self, value):
            self._arbiter.u = value

    property velocity:
        '''
        Velocity
        '''
        def __get__(self):
            return self._arbiter.surface_vr

    property total_impulse:
        '''
        Returns the impulse that was applied this step to resolve the collision
        '''
        def __get__(self):
            return cpArbiterTotalImpulse(self._arbiter)

    property total_impulse_with_friction:
        '''
        Returns the impulse with friction that was applied this step to resolve the collision
        '''
        def __get__(self):
            return cpArbiterTotalImpulseWithFriction(self._arbiter)

    #property stamp:
    #    '''
    #    Time stamp of the arbiter. (from the space)
    #    '''
    #    def __get__(self):
    #        return self._arbiter.stamp

    property is_first_contact:
        '''
        Returns true if this is the first step that an arbiter existed. You can
        use this from preSolve and postSolve to know if a collision between two
        shapes is new without needing to flag a boolean in your begin callback.
        '''
        def __get__(self):
            return cpArbiterIsFirstContact(self._arbiter)

