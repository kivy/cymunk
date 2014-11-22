
import math
import operator

#: Version of Cymunk
__version__ = '0.1'

X, Y = 0, 1

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


def moment_for_poly(mass, vertices,  offset=(0, 0)):
    vertcount = len(vertices)
    cdef cpVect * cverts
    cverts = <cpVect *>malloc(sizeof(cpVect) * vertcount)
    for (i, vertex) in enumerate(vertices):
        cverts[i] = cpv(vertex[0], vertex[1])
    return cpMomentForPoly(mass, vertcount, cverts, cpv(offset[0], offset[1]))


def moment_for_box(mass, width, height):
    '''
    Calculate the moment of inertia for a box
    '''
    return cpMomentForBox(mass, width, height)

def enable_segment_to_segment_collisions():
    '''
    Enable segment to segment collisions
    '''
    cpEnableSegmentToSegmentCollisions()

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


def is_clockwise(points): 
    """
    Check if the points given forms a clockwise polygon
    
    :return: True if the points forms a clockwise polygon
    """
    a = 0
    i, j = 0, 0
    for i in range(len(points)):
        j = i + 1
        if j == len(points): j = 0
        a += points[i][X]*points[j][Y] - points[i][Y]*points[j][X]
    return a <= 0 #or is it the other way around?


cdef class Vec2d:

    def __cinit__(self, x, y=None):
        if y == None:
            self.v = cpv(x[0], x[1])
        else:
            self.v = cpv(x, y)

    property x:
        def __get__(self):
            return self.v.x
        def __set__(self, value):
            self.v.x = value

    property v:
        def __get__(self):
            return self.v

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
    
    def rotate(self, angle_radians):
        """Rotate the vector by angle_radians radians."""
        cdef float x
        cdef float y
        
        cos = math.cos(angle_radians)
        sin = math.sin(angle_radians)
        x = self.v.x*cos - self.v.y*sin
        y = self.v.x*sin + self.v.y*cos
        self.v.x = x
        self.v.y = y

    def rotated(self, angle_radians):
        """Create and return a new vector by rotating this vector by
        angle_radians radians.

        :return: Rotated vector
        """
        cdef float x
        cdef float y
        cos = math.cos(angle_radians)
        sin = math.sin(angle_radians)
        x = self.x*cos - self.y*sin
        y = self.x*sin + self.y*cos
        return Vec2d(x, y)

    # Comparison
    '''def __eq__(self, other):
        if hasattr(other, "__getitem__") and len(other) == 2:
            return self.x == other[0] and self.y == other[1]
        else:
            return False

    def __ne__(self, other):
        if hasattr(other, "__getitem__") and len(other) == 2:
            return self.x != other[0] or self.y != other[1]
        else:
            return True'''

    def __nonzero__(self):
        return self.x != 0.0 or self.y != 0.0

    # Generic operator handlers
    def _o2(self, other, f):
        "Any two-operator operation where the left operand is a Vec2d"
        if isinstance(other, Vec2d):
            return Vec2d(f(self.x, other.x),
                         f(self.y, other.y))
        elif (hasattr(other, "__getitem__")):
            return Vec2d(f(self.x, other[0]),
                         f(self.y, other[1]))
        else:
            return Vec2d(f(self.x, other),
                         f(self.y, other))

    def _r_o2(self, other, f):
        "Any two-operator operation where the right operand is a Vec2d"
        if (hasattr(other, "__getitem__")) and not isinstance(other, Vec2d):
            return (f(other[0], self.x),
                         f(other[1], self.y))
        elif (hasattr(other, "__getitem__")):
            return Vec2d(f(other[0], self.x),
                         f(other[1], self.y))
        else:
            return Vec2d(f(other, self.x),
                         f(other, self.y))

    def _io(self, other, f):
        "inplace operator"
        if (hasattr(other, "__getitem__")):
            self.x = f(self.x, other[0])
            self.y = f(self.y, other[1])
        else:
            self.x = f(self.x, other)
            self.y = f(self.y, other)
        return self

    # Addition
    def __add__(self, other):
        if isinstance(other, Vec2d):
            if isinstance(self, Vec2d):
                return Vec2d(self.x + other.x, self.y + other.y)
            else:
                return (self[0] + other.x, self[1] + other.y)
        elif hasattr(other, "__getitem__"):
            return Vec2d(self.x + other[0], self.y + other[1])
        else:
            return Vec2d(self.x + other, self.y + other)
    __radd__ = __add__

    def __iadd__(self, other):
        if isinstance(other, Vec2d):
            self.x += other.x
            self.y += other.y
        elif hasattr(other, "__getitem__"):
            self.x += other[0]
            self.y += other[1]
        else:
            self.x += other
            self.y += other
        return self

    # Subtraction
    def __sub__(self, other):
        if isinstance(other, Vec2d):
            if isinstance(self, Vec2d):
                return Vec2d(self.x - other.x, self.y - other.y)
            else:
                return (self[0] - other.x, self[1] - other.y)
        elif (hasattr(other, "__getitem__")):
            return Vec2d(self.x - other[0], self.y - other[1])
        else:
            return Vec2d(self.x - other, self.y - other)
    def __rsub__(self, other):
        if isinstance(other, Vec2d):
            return Vec2d(other.x - self.x, other.y - self.y)
        if (hasattr(other, "__getitem__")):
            return Vec2d(other[0] - self.x, other[1] - self.y)
        else:
            return Vec2d(other - self.x, other - self.y)
    def __isub__(self, other):
        if isinstance(other, Vec2d):
            self.x -= other.x
            self.y -= other.y
        elif (hasattr(other, "__getitem__")):
            self.x -= other[0]
            self.y -= other[1]
        else:
            self.x -= other
            self.y -= other
        return self

    # Multiplication
    def __mul__(self, other):
        if isinstance(other, Vec2d):
            if isinstance(self, Vec2d):
                return Vec2d(self.x * other.x, self.y * other.y)
            else:
                return (self[0] * other.x, self[1] * other.y)
        if (hasattr(other, "__getitem__")):
            return Vec2d(self.x*other[0], self.y*other[1])
        else:
            return Vec2d(self.x*other, self.y*other)
    __rmul__ = __mul__

    def __imul__(self, other):
        if isinstance(other, Vec2d):
            self.x *= other.x
            self.y *= other.y
        elif (hasattr(other, "__getitem__")):
            self.x *= other[0]
            self.y *= other[1]
        else:
            self.x *= other
            self.y *= other
        return self

    # Division
    def __div__(self, other):
        if isinstance(other, Vec2d):
            if isinstance(self, tuple):
                return (self[0] / other.x, self[1] / other.y)
        return self._o2(other, operator.div)
    def __rdiv__(self, other):
        return self._r_o2(other, operator.div)
    def __idiv__(self, other):
        return self._io(other, operator.div)

    def __floordiv__(self, other):
        return self._o2(other, operator.floordiv)
    def __rfloordiv__(self, other):
        return self._r_o2(other, operator.floordiv)
    def __ifloordiv__(self, other):
        return self._io(other, operator.floordiv)

    def __truediv__(self, other):
        return self._o2(other, operator.truediv)
    def __rtruediv__(self, other):
        return self._r_o2(other, operator.truediv)
    def __itruediv__(self, other):
        return self._io(other, operator.truediv)

    # Modulo
    def __mod__(self, other):
        return self._o2(other, operator.mod)
    def __rmod__(self, other):
        return self._r_o2(other, operator.mod)

    def __divmod__(self, other):
        return self._o2(other, divmod)
    def __rdivmod__(self, other):
        return self._r_o2(other, divmod)

    # Exponentation
    #def __pow__(self, other):
    #    return self._o2(other, operator.pow)
    #def __rpow__(self, other):
    #    return self._r_o2(other, operator.pow)

    # Bitwise operators
    def __lshift__(self, other):
        return self._o2(other, operator.lshift)
    def __rlshift__(self, other):
        return self._r_o2(other, operator.lshift)

    def __rshift__(self, other):
        return self._o2(other, operator.rshift)
    def __rrshift__(self, other):
        return self._r_o2(other, operator.rshift)

    def __and__(self, other):
        return self._o2(other, operator.and_)
    __rand__ = __and__

    def __or__(self, other):
        return self._o2(other, operator.or_)
    __ror__ = __or__

    def __xor__(self, other):
        return self._o2(other, operator.xor)
    __rxor__ = __xor__

    # Unary operations
    def __neg__(self):
        return Vec2d(operator.neg(self.x), operator.neg(self.y))

    def __pos__(self):
        return Vec2d(operator.pos(self.x), operator.pos(self.y))

    def __abs__(self):
        return Vec2d(abs(self.x), abs(self.y))

    def __invert__(self):
        return Vec2d(-self.x, -self.y)


    # Extra functions, mainly for chipmunk
    def cpvrotate(self, other):
        """Uses complex multiplication to rotate this vector by the other. """
        return Vec2d(self.x*other.x - self.y*other.y, self.x*other.y + self.y*other.x)
    def cpvunrotate(self, other):
        """The inverse of cpvrotate"""
        return Vec2d(self.x*other.x + self.y*other.y, self.y*other.x - self.x*other.y)

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
    
    property total_ke:
        """The amount of energy lost in a collision including static, but 
        not dynamic friction.
        
        This property should only be called from a post-solve, post-step"""
        
        def __get__(self):
            return cpArbiterTotalKE(self._arbiter)

