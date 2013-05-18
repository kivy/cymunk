cdef class Shape:
    '''
    Base class for all the shapes.

    You usually dont want to create instances of this class directly but use one
    of the specialized shapes instead.
    '''

    cdef cpShape* _shape
    cdef int automanaged
    cdef Body _body

    def __init__(self):
        self._shape = NULL
        self.automanaged = 1

    def __dealloc__(self):
        if self.automanaged:
            cpShapeFree(self._shape)

    property sensor:
        '''
        A boolean value if this shape is a sensor or not. Sensors only call
        collision callbacks, and never generate real collisions.
        '''
        def __get__(self):
            return self._shape.sensor
        def __set__(self, is_sensor):
            self._shape.sensor = is_sensor

    property collision_type:
        '''
        User defined collision type for the shape. See add_collisionpair_func
        function for more information on when to use this property
        '''
        def __get__(self):
            return self._shape.collision_type
        def __set__(self, t):
            self._shape.collision_type = t

    property group:
        '''
        Shapes in the same non-zero group do not generate collisions. Useful
        when creating an object out of many shapes that you don't want to self
        collide. Defaults to 0
        '''
        def __get__(self):
            return self._shape.group
        def __set__(self, group):
            self._shape.group = group

    property elasticity:
        '''
        Elasticity of the shape. A value of 0.0 gives no bounce, while a value
        of 1.0 will give a 'perfect' bounce. However due to inaccuracies in the
        simulation using 1.0 or greater is not recommended.
        '''
        def __get__(self):
            return self._shape.e
        def __set__(self, e):
            self._shape.e = e

    property friction:
        '''
        Friction coefficient. pymunk uses the Coulomb friction model, a value of
        0.0 is frictionless.
        '''
        def __get__(self):
            return self._shape.u
        def __set__(self, u):
            self._shape.u = u

    property surface_velocity:
        '''
        The surface velocity of the object. Useful for creating conveyor belts
        or players that move around. This value is only used when calculating
        friction, not resolving the collision.
        '''
        def __get__(self):
            return (self._shape.surface_v.x, self._shape.surface_v.y)
        def __set__(self, surf):
            self._shape.surface_v = cpv(surf[0], surf[1])

    property body:
        '''
        The body this shape is attached to
        '''
        def __get__(self):
            return self._body

    property _hashid_private:
        def __get__(self):
            return self._shape.hashid_private

    def cache_bb(self):
        '''
        Update and returns the bouding box of this shape
        '''
        return cpShapeCacheBB(self._shape)

    def point_query(self, p):
        '''
        Check if the given point lies within the shape
        '''
        return cpShapePointQuery(self._shape, cpv(p.x, p.y))

    def segment_query(self, start, end):
        '''
        Check if the line segment from start to end intersects the shape.
        '''
        cdef cpSegmentQueryInfo* info
        if cpShapeSegmentQuery(self._shape, cpv(start.x, start.y), cpv(end.x, end.y), info):
            return SegmentQueryInfo(self, start, end, info.t, info.n)
        return None


cdef class Circle(Shape):
    '''
    A circle shape defined by a radius

    This is the fastest and simplest collision shape
    '''
    cdef float radius

    def __init__(self, Body body, cpFloat radius, offset=(0, 0)):
        Shape.__init__(self)
        self._body = body
        self.radius = radius
        self._shape = cpCircleShapeNew(body._body, radius, cpv(offset[0], offset[1]))
        #self._cs = ct.cast(self._shape, ct.POINTER(cp.cpCircleShape))

    def unsafe_set_radius(self, r):
        '''
        Unsafe set the radius of the circle.
        '''
        cpCircleShapeSetRadius(self._shape, r)

    def unsafe_set_offset(self, o):
        '''
        Unsafe set the offset of the circle.
        '''
        cpCircleShapeSetOffset(self._shape, cpv(o.x, o.y))

    property radius:
        def __get__(self):
            return self.radius
        def __set__(self, radius):
            self.radius = radius

    def _get_radius(self):
        return self.radius
    #radius = property(_get_radius)

    #def _get_offset (self):
    #    return cp.cpCircleShapeGetOffset(self._shape)
    #offset = property(_get_offset)


cdef class Segment(Shape):
    '''
    A line segment shape between two points

    This shape can be attached to moving bodies, but don't currently generate
    collisions with other line segments. Can be beveled in order to give it a
    thickness.
    '''

    cdef cpSegmentShape* _segment_shape

    def __init__(self, Body body, a, b, cpFloat radius):
        Shape.__init__(self)
        self._body = body
        self._shape = cpSegmentShapeNew(body._body, cpv(a.x, a.y), cpv(b.x, b.y), radius)
        self._segment_shape = <cpSegmentShape *>self._shape

    property a:
        '''
        The first of the two endpoints for this segment
        '''
        def __get__(self):
            return (self._segment_shape.a.x, self._segment_shape.a.y)
        def __set__(self, a):
            self._segment_shape.a = cpv(a[0], a[1])

    property b:
        '''
        The second of the two endpoints for this segment
        '''
        def __get__(self):
            return (self._segment_shape.b.x, self._segment_shape.b.y)
        def __set__(self, a):
            self._segment_shape.b = cpv(a[0], a[1])

    property radius:
        # TODO
        '''
        The thickness of the segment
        '''
        def __get__(self):
            pass

cdef class BoxShape(Shape):

    cdef float width
    cdef float height

    def __init__(self, Body body, width, height):
        Shape.__init__(self)
        self._body = body
        self.width = width
        self.height = height
        self._shape = cpBoxShapeNew(body._body, width, height)

    property width:

        def __get__(self):
            return self.width
        def __set__(self, width):
            self.width = width

    property height:
        def __get__(self):
            return self.height
        def __set__(self, height):
            self.height = height


cdef class Poly(Shape):

    def __cinit__(self, Body body, vertices, offset=(0, 0), auto_order_vertices=True):
        Shape.__init__(self)
        self._body = body
        self.offset = offset

        #self.verts = (Vec2d * len(vertices))
        #self.verts = self.verts(Vec2d(0, 0))

        i_vs = enumerate(vertices)
        #if auto_order_vertices and not u.is_clockwise(vertices):
        #    i_vs = zip(range(len(vertices)-1, -1, -1), vertices)

        for i, vertex in i_vs:
            self.verts[i].x = vertex[0]
            self.verts[i].y = vertex[1]

        #self._shape = cpPolyShapeNew(body._body, len(vertices), self.verts, offset)


    #@staticmethod
    #def create_box(body, size=(10,10)):
    #    x,y = size[0]*.5,size[1]*.5
    #    vs = [(-x,-y),(-x,y),(x,y),(x,-y)]
    #    return Poly(body,vs)

    #def get_points(self):
    #    points = []
    #    rv = self._body.rotation_vector
    #    bp = self._body.position
    #    vs = self.verts
    #    o = self.offset
    #    for i in range(len(vs)):
    #        p = (vs[i]+o).cpvrotate(rv)+bp
    #        points.append(Vec2d(p))
    #    return points


cdef class SegmentQueryInfo:
    def __cinit__(self, shape, start, end, t, n):
        self._shape = shape
        self._t = t
        self._n = n
        self._start = start
        self._end = end

    def __repr__(self):
        return 'SegmentQueryInfo(%r, %r, %r, %r, %r)' % (
            self.shape, self._start, self._end, self.t, self.n)

    property shape:
        def __get__(self):
            return self._shape

    property t:
        def __get__(self):
            return self._t

    property n:
        def __get__(self):
            return self._n

    #def get_hit_point(self):
    #    return Vec2d(self._start).interpolate_to(self._end, self.t)

    #def get_hit_distance(self):
    #    return Vec2d(self._start).get_distance(self._end) * self.t
