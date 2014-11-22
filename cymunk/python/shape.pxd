cdef class Shape:
    cdef cpShape* _shape
    cdef int automanaged
    cdef Body _body

cdef class Circle(Shape):
    cdef float radius
    cdef cpVect offset

cdef class BoxShape(Shape):
    cdef float width
    cdef float height

cdef class Poly(Shape):
    cdef cpVect _offset
    cdef cpVect *_vertices
    cdef int     _vertices_count
    cdef tuple   offset
cdef class Segment(Shape):
    cdef cpVect a
    cdef cpVect b
    cdef float radius
    cdef cpSegmentShape* _segment_shape

cdef extern from "chipmunk/chipmunk.h":
    ctypedef struct cpSegmentShape:
        cpShape shape
        cpVect a, b, n
        cpVect ta, tb, tn
        cpFloat r
        cpVect a_tangent, b_tangent

    void cpShapeDestroy(cpShape *shape)
    void cpShapeFree(cpShape *shape)

    cpBB cpShapeCacheBB(cpShape *shape)
    cpBool cpShapePointQuery(cpShape *shape, cpVect p)

    cpBool cpShapeSegmentQuery(cpShape *shape, cpVect a, cpVect b, cpSegmentQueryInfo *info)

    cpShape cpCircleShape
    CP_DeclareShapeGetter(cpCircleShape, cpFloat, Radius)
    cpShape* cpCircleShapeNew(cpBody *body, cpFloat radius, cpVect offset)
    void cpCircleShapeSetRadius(cpShape *shape, cpFloat radius)
    void cpCircleShapeSetOffset(cpShape *shape, cpVect offset)

    cpShape* cpBoxShapeNew(cpBody *body, cpFloat width, cpFloat heigth)

    cpShape* cpSegmentShapeNew(cpBody *body, cpVect a, cpVect b, cpFloat radius)


    cpShape* cpPolyShapeNew(cpBody *body, int numVerts, cpVect *verts, cpVect offset)
    
    cpBool cpPolyValidate(cpVect *verts, int numVerts)
    
    cpVect cpPolyShapeGetVert(cpShape* shape, int idx)

