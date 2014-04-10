cdef class BB:
    cdef cpBB _bb
    cdef float l
    cdef float b
    cdef float r
    cdef float t

cdef class Vec2d:
    cdef cpVect v