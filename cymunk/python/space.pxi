from cpython.ref cimport PyObject, Py_INCREF, Py_DECREF, Py_XDECREF


current_spaces = []
handlers = {}
    
cdef void _call_space_bb_query_func(cpShape *shape, void *data):
    global current_spaces
    global handlers
    space = current_spaces[0]
    py_shape = space.shapes[shape.hashid_private]
    handlers['bb_query_func'](py_shape)

cdef void _call_space_segment_query_func(cpShape *shape, cpFloat t, cpVect n, void *data):
    global current_spaces
    global handlers
    space = current_spaces[0]
    py_shape = space.shapes[shape.hashid_private]
    handlers['segment_query_func'](py_shape, t, n)


cdef bool _call_collision_begin_func(cpArbiter *_arb, cpSpace *_space, void *_data):
    global current_spaces
    global handlers
    space = current_spaces[0]
    arbiter = Arbiter(space)
    arbiter._arbiter = _arb
    a = arbiter.shapes[0].collision_type
    b = arbiter.shapes[1].collision_type
    if handlers[(a, b)]['begin_func'] is not None:
        if handlers[(a, b)]['begin_func'](arbiter, space):
            return True
        else:
            return False
    return True

cdef bool _call_collision_pre_solve_func(cpArbiter *_arb, cpSpace *_space, void *_data):
    global current_spaces
    global handlers
    space = current_spaces[0]
    arbiter = Arbiter(space)
    arbiter._arbiter = _arb
    a = arbiter.shapes[0].collision_type
    b = arbiter.shapes[1].collision_type
    if handlers[(a, b)]['pre_solve_func'] is not None:
        if handlers[(a, b)]['pre_solve_func'](arbiter, space):
            return True
        else:
            return False
    return True

cdef bool _call_collision_post_solve_func(cpArbiter *_arb, cpSpace *_space, void *_data):
    global handlers
    global current_spaces
    space = current_spaces[0]
    arbiter = Arbiter(space)
    arbiter._arbiter = _arb
    a = arbiter.shapes[0].collision_type
    b = arbiter.shapes[1].collision_type
    if handlers[(a, b)]['post_solve_func'] is not None:
        if handlers[(a, b)]['post_solve_func'](arbiter, space):
            return True
        else:
            return False
    return False

cdef bool _call_collision_separate_func(cpArbiter *_arb, cpSpace *_space, void *_data):
    global handlers
    global current_spaces
    space = current_spaces[0]
    arbiter = Arbiter(space)
    arbiter._arbiter = _arb
    a = arbiter.shapes[0].collision_type
    b = arbiter.shapes[1].collision_type
    if handlers[(a, b)]['separate_func'] is not None:
        if handlers[(a, b)]['separate_func'](arbiter, space):
            return True
        else:
            return False
    return False


cdef bool _collision_begin_func(cpArbiter *_arb, cpSpace *_space, void *_data):
    cdef PyObject *obj = <PyObject *>_data
    cdef Space space = <Space>obj
    cdef object func
    cdef Arbiter arbiter
    if space._default_handlers is not None:
        func = space._default_handlers[0]
        if func is not None:
            arbiter = Arbiter(space)
            arbiter._arbiter = _arb 
            if not func(arbiter,
                    *space._default_handlers[-2],
                    **space._default_handlers[-1]):
                return False
    return True

cdef bool _collision_pre_solve_func(cpArbiter *_arb, cpSpace *_space, void *_data):
    cdef PyObject *obj = <PyObject *>_data
    cdef Space space = <Space>obj
    cdef object func
    cdef Arbiter arbiter
    if space._default_handlers is not None:
        func = space._default_handlers[1]
        if func is not None:
            arbiter = Arbiter(space)
            arbiter._arbiter = _arb
            if not func(arbiter,
                    *space._default_handlers[-2],
                    **space._default_handlers[-1]):
                return False

    return True

cdef bool _collision_post_solve_func(cpArbiter *_arb, cpSpace *_space, void *_data):
    cdef PyObject *obj = <PyObject *>_data
    cdef Space space = <Space>obj
    cdef object func
    cdef Arbiter arbiter
    if space._default_handlers is not None:
        func = space._default_handlers[2]
        if func is not None:
            arbiter = Arbiter(space)
            arbiter._arbiter = _arb
            if func(arbiter,
                    *space._default_handlers[-2],
                    **space._default_handlers[-1]):
                return True

    return False

cdef bool _collision_seperate_func(cpArbiter *_arb, cpSpace *_space, void *_data):
    cdef PyObject *obj = <PyObject *>_data
    cdef Space space = <Space>obj
    cdef object func
    cdef Arbiter arbiter
    if space._default_handlers is not None:
        func = space._default_handlers[3]
        if func is not None:
            arbiter = Arbiter(space)
            arbiter._arbiter = _arb
            if func(arbiter,
                    *space._default_handlers[-2],
                    **space._default_handlers[-1]):
                return True

    return False

cdef class Space:
    '''
    Spaces are the basic unit of simulation. You add rigid bodies, shapes and
    joints to it and then step them all forward together through time.
    '''
    cdef cpSpace* _space
    cdef Body _static_body
    cdef dict _shapes
    cdef dict _static_shapes
    cdef list _bodies
    cdef list _constraints
    cdef dict _post_step_callbacks
    cdef dict _handlers
    cdef tuple _default_handlers

    def __init__(self, int iterations=10):
        '''
        Create a new instace of the Space

        Its usually best to keep the elastic_iterations setting to 0. Only
        change if you have problem with stacking elastic objects on each other.
        If that is the case, try to raise it. However, a value other than 0 will
        affect other parts, most importantly you wont get reliable total_impulse
        readings from the Arbiter object in collsion callbacks!
        '''
        global current_spaces
        current_spaces.append(self)
        self._space = cpSpaceNew()
        self._space.iterations = iterations
        self._static_body = Body()
        self._static_body._body = self._space.staticBody
        self._handlers = {}
        self._default_handlers = None
        self._post_step_callbacks = {}
        self._shapes = {}
        self._static_shapes = {}
        self._bodies = []
        self._constraints = []
        self.set_default_collision_handler()
        #self._bodies = set()
        #self._constraints = set()

    def __dealloc__(self):
        self._static_body = None
        cpSpaceSetDefaultCollisionHandler(self._space, NULL, NULL, NULL, NULL, NULL)
        cpSpaceFree(self._space)

    property shapes:
        '''
        A list of the shapes added to this space
        '''
        def __get__(self):
            return dict(self._shapes)

    property static_shapes:
        '''
        A list of the static shapes added to this space
        '''
        def __get__(self):
            return list(self._static_shapes.values())

    property bodies:
        '''
        A list of the bodies added to this space
        '''
        def __get__(self):
            return list(self._bodies)

    property constraints:
        '''
        A list of the constraints added to this space
        '''
        def __get__(self):
            return self._constraints

    property static_body:
        '''
        A convenience static body already added to the space
        '''
        def __get__(self):
            return self._static_body

    property iterations:
        '''
        Number of iterations to use in the impulse solver to solve contacts.
        '''
        def __get__(self):
            return self._space.iterations
        def __set__(self, int iterations):
            self._space.iterations = iterations

    property gravity:
        '''
        Default gravity to supply when integrating rigid body motions.
        '''
        def __get__(self):
            return Vec2d(self._space.gravity.x, self._space.gravity.y)
        def __set__(self, gravity):
            cdef Vec2d vec
            if isinstance(gravity, Vec2d):
                vec = gravity
                self._space.gravity = vec.v
            else:
                self._space.gravity = cpv(gravity[0], gravity[1])

    property damping:
        '''
        Damping rate expressed as the fraction of velocity bodies retain each second.
        '''
        def __get__(self):
            return self._space.damping
        def __set__(self, damping):
            self._space.damping = damping

    property idle_speed_threshold:
        '''
        Speed threshold for a body to be considered idle. The default value of 0
        means to let the space guess a good threshold based on gravity.
        '''
        def __get__(self):
            return self._space.idleSpeedThreshold
        def __set__(self, idle_speed_threshold):
            self._space.idleSpeedThreshold = idle_speed_threshold

    property sleep_time_threshold:
        '''
        Time a group of bodies must remain idle in order to fall asleep.
        '''
        def __get__(self):
            return self._space.sleepTimeThreshold
        def __set__(self, sleep_time_threshold):
            self._space.sleepTimeThreshold = sleep_time_threshold

    property collision_slop:
        '''
        Amount of allowed penetration.
        '''
        def __get__(self):
            return self._space.collisionSlop
        def __set__(self, collision_slop):
            self._space.collisionSlop = collision_slop

    property collision_bias:
        '''
        Determines how fast overlapping shapes are pushed apart.
        '''
        def __get__(self):
            return self._space.collisionBias
        def __set__(self, collision_bias):
            self._space.collisionBias = collision_bias

    property collision_persistence:
        '''
        Number of frames that contact information should persist.
        '''
        def __get__(self):
            return self._space.collisionPersistence
        def __set__(self, collision_persistence):
            self._space.collisionPersistence = collision_persistence

    property enable_contact_graph:
        '''
        Rebuild the contact graph during each step.
        '''
        def __get__(self):
            return self._space.enableContactGraph
        def __set__(self, enable_contact_graph):
            self._space.enableContactGraph = enable_contact_graph


    def add(self, *objs):
        '''
        Add one or many shapes, bodies or joints to the space
        '''
        for o in objs:
            if isinstance(o, Body):
                if o.is_static:
                    raise Exception('Cannot add a static Body in Space')
                self.add_body(o)
            elif isinstance(o, Shape):
                self.add_shape(o)
            #elif isinstance(o, Constraint):
            #    self.add_constraint(o)
            else:
                for oo in o:
                    self.add(oo)

    def add_static(self, *objs):
        '''
        Add one or many static shapes to the space
        '''
        for o in objs:
            if isinstance(o, Shape):
                self.add_static_shape(o)
            else:
                for oo in o:
                    self.add_static(oo)

    def add_shape(self, Shape shape):
        assert shape._hashid_private not in self._shapes, "shape already added to space"
        self._shapes[shape._hashid_private] = shape
        cpSpaceAddShape(self._space, shape._shape)
        return shape

    def add_static_shape(self, Shape static_shape):
        assert static_shape._hashid_private not in self._static_shapes, "shape already added to space"
        self._static_shapes[static_shape._hashid_private] = static_shape
        cpSpaceAddStaticShape(self._space, static_shape._shape)
        return static_shape

    def add_body(self, Body body):
        assert body not in self._bodies, "body already added to space"
        self._bodies.append(body)
        cpSpaceAddBody(self._space, body._body)
        return body

    def add_constraint(self, constraint):
        assert constraint not in self._constraints, "constraint already added to space"
        self._constraints.add(constraint)
    #    cpSpaceAddConstraint(self._space, constraint._constraint)
        return constraint


    def remove(self, *objs):
        '''
        Remove one or many shapes, bodies or constraints from the space
        '''
        for o in objs:
            if isinstance(o, Body):
                self._remove_body(o)
            elif isinstance(o, Shape):
                self._remove_shape(o)
            #elif isinstance(o, Constraint):
            #    self._remove_constraint(o)
            else:
                for oo in o:
                    self.remove(oo)

    def remove_static(self, *objs):
        '''
        Remove one or many static shapes from the space
        '''
        for o in objs:
            if isinstance(o, Shape):
                self._remove_static_shape(o)
            else:
                for oo in o:
                    self.remove_static(oo)

    def _remove_shape(self, Shape shape):
        del self._shapes[shape._hashid_private]
        cpSpaceRemoveShape(self._space, shape._shape)

    def _remove_static_shape(self, Shape static_shape):
        del self._static_shapes[static_shape._hashid_private]
        cpSpaceRemoveStaticShape(self._space, static_shape._shape)

    def _remove_body(self, Body body):
        self._bodies.remove(body)
        cpSpaceRemoveBody(self._space, body._body)

    def _remove_constraint(self, constraint):
        self._constraints.remove(constraint)
    #    cpSpaceRemoveConstraint(self._space, constraint._constraint)


    cdef object _get_shape(self, cpShape *_shape):
        hashid_private = _shape.hashid_private
        if hashid_private in self._shapes:
            return self._shapes[hashid_private]
        elif hashid_private in self._static_shapes:
            return self._static_shapes[hashid_private]
        return None


    def reindex_static(self):
        '''
        Update the collision detection info for the static shapes in the space.
        You only need to call this if you move one of the static shapes.
        '''
        cpSpaceReindexStatic(self._space)

    def reindex_shape(self, Shape shape):
        '''
        Update the collision detection data for a specific shape in the space.
        '''
        cpSpaceReindexShape(self._space, shape._shape)

    def step(self, dt):
        '''
        Update the space for the given time step. Using a fixed time step is
        highly recommended. Doing so will increase the efficiency of the contact
        persistence, requiring an order of magnitude fewer iterations to resolve
        the collisions in the usual case.
        '''
        cpSpaceStep(self._space, dt)
        for obj, (func, args, kwargs) in self._post_step_callbacks.items():
            func(obj, *args, **kwargs)
        self._post_step_callbacks = {}

    def register_bb_query_func(self, func):
        self._set_py_bb_query_func(func)

    def register_segment_query_func(self, func):
        self._set_py_segment_query_func(func)

    def _set_py_bb_query_func(self, func):
        global handlers
        handlers['bb_query_func'] = func

    def _set_py_segment_query_func(self, func):
        global handlers
        handlers['segment_query_func'] = func

    def space_segment_query(self, start_vect, end_vect, layers=1, group=0):
        cpSpaceSegmentQuery(self._space, cpv(start_vect[0], start_vect[1]), cpv(end_vect[0], end_vect[1]), layers, group,
            _call_space_segment_query_func, NULL)

    def space_bb_query(self, bb, layers=1, group=0):
        cpSpaceBBQuery(self._space, bb._bb, layers, group, 
            _call_space_bb_query_func, NULL)
        
    def point_query_first(self, Vec2d point, layers=1, group=0):
        cdef cpShape* _shape
        _shape = cpSpacePointQueryFirst(self._space, point.v, layers, group)
        if not _shape:
            return None
        else:
            return self._shapes.get(_shape.hashid_private, None) \
                    or self._static_shapes.get(_shape.hashid_private, None)

    cdef void _add_c_collision_handler(self, a, b):
        cpSpaceAddCollisionHandler(self._space, a, b, _call_collision_begin_func, _call_collision_pre_solve_func, _call_collision_post_solve_func, _call_collision_separate_func, NULL)

    def _set_py_collision_handlers(self, a, b, begin, pre_solve, post_solve, separate):
        global handlers
        handlers[(a, b)] = {'begin_func': begin, 'pre_solve_func': pre_solve, 'post_solve_func': post_solve, 'separate_func': separate}

    def add_collision_handler(self, a, b, begin=None, pre_solve=None, post_solve=None, separate=None, *args, **kwargs):
        self._set_py_collision_handlers(a, b, begin, pre_solve, post_solve, separate)

        self._add_c_collision_handler(a, b)
        

    def set_default_collision_handler(self, begin=None, pre_solve=None, post_solve=None, separate=None, *args, **kwargs):
        '''
        Register a default collision handler to be used when no specific
        collision handler is found. If you do nothing, the space will be given a
        default handler that accepts all collisions in begin() and pre_solve()
        and does nothing for the post_solve() and separate() callbacks.

        All the callback have the signature: callback(arbiter, *args, **kwargs), and must return True or False.
        None is assumed to be False.
        '''
        self._default_handlers = (begin, pre_solve, post_solve, separate, args, kwargs)
        cpSpaceSetDefaultCollisionHandler(self._space,
            _collision_begin_func,
            _collision_pre_solve_func,
            _collision_post_solve_func,
            _collision_seperate_func,
            <PyObject *>self)

