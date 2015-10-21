cdef class Body:
    '''
    A rigid body

    Use forces to modify the rigid bodies if possible. This is likely to be the most stable.
    Modifying a body's velocity shouldn't necessarily be avoided, but applying large changes can cause strange results in the simulation. Experiment freely, but be warned.
    Don't modify a body's position every step unless you really know what you are doing. Otherwise you're likely to get the position/velocity badly out of sync.
    '''

    def __cinit__(self, mass=None, moment=None):
        self._data = None
        self._constraints = []
        if mass is None and moment is None:
            self._body = cpBodyNewStatic()
        elif mass is 'INF' and moment is 'INF':
            self._body = cpBodyNew(INFINITY, INFINITY)
        else:
            self._body = cpBodyNew(float(mass), float(moment))
        self.automanaged = 1
        #self._position_callback = None
        #self._velocity_callback = None

    def __dealloc__(self):
        if self.automanaged:
            cpBodyFree(self._body)

    property mass:
        '''
        Mass of the body
        '''
        def __get__(self):
            return self._body.m
        def __set__(self, mass):
            cpBodySetMass(self._body, mass)

    property moment:
        '''
        Moment of the body
        '''
        def __get__(self):
            return self._body.i
        def __set__(self, moment):
            cpBodySetMoment(self._body, moment)

    property speed:
        def __get__(self):
            return cpvlength(self._body.v)

    property angle:
        '''
        The rotation of the body
        '''
        def __get__(self):
            return self._body.a
        def __set__(self, angle):
            cpBodySetAngle(self._body, angle)

    property rotation_vector:
        '''
        The rotation vector of the body as a unit vector
        '''
        def __get__(self):
            rot = self._body.rot
            return Vec2d(rot.x, rot.y)

    property torque:
        def __get__(self):
            return self._body.t
        def __set__(self, t):
            self._body.t = t

    property position:
        def __get__(self):
            p = self._body.p
            return Vec2d(p.x, p.y)
        def __set__(self, pos):
            self._body.p = cpv(pos[0], pos[1])

    property velocity:
        def __get__(self):
            v = self._body.v
            return Vec2d(v.x, v.y)
        def __set__(self, vel):
            self._body.v = cpv(vel[0], vel[1])

    property velocity_limit:
        def __get__(self):
            return self._body.v_limit
        def __set__(self, vel):
            self._body.v_limit = vel

    property angular_velocity:
        def __get__(self):
            return self._body.w
        def __set__(self, w):
            self._body.w = w

    property angular_velocity_limit:
        def __get__(self):
            return self._body.w_limit
        def __set__(self, w):
            self._body.w_limit = w

    property force:
        def __get__(self):
            return self._body.f
        def __set__(self, f):
            self._body.f = cpv(f.x, f.y)

    property data:
        def __get__(self):
            return self._data
        def __set__(self, data):
            self._data = data

    property is_sleeping:
        '''
        Returns true if the body is sleeping.
        '''
        def __get__(self):
            return cpBodyIsSleeping(self._body)

    property is_rogue:
        '''
        Returns true if the body has not been added to a space.
        '''
        def __get__(self):
            return cpBodyIsRogue(self._body)

    property is_static:
        '''
        Returns true if the body is a static body
        '''
        def __get__(self):
            return cpBodyIsStatic(self._body)

    def apply_impulse(self, j, r=(0, 0)):
        '''
        Apply the impulse j to body at a relative offset (important!) r from
        the center of gravity. Both r and j are in world coordinates.
        '''
        cpBodyApplyImpulse(self._body, cpv(j[0], j[1]), cpv(r[0], r[1]))

    def reset_forces(self):
        '''
        Zero both the forces and torques accumulated on body
        '''
        cpBodyResetForces(self._body)

    def apply_force(self, f, r=(0, 0)):
        '''
        Apply (accumulate) the force f on body at a relative offset
        (important!) r from the center of gravity.
        '''
        cpBodyApplyForce(self._body, cpv(f[0], f[1]), cpv(r[0], r[1]))

    def activate(self):
        '''
        Wake up a sleeping or idle body.
        '''
        cpBodyActivate(self._body)

    def sleep(self):
        '''
        Force a body to fall asleep immediately.
        '''
        cpBodySleep(self._body)

    def local_to_world(self, v):
        '''
        Convert body local coordinates to world space coordinates
        '''
        return cpBodyLocal2World(self._body, cpv(v.x, v.y))

    def world_to_local(self, v):
        '''
        Convert world space coordinates to body local coordinates
        '''
        return cpBodyWorld2Local(self._body, cpv(v[0], v[1]))

    #def apply_damped_spring(self, b, anchor1, anchor2, rlen, k, dmp, dt):
    #    cpApplyDampedSpring(self._body, b._body, anchor1, anchor2, rlen, k, dmp, dt)

    #def sleep_with_group(self, body):
    #    cpBodySleepWithGroup(self._body, body._body)

    #def _set_velocity_func(self, func):
    #    def _impl(_, gravity, damping, dt):
    #        return func(self, gravity, damping, dt)

    #    self._velocity_callback = cpBodyVelocityFunc(_impl)
    #    self._bodycontents.velocity_func = self._velocity_callback
    #velocity_func = property(fset=_set_velocity_func)
    #
    # def _set_position_func(self, func):
    #
    # def _set_position_func(self, func):
    #

    #@staticmethod
    #@classmethod
    #def update_velocity(body, gravity, damping, dt):
    #    cpBodyUpdateVelocity(body._body, cpv(gravity.x, gravity.y), damping, dt)

    #@staticmethod
    #@classmethod
    #def update_position(body, dt):
    #    cpBodyUpdatePosition(body._body, dt)

