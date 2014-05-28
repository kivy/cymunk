'''
Example code using Kivy and cymunk with the updated cymunk code in this github
This code will retrieve vertices from cymunk and graph it using Kivy

-The cymunk code was updated to properly retrieve vertices of Polygon objects (see shape.pxi and core.pxi)
-A video of what this code does can be found here: http://instagram.com/p/oh9zgPInUh/ 

'''
import cymunk as cy
from os.path import dirname, join
from kivy.clock import Clock
from kivy.core.window import Window
from kivy.app import App
from kivy.graphics import Color, Rectangle, Quad
from kivy.uix.widget import Widget
from kivy.properties import DictProperty, ListProperty
from kivy.core.image import Image
from random import random

WX = Window.size[0]
WY = Window.size[1]

VERSION = '1.0.0'

class Playground(Widget):

    cbounds = ListProperty([])
    cmap = DictProperty({})
    blist = ListProperty([])

    def __init__(self, **kwargs):
        self._hue = 0
        super(Playground, self).__init__(**kwargs)

        # create a cymunk space with gravity
        self.init_physics()
        
        # update bounds with good positions ie edges of our widget
        self.bind(size=self.update_bounds, pos=self.update_bounds)

        # load in circle picture
        self.texture = Image(join(dirname(__file__), 'circle.png'), mipmap=True).texture

        # create a block
        self.init_block()

        # add motion to our game
        Clock.schedule_interval(self.step, 1 / 60.)

    def init_block(self):

        # Add block
        xsize = 0.1*WX
        ysize = 0.1*WY
        xpos = 0.3*WX
        ypos = 0.3*WY

        # create a new rigid body for our space
        building_body = cy.Body(100,cy.moment_for_box(100,xsize*2,ysize*2)) # FIXME
        building_body.velocity = (0,0)
        building_body.position = (xpos,ysize)
        
        # set up block vertices
        x, y = WX * .1, WY * .1
        vs = [(-x, -y), (-x, y), (x, y), (x, -y+50)]

        # Create the shape for our body
        building_poly = cy.Poly(building_body,vs)
        building_poly.collision_type = 1
        building_poly.friction = 0
        print 'building has type:',building_poly.collision_type
        
        # add new block to our space
        self.space.add(building_body, building_poly)
        
        # update graphics for our new block
        with self.canvas.before:
            self._hue = (self._hue + 0.01) % 1
            color = Color(self._hue, 1, 1, mode='hsv')
            verts = [ i[j] for i in vs for j in range(len(i)) ]
            rect = Quad(points = verts)
        
        # add reference to our poly and rect figure
        self.cmap[building_poly] = rect
        
        # add to list to keep track of all objects on screen
        self.blist.append((building_body, building_poly))


    def init_physics(self):
        # create the space for physics simulation
        self.space = space = cy.Space()
        space.gravity = (0, -0.5*Window.size[1])
        
        # Number of iterations to use in the impulse solver to solve contacts
        space.iterations = 10
        
        # time a group of bodies must remain idle in roder to fall asleep
        space.sleep_time_threshold = 0.5

        # amount of allowed penetration
        # used to reduce oscillating contacts and keep the collision cache warm
        space.collision_slop = 20
        

        # create 4 segments that will act as a bounds
        for x in xrange(4): 
            seg = cy.Segment(space.static_body,cy.Vec2d(0, 0), cy.Vec2d(WX, 0), 0)
            seg.elasticity = 0.0
            seg.friction = 1.0
            self.cbounds.append(seg)
            space.add_static(seg)


    def draw_collision(self, _space, _arb):
        if _arb.is_first_contact == 1:
            print 'COLLISION BTWN SHAPES:',_arb.shapes


    def update_bounds(self, *largs):
        assert(len(self.cbounds) == 4)
        a, b, c, d = self.cbounds
        x0, y0 = self.pos
        x1 = self.right
        y1 = self.top
        print 'SIZE OF BOX =',x0,y0,x1,y1

        self.space.remove_static(a)
        self.space.remove_static(b)
        self.space.remove_static(c)
        self.space.remove_static(d)
        a.a = (x0, y0)
        a.b = (x1, y0)
        b.a = (x1, y0)
        b.b = (x1, y1)
        c.a = (x1, y1)
        c.b = (x0, y1)
        d.a = (x0, y1)
        d.b = (x0, y0)
        self.space.add_static(a)
        self.space.add_static(b)
        self.space.add_static(c)
        self.space.add_static(d)


    def update_objects(self):
        for poly, obj in self.cmap.iteritems():
            # sync canvas figure up with body in our space
            #p = body.position
            #print body.position

            try: # update polygon
                a = poly.get_vertices()
                verts = []
                verts = [ i[j] for i in a for j in range(len(i)) ]
                #print 'NORM CODE:',verts
                obj.points = verts
            except: # update circle
                p = poly.position
                radius,color,rect = obj
                rect.pos = p.x - radius, p.y - radius


    def step(self, dt):
        self.space.step(dt)
        self.update_objects()


    def add_circle(self, x, y, radius):

        # create a falling circle
        body = cy.Body(10, cy.moment_for_circle(10,0,radius))
        body.velocity = 0.1*(x-Window.size[0]*0.5),0
        body.position = x, y
        body.angular_velocity_limit = 0
        circle = cy.Circle(body, radius)
        circle.collision_type = 1.0
        print 'dis circle has type=',circle.collision_type
        
        circle.elasticity = 0.001
        circle.friction = 2
        self.space.add(body, circle)

        with self.canvas.before:
            self._hue = (self._hue + 0.01) % 1
            color = Color(self._hue, 1, 1, mode='hsv')
            rect = Rectangle(
                texture=self.texture,
                pos=(x - radius, y - radius),
                size=(radius * 2, radius * 2))
        self.cmap[body] = (radius, color, rect)

        # remove the oldest one
        self.blist.append((body, circle))
        if len(self.blist) > 200:
            body, circle = self.blist.pop(0)
            self.space.remove(body)
            self.space.remove(circle)
            radius, color, rect = self.cmap.pop(body)
            self.canvas.before.remove(color)
            self.canvas.before.remove(rect)


    def on_touch_down(self, touch):
        self.add_circle(touch.x, touch.y, 0.05*Window.size[0] + random() * 0.05*Window.size[0])
        return True


class PhysicsApp(App):
    def build(self):
        return Playground()

if __name__ == '__main__':
    PhysicsApp().run()
