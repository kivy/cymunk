from os import environ
from os.path import dirname, join
from distutils.core import setup
from distutils.extension import Extension
try:
    from Cython.Distutils import build_ext
    have_cython = True
except ImportError:
    have_cython = False

c_chipmunk_root = join(dirname(__file__), 'cymunk', 'Chipmunk-Physics')
c_chipmunk_src = join(c_chipmunk_root, 'src')
c_chipmunk_incs = [join(c_chipmunk_root, 'include'),
        join(c_chipmunk_root, 'include', 'chipmunk')]
c_chipmunk_files = [join(c_chipmunk_src, x) for x in (
    'cpSpatialIndex.c', 'cpSpaceHash.c', 'constraints/cpPivotJoint.c',
    'constraints/cpConstraint.c', 'constraints/cpSlideJoint.c',
    'constraints/cpRotaryLimitJoint.c', 'constraints/cpGrooveJoint.c',
    'constraints/cpGearJoint.c', 'constraints/cpRatchetJoint.c',
    'constraints/cpSimpleMotor.c', 'constraints/cpDampedRotarySpring.c',
    'constraints/cpPinJoint.c', 'constraints/cpDampedSpring.c', 'cpSpaceStep.c',
    'cpArray.c', 'cpArbiter.c', 'cpCollision.c', 'cpBBTree.c', 'cpSweep1D.c',
    'chipmunk.c', 'cpSpaceQuery.c', 'cpBB.c', 'cpShape.c', 'cpSpace.c',
    'cpVect.c', 'cpPolyShape.c', 'cpSpaceComponent.c', 'cpBody.c',
    'cpHashSet.c')]

if have_cython:
    cymunk_files = [
        'cymunk/python/constraint.pxi',
        'cymunk/python/core.pxi',
        'cymunk/python/space.pxi',
        'cymunk/python/shape.pxi',
        'cymunk/python/body.pxi',
        'cymunk/python/cymunk.pyx'
        ]
    cmdclass = {'build_ext': build_ext}
else:
    cymunk_files = ['cymunk/python/cymunk.c']
    cmdclass = {}

ext = Extension('cymunk',
    cymunk_files + c_chipmunk_files,
    include_dirs=c_chipmunk_incs,
    extra_compile_args=['-std=c99', '-ffast-math', '-fPIC', '-DCHIPMUNK_FFI'])
 

if environ.get('READTHEDOCS', None) == 'True':
    ext.pyrex_directives = {'embedsignature': True}

setup(
    name='cymunk',
    description='Cython bindings for Chipmunk',
    author='Mathieu Virbel and Nicolas Niemczycki',
    author_email='mat@kivy.org',
    cmdclass=cmdclass,
    ext_modules=[ext])
