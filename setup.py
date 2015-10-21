import sys
from os import environ
from os.path import dirname, join
from distutils.core import setup
from distutils.extension import Extension
try:
    from Cython.Distutils import build_ext
    have_cython = True
except ImportError:
    have_cython = False


platform = sys.platform
if platform == 'win32':
	cstdarg = '-std=gnu99'
else:
	cstdarg = '-std=c99'
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
        'cymunk/constraint.pxi',
        'cymunk/core.pxi',
        'cymunk/space.pxi',
        'cymunk/shape.pxi',
        'cymunk/body.pxi',
        'cymunk/cymunk.pyx'
        ]
    cmdclass = {'build_ext': build_ext}
else:
    cymunk_files = ['cymunk/cymunk.c']
    cmdclass = {}

ext = Extension('cymunk.cymunk',
    cymunk_files + c_chipmunk_files,
    include_dirs=c_chipmunk_incs,
    extra_compile_args=[cstdarg, '-ffast-math', '-fPIC', '-DCHIPMUNK_FFI'])
 

setup(
    name='cymunk',
    description='Cython bindings for Chipmunk',
    author='Mathieu Virbel and Nicolas Niemczycki',
    author_email='mat@kivy.org',
    cmdclass=cmdclass,
    packages=['cymunk'],
    package_data={'cymunk': ['*.pxd', '*.pxi', 'chipmunk/*.h',
        'chipmunk/constraints/*.h']},
    package_dir={'cymunk': 'cymunk'},
    ext_modules=[ext])
