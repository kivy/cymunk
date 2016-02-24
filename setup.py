import sys
from os.path import dirname, join
from setuptools import setup, find_packages, Extension

if sys.platform == 'win32':
    compile_args = ['-std=gnu99', '-ffast-math', '-fPIC', '-DCHIPMUNK_FFI']
    libraries = ['opengl32', 'glu32','glew32']
else:
    compile_args = ['-std=c99', '-ffast-math', '-fPIC', '-DCHIPMUNK_FFI', '-w']
    libraries = []

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

setup(
    name='cymunk',
    description='Cython bindings for Chipmunk',
    author='Mathieu Virbel and Nicolas Niemczycki',
    author_email='mat@kivy.org',
    packages=find_packages(),
    package_data={
        '': ['*.pxd', '*.so', '*.pxi', 'chipmunk/*.h', 'chipmunk/constraints/*.h']
    },
    setup_requires=['cython', 'setuptools.autocythonize'],
    auto_cythonize={
        "compile_args": compile_args,
        "libraries": libraries,
        "includes": c_chipmunk_incs,
        "extra_sources": c_chipmunk_files
    }
)

'''

'''


