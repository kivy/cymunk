import sys
from os.path import join
from setuptools import setup, Extension

platform = sys.platform
cstdarg = '-std=gnu99' if platform == 'win32' else '-std=c99'
c_chipmunk_root = join('cymunk', 'Chipmunk-Physics')
c_chipmunk_src = join(c_chipmunk_root, 'src')
c_chipmunk_incs = [
    join(c_chipmunk_root, 'include'),
    join(c_chipmunk_root, 'include', 'chipmunk')
]
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

extra_compile_args = [cstdarg, '-ffast-math', '-fPIC', '-DCHIPMUNK_FFI']

if platform == 'darwin':
    extra_compile_args += ['-Wno-implicit-function-declaration', '-Wno-incompatible-function-pointer-types']
elif platform == 'linux':
    extra_compile_args += ['-Wno-error=incompatible-pointer-types', '-Wno-error=implicit-function-declaration']

ext = Extension(
    'cymunk.cymunk',
    ['cymunk/cymunk.pyx'] + c_chipmunk_files,
    include_dirs=c_chipmunk_incs,
    extra_compile_args=extra_compile_args,
)
 

setup(
    name='cymunk',
    packages=['cymunk'],
    package_data={'cymunk': ['*.pxd', '*.pxi', 'chipmunk/*.h',
                             'chipmunk/constraints/*.h']},
    package_dir={'cymunk': 'cymunk'},
    ext_modules=[ext],
    version='0.0.0.dev0',
)
