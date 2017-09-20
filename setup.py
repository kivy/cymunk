import sys
from os import environ
from os.path import dirname, join, isfile

if environ.get('CYMUNK_USE_SETUPTOOLS'):
    from setuptools import setup, Extension
    print('Using setuptools')
else:
    from distutils.core import setup
    from distutils.extension import Extension
    print('Using distutils')

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
        join(dirname(__file__), 'cymunk'),
        join(dirname(__file__), 'cymunk', 'chipmunk'),
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


c_defines = {
    'CP_LAYERS_TYPE':           'uint32_t',
    'CP_GROUP_TYPE':            'uint32_t',
    'CP_COLLISION_TYPE_TYPE':   'uint32_t',
    'CHIPMUNK_FFI':             1
    }

# Bake our defines into the code.
_defines = "#ifndef CONFIG_H_\n#define CONFIG_H_\n"
_defines += "\n".join('#define %s %s' % kv for kv in c_defines.iteritems())
_defines += "\n#endif /* CONFIG_H */"
# Check for changes to avoid rebuilding the code if not required.
need_config_rebuild = True
_config_path = join(dirname(__file__), 'cymunk', 'config.h')
if isfile(_config_path):
    with open(_config_path, 'rb') as fd:
        old = fd.read()
    if _defines == old:
        need_config_rebuild = False
if need_config_rebuild:        
    print('Generating config.h')
    with open(_config_path, 'w') as fd:
        fd.write(_defines)

if have_cython:
    cymunk_files = [
        'cymunk/constraint.pxi',
        'cymunk/core.pxi',
        'cymunk/space.pxi',
        'cymunk/shape.pxi',
        'cymunk/body.pxi',
        'cymunk/cymunk.pyx',
        ]
    cmdclass = {'build_ext': build_ext}
else:
    cymunk_files = ['cymunk/cymunk.c']
    cmdclass = {}

ext = Extension('cymunk.cymunk',
    cymunk_files + c_chipmunk_files,
    include_dirs=c_chipmunk_incs,
    extra_compile_args=[cstdarg, '-ffast-math', '-fPIC'],
    depends=['cymunk/config.h'])
 

setup(
    name='cymunk',
    description='Cython bindings for Chipmunk',
    author='Mathieu Virbel and Nicolas Niemczycki',
    author_email='mat@kivy.org',
    cmdclass=cmdclass,
    packages=['cymunk'],
    package_data={'cymunk': ['config.h', '*.pxd', '*.pxi', 'chipmunk/*.h',
        'chipmunk/constraints/*.h']},
    #data_files=[('cymunk/chipmunk', 'cymunk/Chipmunk-Physics/include/chipmunk/*')],
    package_dir={'cymunk': 'cymunk'},
    ext_modules=[ext],
    version='0.0.0.dev0'
)
