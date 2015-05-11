Cymunk
======

Cymunk is a cython wrapper for [Chipmunk Physics](https://chipmunk-physics.net/)  
Some code is ported from [pymunk](https://github.com/viblo/pymunk) and much of the API is compatable  

Cymunk should be significantly faster than pymunk and run on android with [p4a](https://github.com/kivy/python-for-android) - but it is currently less feature complete. 

Check out the examples folder, [cymunk docs](http://cymunk.readthedocs.org/en/latest/) and [pymunk docs](https://pymunk.readthedocs.org/en/latest/)  

To build for windows you should use MinGW - [setup](http://stackoverflow.com/a/5051281/445831) or ```python setup.py build --compiler=mingw32```
