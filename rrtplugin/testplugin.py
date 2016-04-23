#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/rrtplugin')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    RRTModule = RaveCreateModule(env,'RRTModule')
    print RRTModule.SendCommand('RRTConnectCommand')
finally:
    RaveDestroy()
