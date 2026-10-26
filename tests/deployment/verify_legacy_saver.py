#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run with system Python 2.7 on Ubuntu 18.04; no ROS master required."""
from __future__ import print_function
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), 'tools', 'pgm_saver'))
import save_continuous_map as module


class FakeROS(object):
    def init_node(self, *args, **kwargs): pass
    def Subscriber(self, *args, **kwargs): pass
    def loginfo(self, *args, **kwargs): pass
    def spin(self): pass
    def signal_shutdown(self, *args): pass


class Record(object):
    def __init__(self, **kwargs): self.__dict__.update(kwargs)


root = tempfile.mkdtemp()
try:
    module.rospy = FakeROS()
    node = module.ContinuousMapSaver(os.path.join(root, 'map'))
    message = Record(data=[-1, 100, 0, 50], info=Record(
        width=2, height=2, resolution=0.2,
        origin=Record(position=Record(x=12, y=-5),
                      orientation=Record(w=1, x=0, y=0, z=0))))
    node.map_callback(message)
    with open(os.path.join(root, 'map.pgm'), 'rb') as stream:
        assert stream.read() == b'P5\n2 2\n255\n\xfe\x7f\xcd\x00'
    with open(os.path.join(root, 'map.yaml')) as stream:
        metadata = stream.read()
    assert 'image: "map.pgm"' in metadata
    assert 'origin: [12, -5, 0.0]' in metadata
    print('Legacy saver passed on Python {}'.format(sys.version.split()[0]))
finally:
    shutil.rmtree(root)
