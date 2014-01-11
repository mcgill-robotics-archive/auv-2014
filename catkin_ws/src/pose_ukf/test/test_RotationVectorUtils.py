from unittest import TestCase
import numpy
import random as r
import math
from src import RotationVectorUtils

__author__ = 'mkrogius'


class TestRotationVectorUtils(TestCase):
    identity = numpy.matrix([[0],[0],[0]])
    x = numpy.matrix([[1],[0],[0]])
    y = numpy.matrix([[0],[1],[0]])
    z = numpy.matrix([[0],[0],[1]])


    def test_rotateThisByThat_identityRotation(self):
        random = numpy.matrix([[r.random()],[r.random()],[r.random()]])
        result = RotationVectorUtils.rotateThisByThat(random, self.identity)
        self.assertEquals(random, result)

    def test_rotateThisByThat_simpleRotation(self):
        self.assertEquals(self.y, RotationVectorUtils.rotateThisByThat(self.x, self.z*math.pi/2))

    def test_rotateThisByThat_multipleRotations(self):
        temp = self.x
        temp = RotationVectorUtils.rotateThisByThat(temp, self.z*math.pi/2)
        temp = RotationVectorUtils.rotateThisByThat(temp, self.x*math.pi/2)
        temp = RotationVectorUtils.rotateThisByThat(temp, self.y*math.pi/2)
        self.assertEquals(temp, self.x)

    def test_norm(self):
        x = r.random()
        y = r.random()
        z = r.random()
        vector = numpy.matrix([[x],[y],[z]])
        norm = RotationVectorUtils.norm(vector)
        self.assertEquals(norm*norm,x*x+y*y+z*z)

    def test_dot(self):
        self.assertEquals(0, RotationVectorUtils.dot(self.x, self.y))

    def test_cross(self):
        self.assertEquals(self.z, RotationVectorUtils.cross(self.x,self.y))

    def test_inverse(self):
        vector = numpy.matrix([[r.random()],[r.random()],[r.random()]])
        test = RotationVectorUtils.rotateThisByThat(vector, self.x)
        test = RotationVectorUtils.rotateThisByThat(test, RotationVectorUtils.inverse(self.x))
        self.assertEquals(vector, test)

    def test_composeRotations_identityRotation(self):
        self.assertEquals(self.x, RotationVectorUtils.composeRotations(self.x, self.identity))

    def test_composeRotations_multipleRotationsSameDirection(self):
        self.fail()