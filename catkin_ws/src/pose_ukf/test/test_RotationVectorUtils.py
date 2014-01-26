from unittest import TestCase
import numpy
import random as r
import math
from src import RotationVectorUtils

class TestRotationVectorUtils(TestCase):
    identity = numpy.matrix([[0],[0],[0]])
    x = numpy.matrix([[1],[0],[0]])
    y = numpy.matrix([[0],[1],[0]])
    z = numpy.matrix([[0],[0],[1]])
    tolerance = 10**(-8)

    def assertMatrixEqual(self, A, B):
        diff = A-B
        size = diff.transpose()*diff
        self.assertTrue(size < self.tolerance)

    def test_rotateThisByThat_identityRotation(self):
        random = numpy.matrix([[r.random()],[r.random()],[r.random()]])
        result = RotationVectorUtils.rotateThisByThat(random, self.identity)
        self.assertMatrixEqual(random, result)

    def test_rotateThisByThat_simpleRotation(self):
        temp = RotationVectorUtils.rotateThisByThat(self.x, self.z*math.pi/2)
        self.assertMatrixEqual(self.y, temp)

    def test_rotateThisByThat_multipleRotations(self):
        temp = self.x
        temp = RotationVectorUtils.rotateThisByThat(temp, self.z*math.pi/2)
        temp = RotationVectorUtils.rotateThisByThat(temp, self.x*math.pi/2)
        temp = RotationVectorUtils.rotateThisByThat(temp, self.y*math.pi/2)
        self.assertMatrixEqual(temp, self.x)

    def test_norm(self):
        x = r.random()
        y = r.random()
        z = r.random()
        vector = numpy.matrix([[x],[y],[z]])
        norm = RotationVectorUtils.norm(vector)
        self.assertEquals(norm*norm,x*x+y*y+z*z)

    def test_dot(self):
        self.assertMatrixEqual(0, RotationVectorUtils.dot(self.x, self.y))

    def test_cross(self):
        self.assertMatrixEqual(self.z, RotationVectorUtils.cross(self.x,self.y))

    def test_inverse(self):
        vector = numpy.matrix([[r.random()],[r.random()],[r.random()]])
        test = RotationVectorUtils.rotateThisByThat(vector, self.x)
        test = RotationVectorUtils.rotateThisByThat(test, RotationVectorUtils.inverse(self.x))
        self.assertMatrixEqual(vector, test)

    def test_composeRotations_identityRotation(self):
        self.assertMatrixEqual(self.x, RotationVectorUtils.composeRotations(self.x, self.identity))

    def test_composeRotations_multipleRotationsSameDirection(self):
        rotation = RotationVectorUtils.composeRotations(self.x, self.x)
        rotation = RotationVectorUtils.composeRotations(self.x, rotation)
        self.assertMatrixEqual(rotation, 3*self.x)

    def test_composeRotations_inverse(self):
        self.assertMatrixEqual(self.identity,
                               RotationVectorUtils.composeRotations(
                                   self.x,
                                   RotationVectorUtils.inverse(self.x)))