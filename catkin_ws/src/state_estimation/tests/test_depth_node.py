import depth_node

class TestDepthNode(TestCase):

    def setup(self):
        pass

    def test_sanity(self):
        self.assertEqual(4, 2+2, "2 + 2 != 4")

    def test_filter(self):
        for i in range(0, 20):
            current_depth, depth = depth_filter(1.0)

            if i > 10:
                self.assertEqual(1.0, depth)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('test_state_estimation', 'test_depth_node', TestDepthNode)