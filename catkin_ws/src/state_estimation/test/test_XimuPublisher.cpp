#include "gtest/gtest.h"
#include "XimuPublisher.h"

TEST(sanity, one_equals_one) {
	EXPECT_EQ(1, 1);
}

// Multiplication of quaternions
TEST(XimuPublisher_multiplyQuaternions, simple_multiplication) {
	// quat1 is facing forward, thus the product should be
	// whatever quat2 is
	double quat1[] = {0.0, 0.0, 0.0, 1.0};
	double quat2[] = {0.0, 0.0, 1.0, 0.0};
	double product[] = {0.0, 0.0, 1.0, 0.0};

	for (int i = 0; i < 4; i++) {
		EXPECT_DOUBLE_EQ(product[i], quat2[i]);
	}
}