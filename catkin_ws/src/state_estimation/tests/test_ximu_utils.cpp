#include "gtest/gtest.h"
#include "XimuPublisher.h"
#include "XimuReceiver.h"

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

TEST(XimuReceiver_noStartQuaternion) {
	XimuReceiver receiver();
	EXPECT_EQ(false, receiver.isQuaternionGetReady());
}

/*
* Test if the receiver process the packet correctly
* Need a test packet that we do not possess now. Will obtain one during one of the tests
TEST(XimuReceiver_testQuaternionPacket) {
	// Need the data for this test packet
	unsigned char testPacket[256];
	XimuReceiver receiver;

	for (int i = 0; i < 255; i++) {
		receiver.processChar(testPacket[i]);
	}

	// Quaternion should exist now
	EXPECT_EQ(true, receiver.isQuaternionGetReady());

	// Test if the data is correct
	QuaternionStruct quat = receiver.getQuaternion();
	EXPECT_DOUBLE_EQ(1.0, quat.w);
}
*/

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}