#include <gtest/gtest.h>
#include "controls.h"
//#include <cmath>

TEST(Practice,equals)
{
	EXPECT_EQ(2,2);
}

/*TEST(LimitCheck,output)
{
	EXPECT_EQ(0,output_limit_check(8,-5,5,"one"));
	EXPECT_EQ(0,output_limit_check(-8,-5,5,"one"));
	EXPECT_EQ(2,output_limit_check(2,-5,5,"one"));
}*/


/*TEST(LimitCheck,saturation)
{
	char *c1="one";

	EXPECT_EQ(5,saturate(8,5,c1));
	EXPECT_EQ(-5,saturate(-8,5,c1));
	EXPECT_EQ(2,saturate(2,5,c1));
	EXPECT_EQ(-2,saturate(-2,5,c1));
}*//*

TEST(LimitCheck,thrustlimit)
{
	EXPECT_EQ(0,output_limit_check(8,-5,5,"one", "one"));
	EXPECT_EQ(0,output_limit_check(-8,-5,5,"one", "one"));
	EXPECT_EQ(2,output_limit_check(2,-5,5,"one", "one"));
}*/

TEST(transform, pitch)
{
	double theta = 0.1;
	double tol = 1e-10;
	tf::Quaternion q(0,sin(theta/2),0,cos(theta/2));
	tf::Matrix3x3 m(q); //convert quaternion to matrix

	double roll, pitch, yaw;
	m.getEulerYPR(yaw, pitch, roll);
	
	EXPECT_NEAR(0, roll, tol);
	EXPECT_NEAR(theta, pitch, tol);
	EXPECT_NEAR(0, yaw, tol);
}

TEST(transform, roll)
{
	double theta = 0.1;
	double tol = 1e-10;
	tf::Quaternion q(sin(theta/2),0,0,cos(theta/2));
	tf::Matrix3x3 m(q); //convert quaternion to matrix

	double roll, pitch, yaw;
	m.getEulerYPR(yaw, pitch, roll);
	
	EXPECT_NEAR(theta, roll, tol);
	EXPECT_NEAR(0, pitch, tol);
	EXPECT_NEAR(0, yaw, tol);
}

TEST(transform, yaw)
{
	double theta = 0.1;
	double tol = 1e-10;
	tf::Quaternion q(0,0,sin(theta/2),cos(theta/2));
	tf::Matrix3x3 m(q); //convert quaternion to matrix

	double roll, pitch, yaw;
	m.getEulerYPR(yaw, pitch, roll);
	
	EXPECT_NEAR(0, roll, tol);
	EXPECT_NEAR(0, pitch, tol);
	EXPECT_NEAR(theta, yaw, tol);
}

int main(int argc, char **argv){
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
