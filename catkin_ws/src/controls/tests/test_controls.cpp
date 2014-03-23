#include <gtest/gtest.h>
#include <ros/package.h>
#include <../include/thrust_mapper.h>
#include <../include/controls.h>

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


TEST(LimitCheck,saturation)
{
	char *c1="one";

	EXPECT_EQ(5,saturate(8,5,c1));
	EXPECT_EQ(-5,saturate(-8,5,c1));
	EXPECT_EQ(2,saturate(2,5,c1));
	EXPECT_EQ(-2,saturate(-2,5,c1));
}/*

TEST(LimitCheck,thrustlimit)
{
	EXPECT_EQ(0,output_limit_check(8,-5,5,"one", "one"));
	EXPECT_EQ(0,output_limit_check(-8,-5,5,"one", "one"));
	EXPECT_EQ(2,output_limit_check(2,-5,5,"one", "one"));
}*/



int main (int argc, char **argv)
{
	testing::InitGoogleTest(&argc,argv);
	return RUN_ALL_TESTS();
}