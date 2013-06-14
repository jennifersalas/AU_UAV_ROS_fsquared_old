/*
 * fsquared_test
 *
 * let's test some stuff
 * team 1 2013  
 */

#include <gtest/gtest.h>

//what are we testing
#include "AU_UAV_ROS/vmath.h"
#include "AU_UAV_ROS/planeObject.h"
#include "AU_UAV_ROS/Fsquared.h"
namespace	{
class F_Squared_tester: public ::testing::Test	{

	protected:
		F_Squared_tester()	{
			enemy.setCurrentLoc(0,0,0);	
			enemy.setCurrentBearing(0);	//going north	

			northPlane.setCurrentLoc(0, 100, 0);
			northPlane.setCurrentBearing(0);
			eastPlane.setCurrentLoc(100, 0, 0);
			eastPlane.setCurrentBearing(0);
			southPlane.setCurrentLoc(0, -100, 0);
			southPlane.setCurrentBearing(0);
			westPlane.setCurrentLoc(-100,0 , 0);
			westPlane.setCurrentBearing(0);
		}

	AU_UAV_ROS::PlaneObject enemy;
	AU_UAV_ROS::PlaneObject northPlane;
	AU_UAV_ROS::PlaneObject eastPlane;
	AU_UAV_ROS::PlaneObject westPlane;
	AU_UAV_ROS::PlaneObject southPlane;

};


class VectorStuff : public ::testing::Test	{

	protected:
		VectorStuff()	{
			north.setDirection(90); north.setMagnitude(1);
			east.setDirection(0); east.setMagnitude(1);
			south.setDirection(270); south.setMagnitude(1);
			west.setDirection(180); west.setMagnitude(1);
		}
	AU_UAV_ROS::mathVector north;
	AU_UAV_ROS::mathVector east;
	AU_UAV_ROS::mathVector south;
	AU_UAV_ROS::mathVector west;

};


//find angle between two vectors
TEST_F(VectorStuff, findingAngle)	{
	int angleDiff;
	//this greater angle than other when both are [0,360]
	EXPECT_TRUE((angleDiff=south.findAngleBetween(north))==180||angleDiff==-180);
	EXPECT_EQ(90, south.findAngleBetween(west));
	EXPECT_EQ(-90, south.findAngleBetween(east));

	//this smaller angle than other when both are [0,360]
	EXPECT_TRUE((angleDiff=north.findAngleBetween(south))==180||angleDiff==-180);
	EXPECT_EQ(-90, north.findAngleBetween(west));
	EXPECT_EQ(90, north.findAngleBetween(east));

	//sanity check
	EXPECT_EQ(0, south.findAngleBetween(south));	
}



}




int main(int argc, char **argv)	{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

