/*
Authors: Andrew Cunningham
		 Victoria Wu

Description:
		This is an implementation of the fsquared algorithm for collision avoidance and detection.
		For a description of each of these functions refer to Fsquared.h, comments in this file
		will be limited to comments on implementation


Date: 6/13/13


*/

#include "AU_UAV_ROS/Fsquared.h"
#include "AU_UAV_ROS/planeObject.h"
#define ATTRACTIVE_FORCE 100

//-----------------------------------------
//Fields
//-----------------------------------------

/*
 * findFieldAngle
 * Preconditions:	assumes valid planes
 * params:		me: plane that is potentially in enemy's field
 * 			enemy: plane that is producing field
 * returns:		field angle - angle between enemy's bearing and my location.
 * 				0  < x < 180  = to enemy's right
 * 				-180< x < 0= to enemy's left 
 *
 * note: Different from AU_UAV_ROS::PlaneObject::findAngle(). FindAngle finds
 * the angle between the relative position vector from one plane to another and the
 * x axis in a global, absolute x/y coordinate system based on latitude and
 * longitude.
 */
double fsquared::findFieldAngle(AU_UAV_ROS::PlaneObject& me, AU_UAV_ROS::PlaneObject &enemy)	{

	//Make two vectors - one representing bearing of enemy plane
	//		     the other representing relative position from
	//		     enemy to me
	AU_UAV_ROS::mathVector enemyBearing(1, toCartesian(enemy.getCurrentBearing())); 
	AU_UAV_ROS::mathVector positionToMe(1, enemy.findAngle(me));


	//Find angle between two vectors
	return enemyBearing.findAngleBetween(positionToMe); 
}

/*
 *Precondition: Assume valid planes
 *Use: Find "me's" position from enemy's POV
 *Params:
 *		me: Plane that is potentially in enemy's field
 *		enemy: Plane that is producing the field
 *Returns:	relativeCoordinates in meters of "me" from the enemy's POV, where enemy's bearing is towards the positive y axis.
 *Implementation:
 *			
 *who:		vw
*/

fsquared::relativeCoordinates fsquared::findRelativePosition(AU_UAV_ROS::PlaneObject &me, AU_UAV_ROS::PlaneObject &enemy )	{

	fsquared::relativeCoordinates loc;
	
	double distance = enemy.findDistance(me);	
	double fieldAngle = fsquared::findFieldAngle(me, enemy);
	
	//Find Y axis coordinate (in front or behind enemey)
	loc.y = cos(fieldAngle*PI/180.0)*distance;

	//Find X Axis coordinate (to the left or right)
	loc.x = sin(fieldAngle*PI/180.0)*distance;

	return loc;
}


//-----------------------------------------
//Waypoint Generation
//-----------------------------------------

/*
 *Precondition: Valid waypoint for me_loc 
 *Use: Converts from desired angle heading to a waypoint. Distance to generated waypoint dependent on previously defined scalar WP_GEN_SCALAR. 
 *Params:
 *		motionAngle: angle between [0,360), CCW from positive x axis (longitude axis)
 *		me_coor: "me's" current location 
 *tood:		vw
 */
AU_UAV_ROS::waypoint motionVectorToWaypoint(double angle, AU_UAV_ROS::waypoint me_loc) {
	AU_UAV_ROS::waypoint dest_wp;

	//Find relative offset for new waypoint.
	double x_delta_meters = WP_GEN_SCALAR*cos(angle*PI/180.0); 
	double y_delta_meters = WP_GEN_SCALAR*sin(angle*PI/180.0); 

	//Calculate new waypoint 
	double dest_wp_long= me_loc.longitude+ (x_delta_meters*METERS_TO_DELTA_LON);
	double dest_wp_lat= me_loc.latitude+ (y_delta_meters*METERS_TO_DELTA_LAT);
	dest_wp.longitude = dest_wp_long;
	dest_wp.latitude = dest_wp_lat;	
	dest_wp.altitude = me_loc.altitude;
	return dest_wp;	
}

