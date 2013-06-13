/*
Authors: Andrew Cunningham
		 Victoria Wu

Description:
		This is an implementation of the fsquared algorithm for collision avoidance and detection

Date: 6/12/13

TODO:
Move force header and implementation into the correct folders
Determine waypoint format (for hardware implementation)
Modify Plane object to include field as an enumeration (only use last years for now)




Future:
Situate our algorithm so that it can be simulated
Diversify field types (work on once this implementation is completed and tested


Future Future:
Organize in nodes for hardware


*/


#include "AU_UAV_ROS/vmath.h" 		 //MOVE ME IN
#include "AU_UAV_ROS/pobject.h"
#include "AU_UAV_ROS/standardDefs.h" //contains waypoint struct



#ifndef FSQUARED_H
#define FSQUARED_H

namespace fsqaured{



//**********************************************************************************************
//Forces
//**********************************************************************************************


//Precondition: "Me's" position in relation to enemy's doesn't matter.
//Use: Calculates the force that the enemy's field exerts on me
//Params:
//		me: Plane that is feeling the force
//		enemy: Plane that is exerting the force
//IMPLEMENTATION:
//		
//		calculate field angle from plane methods
//		find coordinates relative enemy
//				
//
//todo:		ac
mathVector calculateRepulsiveForce(PlaneObject &me, PlaneObject &enemy);


//Precondition: Assume valid waypoint
//Use: Calculates the force that the waypoint exerts on me
//Params:
//		me: Plane that is feeling the force
//		goal_wp: The waypoint that the plane would like to reach
//todo:		ac
mathVector calculateAttractiveForce(PlaneObject &me, AU_UAV_ROS::waypoint goal_wp);



//**********************************************************************************************
//Fields
//**********************************************************************************************

//Precondition: Assume valid planes
//Use: Find "me's" position from enemy's POV
//Params:
//		me: Plane that is potentially in enemy's field
//		enemy: Plane that is producing the field
//		fieldAngle: fieldAngle is the angle between the bearing of the plane generating the force to the location
//		of pobj1, and aAngle is the angle between the bearing of pobj1 and the location of its destination?
//Returns:	Coordinates of "me" from the enemy's POV. 
//TODO
//		Make the function return a coordinate
//who:		vw
Coordinates findRelativePosition(PlaneObject &me, PlaneObject &enemy, double fieldAngle);



//Precondition: Assume valid planes
//Use: 		Given a pair of coordinates of "me", will see if "me" is located inside
//		enemy field.
//Params:
//		enemy: Plane that is producing the field
//		locationOfMe: Coordinate location of "me" from the POV of the enemy. 
//				Calculated by findRelativePosition()
//TODO:
//		make and associate with field object
//who:		ac
bool inEnemyField(PlaneObject &enemy, Coordinates locationOfMe);



//Precondition: Assume valid planes
//Use: Calculates the relative angle needed to find the relative position of "me" to enemy
//Params:
//		me: Plane that is potentially in enemy's field
//		enemy: Plane that is producing the field
//who:		vw
double findFieldAngle(PlaneObject &me, PlaneObject &enemy)


//**********************************************************************************************
//Waypoint generation
//**********************************************************************************************



//Precondition: motionVector has been normalized
//Use: Converts from directional vector to a waypoint for a physical plane
//	   waypoint should be far enough away so that the plane will travel in
//	   the correct direction but not reach the waypoint
//Params:
//		motionVector: Vector describing the direction of motion that the plane should take
//tood:		vw
AU_UAV_ROS::waypoint motionVectorToWaypoint_Hardware(mathVector motionVector);


//Precondition: motionVector has been normalized
//Use: Converts from directional vector to a waypoint for a simulated plane
//	   The waypoint represents the position of the simulated UAV in the next
//	   time step
//Params:
//		motionVector: Vector describing the direction of motion that the plane should take
//todo: 	vw

AU_UAV_ROS::waypoint motionVectorToWaypoint_Software(mathVector motionVector);


}



#endif
