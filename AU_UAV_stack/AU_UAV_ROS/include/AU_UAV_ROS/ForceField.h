/*
Authors: Andrew Cunningham
		 Victoria Wu

Description:
		This is an implementation of the ForceField object that is required for the F^2 approach
		(ADD MORE)

Date: 6/12/13

FUTURE/WORK IN PROGRESS:
		Add in different field configurations and functions as needed



*/


#include "AU_UAV_ROS/vmath.h" 		 //MOVE ME IN
#include "AU_UAV_ROS/pobject.h"
#include "AU_UAV_ROS/standardDefs.h" //contains waypoint struct

#ifndef FORCEFIELD_H
#define FORCEFIELD_H




class ForceField{
public:
	enum FIELD_SHAPE_E{
		FIELD_SHAPE_OVAL,
		FIELD_SHAPE_TRIANGLE
	};

	bool isField


private:
	FIELD_SHAPE_E shape;








};

























#endif
