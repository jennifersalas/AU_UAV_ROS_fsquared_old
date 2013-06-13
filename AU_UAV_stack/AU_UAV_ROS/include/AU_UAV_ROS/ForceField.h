/*
Authors: Andrew Cunningham
		 Victoria Wu

Description:
		This file contains definitions of the ForceField object that is required for the F^2 approach
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




/* Description:
 *		This class is the abstract base class from which the specific field type are
 *		derived from.
 *
 */
class ForceField{
public:
	enum FIELD_SHAPE_E{
		FIELD_SHAPE_OVAL,
		FIELD_SHAPE_TRIANGLE
	};

	virtual ~ForceField();

	//Precondition: Coordinates must be within the field
	//Use:
	//		This method will calculate the magnitude of the force by this field
	//		on a point inside this field
	//Params:
	//		positionInField: point at which
	virtual mathVector findFieldForceMagnitude(Coordinates positionInField) =0;


	//Precondition: None
	//Use:
	//		This method will determine whethher or not coordinates are inside
	//		this field
	//Params:
	//		positionInField: coordinate of the plane that will feel the force
	//						 relative to the position of the plane generating the field
	virtual isCoordinatesInMyField(Coordinates positionInField) =0;
};

























#endif
