/*
Authors: Andrew Cunningham
		 Victoria Wu

Description:
		This file contains definitions of the ForceField object that is required for the F^2 approach
		Currently, it is setup to contain variability of field shape, but not variability of force functions

Date: 6/12/13


FUTURE/WORK IN PROGRESS:
		Add in different field configurations and functions as needed

TODO:
	Create variable force functions within the force field class
	Possibly create FieldShape and FieldFunction classes and situate them within
		the ForceField framework

*/


#include "AU_UAV_ROS/vmath.h" 		 //MOVE ME IN
#include "AU_UAV_ROS/pobject.h"
#include "AU_UAV_ROS/standardDefs.h" //contains waypoint struct

#ifndef FORCEFIELD_H
#define FORCEFIELD_H






/* Description:
 *		This class is the abstract base class from which specific field types are
 *		derived.
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
	virtual double findFieldForceMagnitude(Coordinates positionInField) =0;


	//Precondition: None
	//Use:
	//		This method will determine whether or not coordinates are inside
	//		this field
	//Params:
	//		positionInField: coordinate of the plane that will feel the force
	//						 relative to the position of the plane generating the field
	virtual bool isCoordinatesInMyField(Coordinates positionInField) =0;
};


/* Description:
 * 			This class contains the ovoid field shape that was used by the 2012 APF group,
 * 			CURRENTLY USES ONLY ONE FORCE FUNCTION - BIVARIATE NORMAL. SEPERATE SHAPES AND
 * 			FUNCTIONS LATER
 *
 */
class OvalField : public ForceField{
public:
	OvalField();
	double findFieldForceMagnitude(Coordinates positionInField);
	bool isCoordinatesInMyField(Coordinates positionInField, double fieldAngle);

private:
	/* Credit:
	 * 		Hosea Siu and Miriam Figueroa- the 2012 REU group that worked on APFs & flocking (move this later)
	 */
	struct forceVariables
	{
		double maxForce;				// maximum force imposed by one plane on another, except when they are in conflict radius
		// alpha and beta are parameters that define the bivariate normal potential field
		double alpha;
		double beta;
		// these variables define the limits of the independent/swarm leader force function
		double gamma;
		double alphaTop;
		double betaTop;
		double alphaBot;
		double betaBot;
	};

	forceVariables myParams;
};























#endif
