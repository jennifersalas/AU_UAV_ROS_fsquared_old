/*
Authors: Andrew Cunningham
		 Victoria Wu

Description:
		This file contains an implementation of the ForceField object that is required for the F^2 approach
		(ADD MORE)

Date: 6/13/13


*/


#include "AU_UAV_ROS/ForceField.h"


//Construct field to have only one shape and one force function for now
OvalField::OvalField(){
	myParams.maxForce = 4000;
	myParams.alpha = .00129;
	myParams.beta = .000850;
	myParams.gamma = 1500;
	myParams.alphaTop = .5;
	myParams.betaTop = .25;
	myParams.betaBot = 1.6;

}


/*
 * Description:
 */
double OvalField::findFieldForceMagnitude(Coordinates positionInField){
	if(isCoordinatesInMyField(positionInField))
		return forceVars.maxForce * exp(-forceVars.alpha*pow(x,2)-forceVars.beta*pow(y,2));
	else
		return 0;
}
/*
 *
 */
bool OvalField::isCoordinatesInMyField(Coordinates positionInField, double fieldAngle){
	int x = positionInField.x;
	int y = positionInField.y;
	if (fieldAngle > 90 && fieldAngle <270){
		// plane generating the force is behind, therefore use the bottom boundary
		double forceLimit = -sqrt((forceVars.gamma-(forceVars.alphaBot*pow(x,2)))/forceVars.betaBot);
		if (y>forceLimit) return true;
		else return false;
	}
	else{
		// plane generating the force is in front, therefore use the top boundary
		double forceLimit = sqrt((forceVars.gamma-(forceVars.alphaTop*pow(x,2)))/forceVars.betaTop);
		if (y<forceLimit) return true;
		else return false;
	}
}
