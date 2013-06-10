/*
Simulation
- Singleton Class to create WIND and GPS Error Simulation models
*/

#ifndef SIMULATION_H
#define SIMULATION_H

#include "AU_UAV_ROS/standardDefs.h"
#include <AU_UAV_ROS/WindSimulation.h>
#include <AU_UAV_ROS/GPSErrorSimulation.h>

namespace AU_UAV_ROS
{

  class CSimulation {

  public:
    // Singleton Access functions
    static AU_UAV_ROS::CSimulation& GetInstance();

    // Function to update the simulation values
    void UpdateSimulatedValues(double&,double&);

    // 	Get Distance and Bearing between tow lat/longs
    void GetDistanceAndBearing(double lat1,double lat2,
			       double long1,double long2,
			       double& newLat, double& newLong,
			       double& actualBearing,
			       double& bearing,
			       double& distanceToDesitnation);
  private:
    // Constructor
    CSimulation() {
      wind_model = new AU_UAV_ROS::CWindSimulation(gps_error_model);
    };
  
    CSimulation(const CSimulation&);
    CSimulation& operator=(const CSimulation&);
    
    
    // WindSimulation Object
    AU_UAV_ROS::CWindSimulation* wind_model;
    
    // GPS Error Simulation Object
    AU_UAV_ROS::CGPSErrorSimulation gps_error_model;

    // Calculate distance and bearing using Haversines
    void HaversinesCalculation(double lat1,double lat2,
			       double long1,double long2,
			       double& newLat,double& newLong,
			       double& actualBearing,
			       double& bearing,
			       double& distanceToDesitnation);

    // Calculate distance and bearing using Geodetic Datums
    void GeodeticCalculation(double lat1,double lat2,
			     double long1,double long2,
			     double& newLat,double& newLong,
			     double& actualBearing,
			     double& bearing,
			     double& distanceToDesitnation);

    // Utility Function to check the turning radius
    double CheckTurningRadius(const double,double);
  };
}

#endif
