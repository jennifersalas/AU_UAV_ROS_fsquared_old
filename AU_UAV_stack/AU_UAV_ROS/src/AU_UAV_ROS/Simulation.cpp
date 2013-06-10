#include <AU_UAV_ROS/Simulation.h>
#include <cstdlib>
#include <cstdio>

#include <GeographicLib/include/Geodesic.hpp>
#include <GeographicLib/GeodesicLine.hpp>
#include <GeographicLib/DMS.hpp>

using namespace std;

#define USE_HAVERSINES
#include "ros/package.h"

#include "AU_UAV_ROS/SimulatedPlane.h"

// Function to create the instance obejct
AU_UAV_ROS::CSimulation& AU_UAV_ROS::CSimulation::GetInstance() 
{
  static CSimulation singleton;
  return singleton;
}


// Core function that will update the speed and bearing based on WIND and GPS simulation
void AU_UAV_ROS::CSimulation::UpdateSimulatedValues(double& bearing, double& speed)
{
  // Call the wind model function to update the values to reflect the wind effect
  // This functionw will call the gps error function which will add additional gps related modification
  wind_model->calculate_wind_effect(bearing,speed);
}



// Function to calculate distnace and bearing between two waypoints
void AU_UAV_ROS::CSimulation::GetDistanceAndBearing(double lat1,double lat2,
						    double long1,double long2,
						    double& newLat, double& newLong,
						    double& actualBearing,
						    double& bearing,
						    double& distanceToDestination)
{

  // Input all values are in radians
   FILE *fp;
   fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/scores/distance.calc").c_str(), "a");
   fprintf(fp, "Input lat(%f) long(%f) lat2(%f) long2(%f) actualBearing(%f) bearing(%f) \n",
	   lat1,long1,lat2,long2
	   ,actualBearing,bearing);
#ifdef USE_HAVERSINES
   double newLat1,newLong1,actualBearing1=actualBearing,bearing1=bearing,distanceToDestination1=distanceToDestination;
   fprintf(fp, "Haversines lat(%f) long(%f) actualBearing(%f) bearing(%f) distnace(%f)\n",
	   newLat,newLong,actualBearing,bearing,distanceToDestination);	
   HaversinesCalculation( lat1, lat2,
			  long1, long2,
			  newLat,newLong,
			  actualBearing,
			  bearing,
			  distanceToDestination);
   fprintf(fp, "Haversines lat(%f) long(%f) actualBearing(%f) bearing(%f) distnace(%f)\n",
	   newLat,newLong,actualBearing,bearing,distanceToDestination);	
   
   //close the file
   fclose(fp);
#else
   fprintf(fp, "Geographic lat(%f) long(%f) actualBearing(%f) bearing(%f) distnace(%f)\n",
	   newLat,newLong,actualBearing,bearing,distanceToDestination);
   GeodeticCalculation( lat1, lat2,
			long1, long2,
			newLat,newLong,
			actualBearing,
			bearing,
			distanceToDestination);
   fprintf(fp, "Geographic lat(%f) long(%f) actualBearing(%f) bearing(%f) distnace(%f)\n",
	   newLat,newLong,actualBearing,bearing,distanceToDestination);
#endif

}

double AU_UAV_ROS::CSimulation::CheckTurningRadius(const double actualBearing,double bearing)
{
  //calculate the real bearing based on our maximum angle change
  //first create a temporary ebearing that is the same as bearing but at a different numerical value
  FILE *fp;
  fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/scores/turning.calc").c_str(), "a");
  fprintf(fp, "turning1 actualBearing(%f) Bearing(%f) \n",
	  actualBearing,bearing);
  double tempBearing = -1000;
  if((bearing) < 0)
    {
      tempBearing = bearing + 360;
    }
  else
    {
      tempBearing = bearing - 360;
    }
  
  double diff1 = abs(actualBearing - bearing);
  double diff2 = abs(actualBearing - tempBearing);
  
  //check for easy to calculate values first
  if(diff1 < MAXIMUM_TURNING_ANGLE || diff2 < MAXIMUM_TURNING_ANGLE)
    {
      //the difference is less than our maximum angle, set it to the bearing
      bearing = bearing;
    }
  else
    {
      //we have a larger difference than we can turn, so turn our maximum
      double mod;
      if(diff1 < diff2)
	{
	  if(bearing > actualBearing) mod = MAXIMUM_TURNING_ANGLE;
	  else mod = 0 - MAXIMUM_TURNING_ANGLE;
	}
      else
	{
	  if(tempBearing > actualBearing) mod = MAXIMUM_TURNING_ANGLE;
	  else mod = 0 - MAXIMUM_TURNING_ANGLE;
	}
      
      //add our mod, either +22.5 or -22.5
      bearing = actualBearing + mod;
      
      //tweak the value to keep it between -180 and 180
      if(bearing > 180) bearing = bearing - 360;
      if(bearing <= -180) bearing = bearing + 360;
      
    }
  fprintf(fp, "turning2 actualBearing(%f) Bearing(%f) \n",
	  actualBearing,bearing);
  fclose(fp);
  return bearing*DEGREES_TO_RADIANS;
}


// Calculate distance and bearing using Haversines
void AU_UAV_ROS::CSimulation::HaversinesCalculation(double lat1,double lat2,
						    double long1,double long2,
						    double& newLat,double& newLong,
						    double& actualBearing,
						    double& bearing,
						    double& distanceToDestination)
{
  double deltaLat = lat2 - lat1;
  double deltaLong = long2 - long1;	
  //calculate distance from current position to destination
  double a = pow(sin(deltaLat / 2.0), 2);
  a = a + cos(lat1)*cos(lat2)*pow(sin(deltaLong/2.0), 2);
  FILE *fp;
  fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/scores/distance.calc").c_str(), "a");
  fprintf(fp, "Haver1 a(%f) deltaLat(%f)  deltaLong(%f) actualBearing(%f) bearing(%f) \n",
	  a,deltaLat,deltaLong,
	  actualBearing,bearing);
  double c = 2.0 * asin(sqrt(a));
  
  // Distance to Destination Returned to the caller
  distanceToDestination = EARTH_RADIUS * c;
  
  //calculate bearing from current position to destination
  double y = sin(deltaLong)*cos(lat2);
  double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltaLong);
  bearing = atan2(y, x);

  fprintf(fp, "Turning:Haver1 actualBearing(%f) bearing(%f) \n",
	  actualBearing,bearing);

  actualBearing = CheckTurningRadius(actualBearing*RADIANS_TO_DEGREES,bearing*RADIANS_TO_DEGREES);

  fprintf(fp, "Turning:Haver1 actualBearing(%f) bearing(%f) \n",
	  actualBearing,bearing);

  fprintf(fp, "Haver1 newLat(%f) newLOng(%f)  actualBearing(%f) bearing(%f) \n",
	  newLat,newLong,
	  actualBearing,bearing);
  
  /*
    Algorithm for updating position:
    1) Estimate new latitude using basic trig and this equation:
    lat2 = lat1 + (MPS_SPEED*cos(bearing))*METERS_TO_LATITUDE
    2) Use law of haversines to find the new longitude
    haversin(c) = haversin(a-b) + sin(a)*sin(b)*haversin(C)
    where haversin(x) = (sin(x/2.0))^2
    where c = MPS_SPEED/EARTH_RADIUS (radians)
    where a = 90 - lat1 (degrees)
    where b = 90 - lat2 (degrees)
    where C = the change in longitude, what we are solving for
    
    C = 2.0 * arcsin(sqrt((haversin(c) - haversin(a-b))/(sin(a)*sin(b))))
  */  

  //1) Estimate new latitude using basic trig and this equation
  double speed = MPS_SPEED ;
  // Apply Environment and GPS effects
  AU_UAV_ROS::CSimulation::GetInstance().UpdateSimulatedValues(actualBearing,speed);
  fprintf(fp, "Haver2 newLat(%f) newLOng(%f)  actualBearing(%f) bearing(%f) \n",
	  newLat,newLong,
	  actualBearing,bearing);
  
  // Distance in one second is numerical equivalen to the speed
  double distance = speed;

  /*
  newLat = lat1 + ((distance )*cos(actualBearing))*(METERS_TO_LATITUDE);
  
  fprintf(fp, "Haver3 newLat(%f) newLOng(%f)  actualBearing(%f) bearing(%f) \n",
	  newLat,newLong,
	  actualBearing,bearing);
  
  //2) Use the law of haversines to find the new longitude
  double temp = pow(sin((distance/EARTH_RADIUS)/2.0), 2);
  //		double temp = 7.69303281*pow(10, -13); //always the same, see above calculation
  fprintf(fp, "Haver3.1 distance(%f) newLat(%f) temp(%f)  actualBearing(%f) bearing(%f) \n",
	  distance,
	  newLat,temp,
	  actualBearing,bearing);
  temp = temp - pow(sin((newLat - lat1)/2.0), 2);
  fprintf(fp, "Haver3.2 newLat(%f) temp(%f)  actualBearing(%f) bearing(%f) \n",
	  newLat,temp,
	  actualBearing,bearing);
  temp = temp / (sin((M_PI/2.0) - lat1)*sin((M_PI/2.0)-newLat));
  */
  newLat =asin(sin(lat1)*cos(distance/EARTH_RADIUS) + cos(lat1)*sin(distance/EARTH_RADIUS)*cos(actualBearing));
  newLong = long1 + atan2(sin(actualBearing)*sin(distance/EARTH_RADIUS)*cos(lat1),cos(distance/EARTH_RADIUS)-sin(lat1)*sin(lat2)) ;
  fprintf(fp, "Haver3.3 newLat(%f) newLong(%f)  actualBearing(%f) bearing(%f) \n",
	  newLat,newLong,
	  actualBearing,bearing);
  // temp = 2.0 * asin(sqrt(temp));

  //depending on bearing, we should be either gaining or losing longitude
  //  if(actualBearing > 0)
  //{
  //  newLong += temp ;
  //}
  //else
  //{
  //  newLong -= temp ;
  //}
  //fprintf(fp, "Haver4 newLat(%f) newLOng(%f)  actualBearing(%f) bearing(%f) \n",
  //	  newLat,newLong,
  //	  actualBearing,bearing);
  //close the file
   fclose(fp);
}

// Calculate distance and bearing using Geodetic Datums
void AU_UAV_ROS::CSimulation::GeodeticCalculation(double lat1,double lat2,
						  double long1,double long2,
						  double& newLat,double& newLong,
						  double& actualBearing,
						  double& bearing,
						  double& distanceToDestination)
{ 
  GeographicLib::Math::real azi1, azi2, s12, m12, a12, M12, M21, S12;
  // Using WGS84 a,f (radius and flattening values)
  GeographicLib::Math::real a, f;
  a = GeographicLib::Constants::WGS84_a<GeographicLib::Math::real>();
  f = GeographicLib::Constants::WGS84_f<GeographicLib::Math::real>();
  const GeographicLib::Geodesic geod(a, f);

  // All calculation done in degrees
  double lat1_deg,lat2_deg,long1_deg,long2_deg;

  lat1_deg = lat1*RADIANS_TO_DEGREES;
  lat2_deg = lat2*RADIANS_TO_DEGREES;
  long1_deg = long1*RADIANS_TO_DEGREES;
  long2_deg = long2*RADIANS_TO_DEGREES;

  // Calculate Bearing 
  a12 = geod.Inverse(lat1_deg, long1_deg, 
		     lat2_deg, long2_deg, 
		     s12, azi1, azi2,
		     m12, M12, M21, S12);

  bearing = azi1*DEGREES_TO_RADIANS;

  // Distance to Destination Returned to the caller
  distanceToDestination = s12;

  FILE *fp;
  fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/scores/distance.calc").c_str(), "a");
  fprintf(fp, "geod1 bearing(%f) distance(%f) \n",
	  bearing,s12);

  // DIfference between course and bearing
  fprintf(fp, "Turning:Geod actualBearing(%f) bearing(%f) \n",
	  actualBearing,bearing);
  actualBearing = CheckTurningRadius(actualBearing*RADIANS_TO_DEGREES,bearing*RADIANS_TO_DEGREES);
  fprintf(fp, "Turning:Geod actualBearing(%f) bearing(%f) \n",
	  actualBearing,bearing);

  double speed = MPS_SPEED ;

  // Apply Environment and GPS effects
  AU_UAV_ROS::CSimulation::GetInstance().UpdateSimulatedValues(actualBearing,speed);

  // Distance in one second is numerical equivalen to the speed
  double distance = speed;

  // Geodesic Line along which the UAV is moving
  GeographicLib::GeodesicLine l;
  double temp_actualBearing = actualBearing*RADIANS_TO_DEGREES;
  l = geod.Line(lat1_deg, long1_deg,temp_actualBearing );
  
  // Obtain next location along geodesic line
  a12 = l.Position(distance, newLat, newLong, temp_actualBearing, m12, M12, M21, S12);       
 
  fprintf(fp, "geod2 newlat(%f) newlong(%f) actualBearing(%f) \n",
	  newLat,newLong,actualBearing);

  newLat = newLat*DEGREES_TO_RADIANS;
  newLong = newLong*DEGREES_TO_RADIANS;

  fclose(fp);
}
