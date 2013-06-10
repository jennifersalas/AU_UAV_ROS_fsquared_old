#include <AU_UAV_ROS/WindSimulation.h>
#include "AU_UAV_ROS/standardDefs.h"

#include <iostream>
#include <cstdlib>
#include <cstdio>

using namespace std;
#include "ros/package.h"

enum WIND_FLUCTUATION_TYPE {
  CONSTANT,
  SAWTOOTH,
  PERIODIC
};


// Constructor
AU_UAV_ROS::CWindSimulation::CWindSimulation(CGPSErrorSimulation& gps_ref)
  : gps_model(gps_ref)
{
  // Read the Wind Speed from the Environment
  // This was done specifically to avoid changing any regression tests
  // In the future this could be included in a config file

  // Get wind speed and direction from env
  // convert speed to m/s from knots
  user_defined_speed = speed = get_env_value("AUROS_WIND_SPEED")*0.514444;//Convert to m/s

  // Convert direction to radians
  direction = get_env_value ("AUROS_WIND_DIRECTION")*DEGREES_TO_RADIANS;

  // Wind Fluctuation Period
  period = get_env_value ("AUROS_WIND_PERIOD");

  // Wind Fluctuation Type
  fluctuation_type = get_env_value ("AUROS_WIND_TYPE");

  // Initialie the period_counter
  period_counter=0;

  // Initialize the periods_computer
  periods_complete=0;
}


// Function to get value from environment
float AU_UAV_ROS::CWindSimulation::get_env_value(const char* ENV_VARIABLE)
{
  // Temporary variables for error checking
  char * pEnv;
  std::string sEnv;
  
  // Get the value from the environment
  pEnv = getenv (ENV_VARIABLE);
  // Check for value not set
  if (pEnv==NULL) 
    {
    sEnv = "0.0";
    }
  else
    sEnv = pEnv;
  
  // Convert from ASCII to float
  return atof(sEnv.c_str());
}


/* Calculate Wind Effect */
// Calculates the ground track and course based on the current wind spedeespn
/*                       
   calculate_wind_effect(heading, distance)
   - Return course
   - Calculate Ground Speed using formula
   - Calculate Wind Correct Angle (WCA) using formula
   (only applies if TAS > WS)
*/
float AU_UAV_ROS::CWindSimulation::calculate_wind_effect(double& heading,double& distance)
{
  float heading_r = heading;
  double original_heading, original_distance;

  original_heading = heading;
  original_distance = distance;

  // Update the wind speed to reflect time varying nature
  update_wind_speed();

  float wind_corrected_angle = asin((speed/MPS_SPEED)*sin(heading_r - direction));
  float ground_speed = (MPS_SPEED)*cos(wind_corrected_angle) + speed*cos(heading_r-direction);

  float gps_error = gps_model.correction();
  distance = ground_speed + gps_error;
  heading = ( heading_r + wind_corrected_angle);

  FILE* fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/scores/gnuplot.data").c_str(), "a");
  fprintf(fp, "0:%f\n 1:%f\n",speed,gps_error);
  fclose(fp);

  return heading;
}

// Update wind speed based on user defined user defined fluctuation type 
void AU_UAV_ROS::CWindSimulation::update_wind_speed()
{
  // Update the period counter, used for sawtooth and period count
  if(period_counter < period)
    period_counter++;
  else { // If period is reached, reset counter and increment number of periods complete    
    period_counter = 1;
    periods_complete++;
  }


  switch(fluctuation_type)
    {
    case CONSTANT:// No operation as wind is unchanged
      break;
    case SAWTOOTH:
      // Call the sawtooth function to generate wind fluctuation
      update_sawtooth_speed();
      break;
      // Call the periodic function to generate wind fluctuation
    case PERIODIC:
      update_periodic_speed();
      break;
    }
}


/* Update Wind Speed */
// Function to return the speed based on the saw-tooth fluctuation
void AU_UAV_ROS::CWindSimulation::update_sawtooth_speed()
{ 
  float delta = user_defined_speed / (period - 1);
  speed = delta*period_counter;
}

// Periodic Speed, a step function for the speed governed by period
void AU_UAV_ROS::CWindSimulation::update_periodic_speed()
{
  if(periods_complete % 2)// change wind speed to zero for every alternate period
    speed = 0;
  else
    speed = user_defined_speed;
    
}
