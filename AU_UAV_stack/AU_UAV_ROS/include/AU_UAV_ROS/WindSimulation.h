/*
WindSimulation
- Model for the wind simulation
*/

#ifndef WIND_SIMULATION_H
#define WIND_SIMULATION_H


#include <AU_UAV_ROS/GPSErrorSimulation.h>

namespace AU_UAV_ROS
{

  class CWindSimulation {
  public:
    // Constructor
    CWindSimulation(CGPSErrorSimulation&);

    // Calculates the ground track and course based on the current wind speed
    float calculate_wind_effect(double& heading,double& distance);
    
  private:

    // Wind Speed in m/s
    float speed,user_defined_speed;

    // Wind Direction in radians
    float direction;

    // The period for the wind speed fluctuation
    int period; // Value in seconds

    // Period counter, to generate wind fluctuation
    int period_counter;
    int periods_complete;

    // Wind Fluctuation type
    int fluctuation_type; 

    // Function to return the speed based on the saw-tooth fluctuation
    void update_sawtooth_speed();

    // Constant Speed, no varition in speed so the original valur is returned
    void constant_speed() { /* dummhy function */}

    // Periodic Speed, a step function for the speed governed by period
    void update_periodic_speed();

    // Get Value from Environement
    float get_env_value(const char* string);

    // GPS Error Model Object
    // Unfortunately I am using tight coupling which is not the best design approach
    AU_UAV_ROS::CGPSErrorSimulation& gps_model;

    // Update Wind Speed based ont he wind fluctuation type
    void update_wind_speed();

  };
}

#endif
