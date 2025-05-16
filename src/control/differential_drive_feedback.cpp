/**
 * @file    differential_drive_feedback.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   Numerical simulation to test nonlinear feedback control of differential drive class.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Eigen/Core>
#include <fstream>                                                                                  // Reading and writing to files
#include <iostream>  
#include <RobotLibrary/Model/Pose2D.h>
#include <RobotLibrary/Trajectory/MinimumArcLength.h>

double controlFrequency  = 100.0;
double simulationTime    = 6.0;
Eigen::Vector2d endPoint = {-1.0, -1.0};
RobotLibrary::Model::Pose2D startPose(0.0, 0.0, 0.0);

int simulationSteps =  static_cast<int>(controlFrequency * simulationTime);

int main(int argc, char **argv)
{   
    RobotLibrary::Trajectory::MinimumArcLength trajectory(startPose, endPoint, 0.0, simulationTime - 1.0);
    
    // Set up data arrays
    std::vector<std::array<double,3>> desiredConfiguration;
    std::vector<std::array<double,2>> poseError;
    std::vector<std::array<double,2>> controlInputs;
    
    for (int i = 0; i < simulationSteps; ++i)
    {
        double simTime = i / controlFrequency;
        
        const auto &[position, velocity, acceleration] = trajectory.query_state(simTime);           // Get the desired state
        
        RobotLibrary::Model::Pose2D desiredPose(position[0], position[1], position[2]);             // We need to put it in a Pose2D object
        
       desiredConfiguration.push_back({position[0], position[1], position[2]});
    }
    
    std::ofstream file;

    // Save the trajectory data
    file.open("desired_configuration_data.csv");
    for(int i = 0; i < simulationSteps; ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 3; ++j) file << "," << desiredConfiguration[i][j];
      file << "\n";
    }
    file.close();   
    
    return 0;                                                                                       // No problems with main
}
