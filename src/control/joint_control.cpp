/**
 * @file    joint_control.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   Numerical simulation for joint trajectory tracking.
 * 
 * @details This executable performs a numerical simulation to assess the joint trajectory tracking
 *          for the SerialLinkBase class and its child classes.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#include <fstream>                                                                                  // Reading and writing to files
#include <iostream>                                                                                 // std::cout, std::cerr
#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // Custom control class
#include <RobotLibrary/Control/SerialDynamicControl.h>
#include <RobotLibrary/Trajectory/SplineTrajectory.h>                                               // Custom trajectory generator
#include <time.h>

// Parameters for the numerical simulation
double simulationFrequency   = 1000;
unsigned int ratio           = 10;
double controlFrequency      = (double)(simulationFrequency / ratio);
double simulationDuration    = 3.0;
unsigned int simulationSteps = simulationDuration * simulationFrequency;

// Parameters for the trajectory
double startTime = 0.0;
double endTime = simulationDuration - 0.5;
int polynomialOrder = 5;

int main(int argc, char** argv)
{
    // Default for argc is 1 but I don't know why ┐(ﾟ ～ﾟ )┌
	if(argc != 4)
	{
		std::cerr << "[ERROR] [JOINT CONTROL] Invalid arguments. "
		          << "Usage: ./joint_control /path/to/file.urdf endpoint_name MODE\n";
	         
		return -1;                                                                                  // Exit main() with error
	}
	
    srand(time(NULL));                                                                              // Seed the random number generator	

    // Set up the controller
    auto model = std::make_shared<RobotLibrary::Model::KinematicTree>(argv[1]);                     // Create shared ptr for model

    std::string endpointName = argv[2];

    std::unique_ptr<RobotLibrary::Control::SerialLinkBase> controller;                              // This allows for polymorphism

         if (argv[3] == std::string("VELOCITY")) controller = std::make_unique<RobotLibrary::Control::SerialKinematicControl>(model, endpointName);
    else if (argv[3] == std::string("TORQUE"))   controller = std::make_unique<RobotLibrary::Control::SerialDynamicControl>(model, endpointName);
    else
    {
        std::cerr << "[ERROR] [JOINT CONTROL] Invalid argument for control mode. Options are VELOCITY or TORQUE.\n";         
        return -1;
    }

    unsigned int n = model->number_of_joints();

    // Set up the trajectory

    std::vector<RobotLibrary::Trajectory::State> waypoints;

    waypoints.push_back({ Eigen::VectorXd::Random(n),                                               // Position
                          Eigen::VectorXd::Zero(n),                                                 // Velocity
                          Eigen::VectorXd::Zero(n) });                                              // Acceleration

    waypoints.push_back({ 3*Eigen::VectorXd::Random(n),                                             // Position
                            Eigen::VectorXd::Zero(n),                                               // Velocity
                            Eigen::VectorXd::Zero(n) });                                            // Acceleration
                             
    std::vector<double> times = {startTime, endTime};                               

    RobotLibrary::Trajectory::SplineTrajectory trajectory(waypoints,times,polynomialOrder);

    // Set up the simulation
    
    unsigned int m = simulationSteps / ratio;

    Eigen::MatrixXd positionArray(m,n);
    Eigen::MatrixXd velocityArray(m,n);
    Eigen::MatrixXd positionErrorArray(m,n);

    unsigned int rowCounter = 0;

    Eigen::VectorXd jointPosition = waypoints.front().position;
    Eigen::VectorXd jointVelocity = waypoints.front().velocity;
    Eigen::VectorXd jointControl  = Eigen::VectorXd::Zero(n);
     
    // Run the numerical simulation
    for(int i = 0; i < simulationSteps; i++)
    {
        model->update_state(jointPosition, jointVelocity);
        
        double simulationTime = i / simulationFrequency;
        
             if (argv[3] == std::string("VELOCITY")) jointVelocity  = jointControl;
        else if (argv[3] == std::string("TORQUE"))   jointVelocity += model->joint_inertia_matrix().llt().solve(jointControl - model->joint_coriolis_matrix() * model->joint_velocities()) / simulationFrequency;
        else
        {
            std::cerr << "[ERROR] [JOINT CONTROL] Control mode was " << argv[3] << " but must be VELOCITY or TORQUE.\n";
        }
         
        jointPosition += jointVelocity / simulationFrequency;
        
        // Run the control at 1/10th of the simulation
        if(i%ratio == 0)
        { 
            const auto &desired = trajectory.query_state(simulationTime);
            
            jointControl = controller->track_joint_trajectory(desired.position, desired.velocity, desired.acceleration);
            
            // Record data
            positionArray.row(rowCounter)      = jointPosition.transpose();
            velocityArray.row(rowCounter)      = jointVelocity.transpose();
            positionErrorArray.row(rowCounter) = (desired.position - jointPosition).transpose();

            rowCounter++;
        }
    }
    
     std::ofstream file;

     // Save the position data
     file.open("joint_position_data.csv");
     for(int i = 0; i < m; i++)
     {
          file << (double)(i/controlFrequency);
          for(int j = 0; j < n; j++) file << "," << positionArray(i,j);
          file << "\n";
     }
     file.close();
     
     // Save the velocity data
     file.open("joint_velocity_data.csv");
     for(int i = 0; i < m; i++)
     {
          file << (double)(i/controlFrequency);
          for(int j = 0; j < n; j++) file << "," << velocityArray(i,j);
          file << "\n";
     }
     file.close();
     
     // Save position error tracking data
     file.open("joint_position_error_data.csv");
     for(int i = 0; i < m; i++)
     {
          file << (double)(i/controlFrequency);
          for(int j = 0; j < n; j++) file << "," << positionErrorArray(i,j);
          file << "\n";
     }
     file.close();
     
     // Save position limits
     file.open("joint_limits.csv");
     {
          for(int j = 0; j < n; j++)
          {
               std::string name = model->joint(j).name();
               const auto &[lower, upper] = model->joint(j).position_limits();
               double velocity = model->joint(j).speed_limit();
               
               file << name << "," << lower << "," << upper << "," << velocity << "\n";
          }
     }
     file.close();
     
     std::cout << "[INFO] [JOINT CONTROL]: "
               << "Numerical simulation complete. Data saved to .csv file for analyis.\n";
     
     return 0;                                                                                      // No problems with main()
}
