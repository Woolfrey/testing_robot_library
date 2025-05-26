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
#include <RobotLibrary/Control/DifferentialDrivePredictive.h>
#include <RobotLibrary/Model/Pose2D.h>
#include <RobotLibrary/Trajectory/MinimumArcLength.h>

// Simulation parameters
double simulationTime = 10;
double controlFrequency = 100.0;
unsigned int simulationSteps = 1000;
unsigned int predictionSteps = 200;

int main(int argc, char **argv)
{   
    // Set up the trajectory
    RobotLibrary::Model::Pose2D startPose(0.0, 0.0, 0.0);
    Eigen::Vector2d endPoint = {-1.0, 1.0};
    RobotLibrary::Trajectory::MinimumArcLength trajectory(startPose, endPoint, 1.0, simulationTime - 1.0);
    
    // Parameters for the model
    RobotLibrary::Model::DifferentialDriveParameters modelParameters;
    modelParameters.inertia                = 0.5 * 5.0 * 0.25 * 0.25;                              // Rotational inertia (kg*m^2)
    modelParameters.mass                   = 5.0;                                                  // Weight (kg)
    modelParameters.maxAngularAcceleration = 4.0;                                                   // Maximum rotational acceleration (rad/s/s)
    modelParameters.maxAngularVelocity     = 100.0 * M_PI / 30.0;                                   // Maximum rotational speed (rad/s)
    modelParameters.maxLinearAcceleration  = 1.0;                                                   // Maximum forward acceleration (m/s/s)
    modelParameters.maxLinearVelocity      = 2.0;                                                   // Maximum forward speed (m/s)
    modelParameters.propagationUncertainty = Eigen::Matrix3d::Identity();                           // Uncertainty of configuration propagation in Kalman filter
    
    // Parameters for the QP solver
    SolverOptions<double> solverOptions;
    solverOptions.barrierReductionRate = 1e-02;
    solverOptions.initialBarrierScalar = 100;
    solverOptions.stepSizeTolerance    = 1e-08;                                                     // This should be very small
    solverOptions.maxSteps             = 5;
    
    // Parameters for the predictive controller
    RobotLibrary::Control::DifferentialDrivePredictiveParameters controlParameters;
    controlParameters.controlFrequency       = controlFrequency;
    controlParameters.exponent               = -0.05;                                               // Growth or decay of pose error weighting
    controlParameters.maximumControlStepNorm = 1e-06;                                               // DDP algorithm terminates early if max. ||du|| is smaller than this
    controlParameters.numberOfRecursions     = 2;                                                   // No. of forward & backward passes for the DDP algorithm
    controlParameters.predictionSteps        = predictionSteps;                                     // Length of prediction horizon
   
    controlParameters.poseErrorWeight << 500.0,   0.0, 0.0,
                                           0.0, 500.0, 0.9,
                                           0.0,   0.9, 1.0;
    
    RobotLibrary::Control::DifferentialDrivePredictive controller(modelParameters,
                                                                  controlParameters,
                                                                  solverOptions);
 
    RobotLibrary::Model::Pose2D actualPose(-0.2, 0.2, 0.5);                                        // Start offset from the trajectory
    
    Eigen::Vector2d controlInput = {0.0, 0.0};
    
    controller.update_state(actualPose, controlInput);
    
    // Set up obstacle(s)
    std::vector<std::vector<RobotLibrary::Math::Ellipsoid<2>>> obstacles;
       
    // Set up data arrays for analysis
    std::vector<std::array<double,3>> desiredConfiguration; desiredConfiguration.resize(simulationSteps);
    std::vector<std::array<double,3>> actualConfiguration; actualConfiguration.resize(simulationSteps);
    std::vector<std::array<double,2>> poseError; poseError.resize(simulationSteps);
    std::vector<std::array<double,2>> controlInputs; controlInputs.resize(simulationSteps);
    
    // Run the simulation
    for (int i = 0; i < simulationSteps; ++i)
    {
        double simTime = i / controlFrequency;
        
        // Query the desired state from the trajectory across the control horizon
        std::vector<RobotLibrary::Model::DifferentialDriveState> desiredStates;
        
        for (int j = 0; j < predictionSteps + 1; ++j)
        {
            const auto &[pos, vel, acc] = trajectory.query_state(simTime + j / controlFrequency);   // Sample the trajectory across the horizon
         
            // We need to put it in a data structure
            RobotLibrary::Model::DifferentialDriveState state;
            state.pose = RobotLibrary::Model::Pose2D(pos[0], pos[1], pos[2]);
            state.velocity = vel;
            
            desiredStates.push_back(state);   
        }

        try
        {
            controlInput = controller.track_trajectory(desiredStates, obstacles);                   // Solve the predictive control problem
        }
        catch (const std::exception &exception)
        {
            throw std::runtime_error("[ERROR] [DIFFERENTIAL DRIVE PREDICTIVE CONTROL] "
                                     "Failed to solve trajectory tracking:\n"
                                     + std::string(exception.what()));
             
             return -1;
        }
        
        // Save data for future analysis
        desiredConfiguration[i] = {desiredStates[0].pose.translation()[0], desiredStates[0].pose.translation()[1], desiredStates[0].pose.angle()};
        actualConfiguration[i]  = {actualPose.translation()[0], actualPose.translation()[1], actualPose.angle()};
        poseError[i]            = {(desiredStates[0].pose.translation() - controller.pose().translation()).norm(), abs(desiredStates[0].pose.angle() - controller.pose().angle())};                   
        controlInputs[i]        = {controlInput[0], controlInput[1]};
        
        // For next loop
        controller.update_state(actualPose, controlInput);
        actualPose = controller.predicted_pose();                                                   // Propagate the state
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

    // Save the actual configuration data
    file.open("actual_configuration_data.csv");
    for(int i = 0; i < simulationSteps; ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 3; ++j) file << "," << actualConfiguration[i][j];
      file << "\n";
    }
    file.close();  
    
    // Save the control data
    file.open("control_input_data.csv");
    for(int i = 0; i < simulationSteps; ++i)
    {
      file << (double)(i / controlFrequency);
      for(int j = 0; j < 2; ++j) file << "," << controlInputs[i][j];
      file << "\n";
    }
    file.close(); 
    
    // Save the error data
    file.open("tracking_error_data.csv");
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency);
        for(int j = 0; j < 2; ++j) file << "," << poseError[i][j];
        file << "\n";
    }
    file.close(); 
    
    std::cout << "[INFO] [DIFFERENTIAL DRIVE PREDICTIVE CONTROL] Numerical simulation complete."
              << "Data saved to .csv files for analysis.\n";

    return 0;                                                                                       // No problems with main
}
