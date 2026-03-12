/**
 * @file    unicycle_predictive.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    January 2026
 * @version 1.0
 * @brief   Numerical simulation to test nonlinear feedback control of unicycle class.
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
#include <RobotLibrary/Control/UnicyclePredictive.h>
#include <RobotLibrary/Math/Line.h>
#include <RobotLibrary/Model/Pose2D.h>
#include <RobotLibrary/Trajectory/HermiteTrajectory.h>

// Simulation parameters
double simulationTime   =  20.0;
double controlFrequency = 100.0;

unsigned int simulationSteps  = 2000;
unsigned int predictionSteps  = 100;

int main(int argc, char **argv)
{   
    using namespace Eigen;
    using namespace RobotLibrary;
    
    // Set up the trajectory
    Model::Pose2D startPose(0.0, 0.0, 0.0);
    Model::Pose2D endPose(1.0, 1.0, 0.0);
    Trajectory::HermiteTrajectory trajectory(startPose, endPose, 1.0, simulationTime - 1.0);
    
    // Parameters for the model
    Model::UnicycleParameters modelParameters; 
    
    double density = 500.0;
    double height  = 0.30;
    double radius  = 0.1;
    double mass    = M_PI * radius * radius * height * density;
    
    modelParameters.inertia                = mass * radius * radius / 2.0;                          // Rotational inertia (kg*m^2)
    modelParameters.mass                   = mass;                                                  // Weight (kg)
    modelParameters.maxAngularAcceleration = 1.0;                                                   // Maximum rotational acceleration (rad/s/s)
    modelParameters.maxAngularVelocity     = 200.0 * M_PI / 30.0;                                   // Maximum rotational speed (rad/s)
    modelParameters.maxLinearAcceleration  = 1.0;                                                   // Maximum forward acceleration (m/s/s)
    modelParameters.maxLinearVelocity      = 2.0;                                                   // Maximum forward speed (m/s)
    modelParameters.minimumSafeDistance    = 1e-10;
    modelParameters.propagationUncertainty = Matrix3d::Identity();                                  // Uncertainty of configuration propagation in Kalman filter
    
    // Parameters for the predictive controller
    Control::UnicyclePredictiveParameters controlParameters;
    controlParameters.controlFrequency        = controlFrequency;
    controlParameters.exponent                = 1e-03;                                              // Growth or decay of pose error weighting
    controlParameters.maximumControlStepNorm  = 1e-03;                                              // DDP algorithm terminates early if max. ||du|| is smaller than this
    controlParameters.numberOfRecursions      = 50;                                                 // No. of forward & backward passes for the DDP algorithm
    controlParameters.obstaclePotentialScalar = 5e-02;                                              // Scales the repulsion force
    controlParameters.predictionSteps         = predictionSteps;                                    // Length of prediction horizon
  
    controlParameters.poseErrorWeight <<   1.0,    0.0,    0.0,
                                           0.0,    1.0,  9e-05,
                                           0.0,  9e-05,  1e-04;
                                           
    controlParameters.poseErrorWeight *= 1.0;

    // Create the controller and set initial conditions
    Control::UnicyclePredictive controller(modelParameters, controlParameters);
    Model::Pose2D actualPose(0.0, 0.0, 0.0);                                                        // Start offset from the trajectory
    Vector2d controlInput = {0.0, 0.0}; 
    controller.update_state(actualPose, controlInput);
    
    // Set up obstacle(s)
    std::vector<std::vector<std::vector<Model::Obstacle2D>>> obstacles(simulationSteps);
    
    double r_x = 0.10;
    double r_y = 0.10;
        
    Matrix2d shapeMatrix;
    shapeMatrix << r_x * r_x,       -0.0,
                         -0.00, r_y * r_y; 
                         
    for (int i = 0; i < simulationSteps; ++i)
    {
        obstacles[i].resize(predictionSteps+1);                                                     // MUST be N+1; makes it easier to handle in code
        
        for (int j = 0; j <= predictionSteps; ++j)
        {
            for (int k = 0; k < 1; ++k)
            {
                auto ellipse = std::make_unique<Math::Ellipse>(shapeMatrix);
                
                obstacles[i][j].push_back(Model::Obstacle2D(std::move(ellipse)));
                obstacles[i][j].back().update_state(Model::Pose2D(0.6,0.5, 0.0));
                obstacles[i][j].back().set_name("ellipse_" + std::to_string(k+1));
            }
        }
    }
                                       
    // Set up data arrays
    std::vector<std::vector<Model::UnicycleState>> desiredStates(simulationSteps);                  // For saving trajectory data
    std::vector<std::vector<Model::UnicycleState>> predictedStates(simulationSteps);
    
    // Run the simulation
    for (int i = 0; i < simulationSteps; ++i)
    {
        double simTime = i / controlFrequency;                                                      // Dividing is more numerically stable

        for (int j = 0; j <= predictionSteps; ++j)
        {
            Trajectory::PlanarState planarState = trajectory.query_state(simTime + j / controlFrequency); // Sample trajectory
            
            Model::UnicycleState desiredState;                                                      // Need to convert
            
            desiredState.pose = planarState.pose;                                                   // Pose is the same
            
            double angle = planarState.pose.angle();
            
            desiredState.velocity = {planarState.twist[0] * cos(angle) + planarState.twist[1] * sin(angle),
                                     planarState.twist[2]};                                         // Need to convert to [v, w]

            desiredStates[i].push_back(desiredState);                                               // Append to list
        }
        
        try
        {
            controlInput = controller.track_trajectory(desiredStates[i], obstacles[i]);             // Solve the predictive control problem
        }
        catch (const std::exception &exception)
        {
            throw std::runtime_error("[ERROR] [UNICYCLE PREDICTIVE CONTROL] "
                                     "Failed to solve trajectory tracking on simulation step " + std::to_string(i+1) + ":\n"
                                     + std::string(exception.what()));
             
             break;
        }
        
        // Save data for analysis
        predictedStates[i] = controller.predicted_states();

        // For next loop
        controller.update_state(actualPose, controlInput);
        actualPose = controller.predicted_pose();                                                   // Propagate the state
    }
    
    std::ofstream file;

    // Desired configuration
    file.open("unicycle_desired_configuration.csv");
    file << "time_s,x_position_m,y_position_m,heading_angle_rad\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << desiredStates[i][0].pose.translation()[0] << ",";
        file << desiredStates[i][0].pose.translation()[1] << ",";
        file << desiredStates[i][0].pose.angle() << "\n";
    }
    file.close();

    // Feedforward velocity
    file.open("unicycle_desired_velocity.csv");
    file << "time_s,linear_velocity_m_s,angular_velocity_rad_s\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << desiredStates[i][0].velocity[0] << ",";
        file << desiredStates[i][0].velocity[1] << "\n";
    }
    file.close();
  
    // Actual configuration data
    file.open("unicycle_actual_configuration.csv");
    file << "time_s,x_position_m,y_position_m,heading_angle_rad\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << predictedStates[i][0].pose.translation()[0] << ",";
        file << predictedStates[i][0].pose.translation()[1] << ",";
        file << predictedStates[i][0].pose.angle() << "\n";
    }
    file.close();

    // Control inputs
    file.open("unicycle_control_inputs.csv");
    file << "time_s,linear_velocity_m_s,angular_velocity_rad_s\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << predictedStates[i][0].velocity[0] << ",";
        file << predictedStates[i][0].velocity[1] << "\n";
    }
    file.close();

    // Robot properties
    file.open("unicycle_robot_properties.csv");
    file << "robot_inertia,robot_mass,robot_radius\n";
    file << modelParameters.inertia << ",";
    file << mass << ",";
    file << radius;
    file.close();

    // Obstacle
    file.open("unicycle_obstacle_data.csv");
    file << "time_s,centre_x,centre_y,xx,xy,yy\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << obstacles[i][0][0].pose().translation()[0] << ",";
        file << obstacles[i][0][0].pose().translation()[1] << ",";
        file << shapeMatrix(0,0) << ",";
        file << shapeMatrix(0,1) << ",";
        file << shapeMatrix(1,1) << "\n";
    }
    file.close();
    
    // Predicted x positions
    file.open("unicycle_predicted_x.csv");
    for (int i = 0; i < simulationSteps; ++i)
    {
        for (int j = 0; j <= predictionSteps; ++j)
        {
            file << predictedStates[i][j].pose.translation()[0];
            
            if (j < predictionSteps) file << ",";
            else                     file << "\n";
        }
    }
    file.close();
    
    // Predicted x
    file.open("unicycle_predicted_y.csv");
    for (int i = 0; i < simulationSteps; ++i)
    {
        for (int j = 0; j <= predictionSteps; ++j)
        {
            file << predictedStates[i][j].pose.translation()[1];
            
            if (j < predictionSteps) file << ",";
            else                     file << "\n";
        }
    }
    file.close();

    std::cout << "[INFO] [UNICYCLE PREDICTIVE CONTROL] Numerical simulation complete. "
              << "Data saved to .csv files for analysis.\n";
    
    return 0;                                                                                          // No problems with main
}
