/**
 * @file    unicycle_feedback.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    January 2026
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
#include <RobotLibrary/Control/UnicycleFeedback.h>
#include <RobotLibrary/Math/Ellipsoid.h>
#include <RobotLibrary/Model/Pose2D.h>
#include <RobotLibrary/Trajectory/MinimumArcLength.h>

// Simulation parameters
double controlFrequency  = 100;
double simulationTime    = 10.0;

int simulationSteps = 1000;

int main(int argc, char **argv)
{   
    using namespace Eigen;
    using namespace RobotLibrary;
    
    // Set up the trajectory
    Model::Pose2D startPose(0.0, 0.0, 0.0);
    Vector2d endPoint = {-1.0, 1.0};
    Trajectory::MinimumArcLength trajectory(startPose, endPoint, 1.0, simulationTime - 1.0);
    
    // Parameters for the model
    Model::UnicycleParameters modelParameters; 
    
    double density = 500.0;
    double height  = 0.30;
    double radius  = 0.25;
    double mass    = M_PI * radius * radius * height * density;
    
    modelParameters.inertia                = mass * radius * radius / 2.0;                          // Rotational inertia (kg*m^2)
    modelParameters.mass                   = mass;                                                  // Weight (kg)
    modelParameters.maxAngularAcceleration = 1.0;                                                   // Maximum rotational acceleration (rad/s/s)
    modelParameters.maxAngularVelocity     = 100.0 * M_PI / 30.0;                                   // Maximum rotational speed (rad/s)
    modelParameters.maxLinearAcceleration  = 1.0;                                                   // Maximum forward acceleration (m/s/s)
    modelParameters.maxLinearVelocity      = 2.0;                                                   // Maximum forward speed (m/s)
    modelParameters.minimumSafeDistance    = 1e-10;                                                   // Make it the same as the robot
    modelParameters.propagationUncertainty = Matrix3d::Identity();                                  // Uncertainty of configuration propagation in Kalman filter
    
    // Parameters for the feedback controller
    Control::UnicycleFeedbackParameters controlParameters;
    
    controlParameters.controlFrequency    = controlFrequency;
    controlParameters.minimumSafeDistance = 1e-06;
    controlParameters.orientationGain     = 10.0;
    controlParameters.xPositionGain       = 20.0;
    controlParameters.yPositionGain       = 40.0;
    
    controlParameters.qpSolver.stepSizeTolerance = 1e-04;                                           // Needs to be very small for this low dimensional problem

    Control::UnicycleFeedback controller(modelParameters, controlParameters);                       // Here is where we actually create the controller
    
    // Set up the obstacles
    double rx = 0.1;
    double ry = 0.1;
    Matrix2d shapeMatrix;
    shapeMatrix << rx*rx, 0.0,
                     0.0, ry*ry;                                                                    // Defines shape of ellipse
    
    std::vector<std::vector<Model::Obstacle2D>> obstacles(simulationSteps);                         // Container
    
    for (int i = 0; i < simulationSteps; ++i)
    {
        auto ellipse = std::make_unique<Math::Ellipse>(shapeMatrix);                                // Underlying shape
        obstacles[i].push_back(Model::Obstacle2D(std::move(ellipse)));
        obstacles[i].back().update_state(Model::Pose2D(-0.3, 0.70, 0.0));
        obstacles[i].back().set_name("ellipse_01");
    }
    
    // Set up data arrays
    std::vector<Model::UnicycleState> desiredStates(simulationSteps);                                // For saving trajectory data
    std::vector<Model::UnicycleState> actualStates(simulationSteps);
    
    // Start conditions
    Model::Pose2D actualPose(0.0, 0.0, 0.0);                                                        // Start offset from the trajectory
    Vector2d controlInput = {0.0, 0.0};
    controller.update_state(actualPose, controlInput);

    // Run the simulation
    for (int i = 0; i < simulationSteps; ++i)
    { 
        double simTime = i / controlFrequency;
        
        // Query the desired state from the trajectory generator
        const auto &[desiredConfiguration,
                     desiredVelocity,
                     desiredAcceleration] = trajectory.query_state(simTime);                        // Get the desired state
                     
        Model::Pose2D desiredPose(desiredConfiguration[0],
                                  desiredConfiguration[1],
                                  desiredConfiguration[2]);                                         // We need to put it in a Pose2D object
        
        // NOTE: QP solver can throw an error if no solution exists.
        try
        {  
            controlInput = controller.track_trajectory(desiredPose, desiredVelocity, obstacles[i]);
        }
        catch (const std::exception &exception)
        {
            std::cout << exception.what() << "\n";
            
            break;
        }
        
        // Save data for analysis
        desiredStates[i] = Model::UnicycleState{desiredPose, desiredVelocity};
        actualStates[i]  = Model::UnicycleState{actualPose, controlInput};

        // Update for next loop
        controller.update_state(actualPose, controlInput);
        actualPose = controller.predicted_pose();                                                   // Assume perfect tracking
    }
    
    std::ofstream file;

    // Desired configuration
    file.open("unicycle_desired_configuration.csv");
    file << "time_s,x_position_m,y_position_m,heading_angle_rad\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << desiredStates[i].pose.translation()[0] << ",";
        file << desiredStates[i].pose.translation()[1] << ",";
        file << desiredStates[i].pose.angle() << "\n";
    }
    file.close();
    
    // Feedforward velocity
    file.open("unicycle_desired_velocity.csv");
    file << "time_s,linear_velocity_m_s,angular_velocity_rad_s\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << desiredStates[i].velocity[0] << ",";
        file << desiredStates[i].velocity[1] << "\n";
    }
    file.close();
    
    // Actual configuration data
    file.open("unicycle_actual_configuration.csv");
    file << "time_s,x_position_m,y_position_m,heading_angle_rad\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << actualStates[i].pose.translation()[0] << ",";
        file << actualStates[i].pose.translation()[1] << ",";
        file << actualStates[i].pose.angle() << "\n";
    }
    file.close();
    
    // Control inputs
    file.open("unicycle_control_inputs.csv");
    file << "time_s,linear_velocity_m_s,angular_velocity_rad_s\n";
    for (int i = 0; i < simulationSteps; ++i)
    {
        file << (double)(i / controlFrequency) << ",";
        file << actualStates[i].velocity[0] << ",";
        file << actualStates[i].velocity[1] << "\n";
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
        file << obstacles[i][0].pose().translation()[0] << ",";
        file << obstacles[i][0].pose().translation()[1] << ",";
        file << shapeMatrix(0,0) << ",";
        file << shapeMatrix(0,1) << ",";
        file << shapeMatrix(1,1) << "\n";
    }
    file.close();

    std::cout << "[INFO] [UNICYCLE FEEDBACK CONTROL] Numerical simulation complete. "
              << "Data saved to .csv files for analysis.\n";
    
    return 0;                                                                                       // No problems with main
}
