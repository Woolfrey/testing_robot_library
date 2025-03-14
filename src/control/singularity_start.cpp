/**
 * @file singularity_start_test.cpp
 * @author Jon Woolfrey
 * @date July 2024
 * @brief Tests the robustness of Cartesian velocity control when starting from a singular configuration.
 */

#include <Eigen/Core>                                                                               // Eigen::Vector, Eigen::Matrix classes
#include <fstream>                                                                                  // Reading and writing to files
#include <iostream>                                                                                 // std::cout, std::cerr
#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // Custom control class
#include <RobotLibrary/Trajectory/CartesianSpline.h>                                                           // Custom trajectory generator
#include <time.h> 

// Parameters for the numerical simulation
double simulationFrequency = 1000;
unsigned int ratio         = 10;
double controlFrequency    = simulationFrequency/ratio;
double simulationDuration  = 3.0;
unsigned int simulationSteps = simulationDuration*simulationFrequency;

// Parameters for the trajectory
double startTime = 0.0;
double endTime = simulationDuration - 0.5;

int main(int argc, char** argv)
{
    // Default for argc is 1 but I don't know why ┐(ﾟ ～ﾟ )┌
    if(argc != 3)
    {
        std::cerr << "[ERROR] [URDF TEST] Incorrect arguments. "
                  << "Usage: ./singularity_start_test "
                  << "/path/to/file.urdf endpoint_name "
                  << "endpoint_name\n";
             
        return -1;                                                                                  // Exit main() with error
    }

    srand(time(NULL));                                                                              // Seed the random number generator	

    // Set up the controller
    auto model = std::make_shared<RobotLibrary::Model::KinematicTree>(argv[1]);

    RobotLibrary::Control::SerialKinematicControl controller(model, argv[2]);                       // Create controller for given endpoint

    unsigned int n = model->number_of_joints();                                                     // Because I'm lazy

    Eigen::VectorXd jointPosition = Eigen::VectorXd::Zero(n);                                       // Set a random start configuration
    Eigen::VectorXd jointVelocity = Eigen::VectorXd::Zero(n);                                       // Start at rest

    model->update_state(jointPosition, jointVelocity);                                              // Updates the forward kinematics
    controller.update();                                                                            // Updates properties specific to this controller

    std::cout << "Starting manipulability: " << controller.manipulability() << "\n";

    // Set up the Cartesian trajectory
    
    RobotLibrary::Model::Pose startPose = controller.endpoint_pose();                               // Get the current endpoint pose

    Eigen::Vector3d offset = Eigen::VectorXd::Random(3);                                            // Set a random offset

    RobotLibrary::Model::Pose endPose(startPose.translation() + offset, startPose.quaternion());    // Offset the start pose
 
    RobotLibrary::Trajectory::CartesianSpline trajectory(startPose, endPose,
                                                         Eigen::Vector<double,6>::Zero(),
                                                         startTime, endTime);                       // Create the trajectory

    // Establish arrays for saving data
    unsigned int m = simulationSteps/ratio;

    Eigen::MatrixXd positionArray(m,n);
    Eigen::MatrixXd velocityArray(m,n);
    Eigen::MatrixXd poseErrorArray(m,3);

    unsigned int rowCounter = 0;                                                                    // For indexing across arrays

    for(int i = 0; i < simulationSteps; i++)
    {

      double simulationTime = i/simulationFrequency;                                                // Current simulation time
      
      jointPosition += jointVelocity/simulationFrequency;                                           // Update the joint position
      
      // Run the control at 1/10th of the simulation
      if(i%ratio == 0)
      {                                                       
           // Record data
           positionArray.row(rowCounter) = jointPosition.transpose();
           velocityArray.row(rowCounter) = jointVelocity.transpose();
           
           // Solve control
           try
           {
                model->update_state(jointPosition, jointVelocity);                                  // Update kinematics & dynamics
           }
           catch(const std::exception &exception)
           {
                std::cerr << exception.what() << "\n";  
                std::cerr << "[ERROR] [CARTESIAN VELOCITY CONTROL TEST] "
                          << "Model update failed. Terminating.\n";
                
                return -1;                                                                          // Stop
           }
           
           controller.update();                                                                     // Update the controller
           
           RobotLibrary::Trajectory::CartesianState desiredState = trajectory.query_state(simulationTime);

           try
           {
                jointVelocity = controller.track_endpoint_trajectory(desiredState.pose,
                                                                     desiredState.twist,
                                                                     desiredState.acceleration);
           }
           catch(const std::exception &exception)
           {
                std::cerr << exception.what() << "\n";
                
                jointVelocity = 0.5*jointVelocity;                                                  // Slow down
           }

           
           // Save pose error
           Eigen::Vector<double,6> poseError = controller.endpoint_pose().error(desiredState.pose);
           
           poseErrorArray(rowCounter,0) = poseError.head(3).norm();                                 // Position error
           poseErrorArray(rowCounter,1) = poseError.tail(3).norm();                                 // Orientation error
           poseErrorArray(rowCounter,2) = controller.manipulability();                              // Proximity to a singularity
           
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

    // Save pose error tracking data
    file.open("pose_error_data.csv");
    for(int i = 0; i < m; i++)
    {
      file << (double)(i/controlFrequency);
      for(int j = 0; j < 3; j++) file << "," << poseErrorArray(i,j);
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

    std::cout << "[INFO] [CARTESIAN VELOCITY CONTROL TEST]: "
           << "Numerical simulation complete. Data saved to .csv file for analyis.\n";

    return 0;                                                                                      // No problems with main()
}
