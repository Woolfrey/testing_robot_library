#include <iostream>                                                                                 // std::cerr, std::cout
#include <fstream> 
#include <RobotLibrary/Model/KinematicTree.h>
#include <RobotLibrary/Control/SerialKinematicControl.h>                                            // Custom class for robot physics
#include <time.h>                                                                                   // Timer

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    KinematicTree<double> model("../urdf/ur5.urdf");
    
    model.update_state(Eigen::VectorXd::Random(6),
                       Eigen::VectorXd::Random(6));
                       
    SerialKinematicControl<double> controller(&model, "ee_link");
}
