# :alembic: Test Code for RobotLibrary

This repository contains executables to numerically validate the algorithms in [Robot Library](https://github.com/Woolfrey/software_robot_library).

#### :compass: Navigation
- [Requirements](#clipboard-requirements)
- [Installation](#floppy_disk-installation)
- [Usage](#wrench-usage)
- [Contributing](#handshake-contributing)
- [License](#scroll-license)

## :clipboard: Requirements

- [Eigen 3.4](https://eigen.tuxfamily.org/index.php?title=Main_Page), and
- [Robot Library](https://github.com/Woolfrey/software_robot_library) (obviously!)

> [!NOTE]
> This repository was built & tested in Ubuntu 22.04. If you're using Ubuntu 20.04 (or earlier?), you will need to manually install Eigen 3.4, instead of through the command line.

[:top: Back to top.](#alembic-test-code-for-robotlibrary)

## :floppy_disk: Installation

The directory structure should look something like this:
```
workspace/
├── software_robot_library/
└── testing_robot_library/
    ├── python/
    ├── src/
    ├── urdf/
    ├── CMakeLists.txt
    ├── LICENSE
    └── package.xml
```

1. In your `workspace` directory, clone the repository:

  `git clone https://github.com/Woolfrey/testing_robot_library.git`

2. Navigate in to the directory and create the build folder:

  `cd testing_robot_library`
  `mkdir build && cd build`

3. Now make it:

  `cmake ../`
  `make`

If you now type `ls` in to the terminal you should see multiple executables:
- cartesian_velocity_control
- joint_velocity_control
- load_model
- singularity_start
- spline
- trapezoidal_velocity

[:top: Back to top.](#alembic-test-code-for-robotlibrary)

## :wrench: Usage

You can run the executables from the `build/` folder using `./<executable_name> <argument_1> <argument_2> ... etc.`

They each output a `.csv` file of data (trajectory tracking error, joint positions, joint velocities, etc.) that you can use to plot the data.

Inside the `python/` folder are several Python scripts for visualising the data.

[:top: Back to top.](#alembic-test-code-for-robotlibrary)

## :handshake: Contributing

Contributions to this repositore are welcome! Feel free to:
1. Fork the repository,
2. Implement your changes / improvements, then
3. Issue a pull request.

You can also raise an issue to ask for new features.

[:top: Back to top.](#alembic-test-code-for-robotlibrary)

## :scroll: License

This software package is licensed under the [GNU General Public License v3.0 (GPL-3.0)](https://choosealicense.com/licenses/gpl-3.0/). You are free to use, modify, and distribute this package, provided that any modified versions also comply with the GPL-3.0 license. All modified versions must make the source code available and be licensed under GPL-3.0. The license also ensures that the software remains free and prohibits the use of proprietary restrictions such as Digital Rights Management (DRM) and patent claims. For more details, please refer to the [full license text](LICENSE).

[:top: Back to top.](#alembic-test-code-for-robotlibrary)
