# FastSLAM Programming Design Document
Version 1.0, modified 05/23/2024

## Table of Contents
- [Purpose](#1-purpose)
- [Algorithm Overview](#2-algorithm-overview)
- [Component Breakdown](#3-component-breakdown)
- [Notes](#4-notes)
- [References](#5-references)

## 1. Purpose 

This design document lays out the general structure of our implementation of the FastSLAM 1.0 algorithm by [Montemerlo et al (2002)](https://cdn.aaai.org/AAAI/2002/AAAI02-089.pdf). FastSLAM is a filtering-based, factored solution to the Simultaneous Localization and Mapping problem. The algorithm uses a collection of Extended Kalman Filters (EKF) and a particle filter overall to simultaneously track landmarks in the environment and produce a global estimate of the robot's past trajectory through time.

Our goals with this implementation are threefold. First, we want to understand the SLAM problem by dissecting a classical solution from the filtering era of SLAM solutions. Second, we want to practice and hone our software engineering, testing and robotics skills with a complete, from-scratch implementation. Lastly, this project will serve as a teaching tool for future interested students in the lab.

With these goals in mind, our overall FastSLAM system will include more components than just the core algorithm. Before this design document was first written, the software already contained support structures such as Continuous Integration (CI) procedures, automatic unit tests, and an online API documentation website via a simple Continuous Delivery (CD) pipeline. If time and team capacity permits, more, fancier features can be added to the collection. Running this algorithm on robot is very much within our interest, so we also design and include robot-facing components in our system below.

An [earlier design document](link) by Benned already exists. This document aims to further dissect the design and provide concrete guidelines on implementation. The design largely follows what was originally planned by Benned, with small exceptions such as a separate `ParticleFilter` class and balanced-BST lookup for landmark updates.

## 2. Algorithm Overview

At a glance, the FastSLAM algorithm can be broken down into the following steps without delving into the mathematics:

1. **Sample new robot pose** based on input and previous pose
2. **Obtain landmark measurements**. If more than one landmarks are sighted at once, landmarks can be processed sequentially.
3. **Process landmark measurements using EKF instances**. If the landmark is new, add a new EKF instance. If the landmark has been observed before, update landmark position estimates accordingly. Retain previous landmark information if landmark is not observed.
4. **Repeat step 1\~3 for each particle** in the particle filter. If there exists multiple measurements, **repeat step 2\~3 again for each particle in the particle filter for each measurement**. Add the final particle set to the auxiliary particle set
5. **Sample the auxiliary particle set with replacement** to form a new particle set. The sampling is done based on the importance factor calculated for each particle

Based on the general algorithm, we form our implementation with the following components, ranked in the order of increasing scope. 

1. Mathematical Utilities: coordinate, measurement and pose structs with helper functions
2. RobotManager: robot-facing utilities. Any sensor interfacing functions and robot control schemes will fall under this category/class
3. EKF: template for all EKF instances; contains all necessary fields/methods to maintain landmark EKFs
4. Particle Filter: simple class for a Rao-Blackwellised particle filter. Manages instances of EKF within particles and provides sampling utilities to update
5. FastSLAM: general manager class that carries out the entire algorithm; interfaces with particle filter, EKFs, Robot manager, and any supporting services (e.g. simulation, visualization, etc).

We will break down the design of each component in the next section.

## 3. Component Breakdown

### I. Mathematical Utilities

As outlined by Benned's initial design document, the following structs will be implemented:
- `Gaussian2D`: Represents a 2D Gaussian distribution with mean vector $\mu \in \mathbb{R}^2$ and covariance matrix $\Sigma \in \mathbb{R}^{2 \times 2}$.
- `LandmarkObservation`: Represents a range-bearing-signature measurement of a single landmark: $z_t^i = (r_t^i, \phi_t^i, n_t^i)$ (units are meters, radians, and N/A). To represent both known and unknown data association, $n_t^i$ is wrapped in a `std::optional` object.
- `Point2D`: Represents a point (i.e., just position) on the 2D plane: $(x,y)$.
- `Pose2D`: Represents a pose (i.e., position and orientation) on the 2D plane: $(x,y,\theta)$.
- `VelCommand`: Represents a 2D planar robot's nominal velocity at a particular time step as $(v_x, \omega_z)$, with *linear velocity* $v_x$ (m/s) and *angular velocity* $\omega_z$ (rad/s). Note that this command is in the robot frame

Helper functions include:
- `frameTransfer`: transfers global frame quantity to body frame quantity based on rotation. In 2D, this function only depends on heading $\theta$
- `invFrameTransfer`: transfers body frame quantity to global frame quantity
- `sampleGaussian`: sample a provided gaussian distribution

More functions/structs can be added in future revisions. Math utilities are currently implemented as structs purely for simplicity sake. If this code base were to be generalized to 3D in the future, we will refactor the design of these utilities.

### II. RobotManager

This component can be treated as a translation layer between the algorithm and sensor readings. The algorithm part of our FastSLAM system will use an instance of `RobotManager` to query for sensor data. `RobotManager` can be implemented using an abstract virtual class, inherited by specific robot platforms. Instances/children of this class should also take care of communication via a protocol wrapper (ROS or MQTT).

Public methods include:
- `sampleIMU`: query and gather IMU data, usually for process input to the motion model
- `sampleSensor`: sample a specific sensor
- `sampleLandMark`: query and gather landmark from sensors, returning landmark position and id for data association
- `sampleControl`: return the current control signal applied to the robot
- `motionUpdate`: provide an update on robot trajectory given last pose. Note this function is probabilistic, and the result is a sample from the motion model distribution
- `predictMeas`: predicts the measurement based on sensor's measurement model and robot trajectory

Note that `RobotManager` is only responsible for querying robot measurements. It is not responsible for controlling the robot. This approach follows the separation principle of control theory and allows us to use an optimal deterministic controller for the system.

### III. EKF

The EKF module keeps tracks of individual landmark's position in global frame (with respect to an arbitrary origin). Landmark sightings and robot poses are used to update the posterior belief about the landmark position. It is important to note that landmark measurements come in the form of polar positions (range and bearing) in robot frame, so the robot pose is required to transform landmark measurements into global coordinates.

The EKF class will include the following public methods:
- `update`: correct internal states using new landmark measurement. Note that the landmark position is static, so no process input required.
- `updateCov`: update measurement covariance
- `calcCov`: calculate measurement covariance
- `calcCPD`: calculate likelihood of correspondence given the measurement, returns a scalar weight used for solving data association
- `init`: initialize mean and covariance

Since each particle will contain `K` EKF instances given `K` landmarks, one of our future goals is to add multi-threading capabilities for processing multiple sightings.

### IV. Particle Filter

The FastSLAM system employs a [Rao-Blackwellised Particle Filter](https://www.cs.cmu.edu/~motionplanning/reading/boris-presentation.pdf) to estimate the collective posterior of robot pose and landmark positions. 

Particles in the filter can be represented by `std::pair`s of robot pose and landmark observations. Here we can use the `std::map` associative container to represent the set of landmarks, identified by their positive indices. Some work might be needed to support the use of `std::map` with landmark objects. In C++, `std::map` is implemented using red-black trees, a form of self-balancing Binary Search Trees.

Each particle will take ownership and responsibility of `K` EKF instances given `K` landmarks, and each particle, given a measurement and process input, will update the robot pose and EKFs. The particle filter is agnostic to the source of these inputs. It is up to the FastSLAM system to feed process and measurement input to the particle filter at the correct timing.

Public functions of this class include:
- `matchLandMark`: match the observation to landmark given the MLE weights calculated by EKF instances
- `particleUpdate`: function to update all particles given process and measurement input.
- `particleRegen`: Resample the auxiliary set of particles based on the probabilities proportional to weight.

### V. FastSLAM
This is the general main driver class that ties all components of the FastSLAM together as well as interfaces with other services. This class remains a bit undefined for now, since we are unsure of what supporting services we want with the same. Regardless, this class should initialize all components of the FastSLAM, configure initial conditions, and use the components to carry out the state estimation.

## 4. Notes

### I. Suggested Programming Workflow

Following the principles of test-driven development, the workflow to developing this code base should be as follows:
1. **Define interfaces:** define the function and class prototypes 
2. **Develop tests:** write extensive unit tests for the functions and classes in focus
3. **Develop code:** only when all unit tests for the module are finished (and merged into dev!), development work of the actual component can begin 

### II. Testing Guidelines
TODO: Unit test + robot test


## 5. References

- Montemerlo, Michael, et al. "FastSLAM: A factored solution to the simultaneous localization and mapping problem." Aaai/iaai 593598 (2002).
- Thrun, Sebastian, et al. "Fastslam: An efficient solution to the simultaneous localization and mapping problem with unknown data association." Journal of Machine Learning Research 4.3 (2004): 380-407.
- Bagnell, Drew, et al. "Importance Sampling and Particle Filters, Statistical Techniques in Robotics (16-831, F14) Lecture#04 (Thursday September 4)"
- Thrun, Sebastian, et al. Probabilistic Robotics. The MIT Press, 2005. 
