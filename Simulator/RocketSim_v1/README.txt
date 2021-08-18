RocketSim_v1 USER GUIDE

This simulator uses MATLAB's ODE45 solver to propagate the translation and rotation of a rocket in flight over a total simulation time with specified initial conditions.
This guide details how to set up the initial conditions, run a simulation, and what scripts to modify to change the rocket's behavior.

1. Determine the rocket and environment properties from CAD models and other methods. All units are in metric.
     -Open quadParamsScript.m to view/change existing rocket parameters
     -The relevent parameters range from the mass to the static margin and are detailed below:
	-quadParams.m = mass: determine from OpenRocket or whatever the design mass of the rocket is. Currently set to be constant.
	-quadparams.Jq = moment of inertia matrix: generally the moment about the roll axis (x-axis) is smaller than the moment about the pitch and yaw axes (y- and z-axes). Pitch and Yaw moments would be equal if the rocket were completely symmetric. Off-diagonal components are generally non-zero but should be much smaller than the components on the diagonal. Determine from CAD
	-quadparams.Ad = area: a common reference area is the circular area at the largest diameter of the rocket body. (A = pi*(dmax^2)/4). Axial and normal coefficients may change depending on the reference area used
	-quadParams.Cd = coefficient of axial force: determine from hand calculation from standard bodies (Borrowman analysis; only valid at low speed) or from simulation (preferred), or from flight data.
	     -Currently taken from RasAero CFD simulation. The TestCoeff.m script loads the data from the FCD rocket RasAero coefficients excel spreadsheet and turns the data into a form that the simulator can use.
	     -Currently the Cd data at each angle of attack is averaged at each Mach# to get one Mach# vs Cd curve, up to Mach 2 (limit for this rocket). This is because small angles of attack hahave a weak effect on the Cd
	-quadParams.Cnfit = coefficient of normal force quadratic fit: determined by same method as axial coefficient
	     -Since angle of attack and Mach# greatly effect on the Cn, a cirve is fitted at each Mach# to get a relationship between AoA and Cn. Quadratic fit was chosen so that the Cn would increase dramatically at higher AoAs. Linear is more valid at small AoAs. May try exponential fit for better accuracy
	     -Each row in the Cnfit matrix contains the coefficients of a quadratic funtion that relate the AoA to the Cn at the Mach# for that row. Starts at M = 0.01.
	-quadParams.StaticMargin = static margin: absolute distance from the center of mass (CM) and the center of pressure (CP). Aerodynamic forces are chosen to act through the CP. abs(Xcm - Xcp)
	     -In OpenRocket and other programs, can be determined from the "Stability" parameter which is the static margin divided by the max diameter of the rocket and is measured in "calibers". For example, a rocket might have a stability of "2 cal".
     -Engine thrust data may be imported as a "disturbance force"
	-The interpThust.m function interoplates the thrust curve given by the data in Cesaroni 15227N... txt file so that the thrust magnitude at each time step can be found.
     -Open constantsScript to view/change commonly used constants.
     -The sensorParamsScript.m script contains parameters for the sensors (IMU, GNSS, ect.) but is currently unused since no sensor models were developed for this version of the simulator. We are using the true state of the rocket.

2. Determine the initial conditions based on what kind of analysis you want to perform.
     -Open topSimulateControl.m to view the initial conditions which are described below:
	-All values are in metric
	-tic/toc is a MATLAB function that outputs how long it takes to execute the code between tic and toc
	-Tsim = total simulation time: For this rocket, the time to apogee is ~40s. We only care about rocket dynamics up to apogee.
	-delt = update interval (time step): how often is the state calculated from dynamics described by the system of equations? Must be small for numerical computation and accuracy. Currently 0.005s to capture steep increase in thrust at launch
	-N = how many time steps from 0:Tsim
	-tVec = time vector
	-Ignore reference trajectory for now. Important for controlling canards which have not been implemented
	-Tcurve = load thrust curve from file. Currently assumes that launch is at t=0. If checking behaviour after burnout, multiply the thrust by 0.
	-"S" structure is the initial state of the rocket, inertial reference frame
	-S.distMat = disturbance matrix. Currently used to simulate uni-axial thrust. Unlike the other componets of "S", this gives the disturbance force for all time simulated
	-S.state0.r = initial position
	-S.state0.e = initial attitude, given in Euler angles. Can set an initial launch angle with yaw and pitch.
	-S.state0.v = initial velocity: set to 0 to simulate from launch. Can give the rocket an initial velocity to check behavior in-flight. Be sure to set the thrust to 0 if evaluating after burnout.
	-S.state0.omegaB = initial angular velocity: currently unsure if [a b c]' angular vel corresponds to inertial [x y z]' axes or body [x y z]' axes. Results only makes sense if [a 0 0]' is given. Need to determine if dynamics are correct
	-S.oversampFact = oversampling factor: further divides the time step by this factor. Reduces error by calculating the state at an intermediate time step. For example, with a factor of 2 instead of calculating the state from T=10:10.005s, it calculates from T=10:10.0025 then T=10.0025:10.005s. Note that the intermediate state is not recorded.
	-S.rXIMat = feature locations: important for camera simulation, not relevent here
	-"P" structure contains all of the rocket, sensor, environmental, and constant parameters

3. Run the simulation by pressing the green play buttion at the top in the editor tab
     -"Q" structure contains the state of the rocket at every point of the simulation and is filled throughout the simulation. Data analysis uses this "Q" structure
     -The rest of the script displays the results of the simulation and outputs some metrics to SimOut.txt. This file is overwritten each time so close it before every run.

4. Evaluate the performance of the rocket
     -If the trajectory or attitude of the rocket seem fishy, check your initial conditions and parameters.
     -Verify if altitude and max velocity match that of other simulations
     -If the initial conditions and parameters are fine but it still seems erroneous, then the differential equations are likely erroneous. Proceed to the next step

5. Modify the rocket dynamics functions
     -Open quadOdeFunctionHF.m
     -This script contains all of the differential equations used to calculate the state at each time step.
     -Diagnose
     -This section of the guide will be expanded on soon.



