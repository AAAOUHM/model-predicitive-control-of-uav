This is an MTech Thsesis project about finding the least energy path for the UAV quadcopters using **PONYTRYAGINS PRINCIPLE** to solve fot the **OPTIMAL CONTROL LAW** .  Then we design the closed loop control using model predictive controller. We can see in the report that  linear parameter varying MPC (**LPV-MPC**) waasn't performance wasnt great .
So next our quadcopter is controlled by Nonlinear mpc (**NMPC**) where the weights are tuned by **genetic algorithm** such that energy consumption for a mission completion is minimized/optimized. We have then experimented with various versions of the MPC objective functions.In all those cases we got far better results than linear mpc(LPV-MPC). Finally we selected the best objective function along  with tuned parameters that are giving best results in tracking . 
 A short ppt is there to refer to get a **HIGH LEVEL OVERVIEW**.
All codes for the nonlinear mpc are given Iin following: 
1) First u need **GPOPS** software to run the trajectory generation code(**leastenergymain.m**) .
2) Second run the genetic algorithm (**ga_code.m**) to get the optimal parameters.(Q,R weights and N(prediction horizon)) .
3) Finally u need **CASADI** software to the  run (**mtpmpc3tuning.m**) code (controller) and check the drone's trajectory . 
