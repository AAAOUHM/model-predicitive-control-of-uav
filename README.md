This MTech thesis project focuses on determining the minimum-energy trajectory for UAV quadcopters by applying Pontryagin's Principle to derive the optimal control law. Subsequently, a closed-loop control system is designed using model predictive control (MPC). As detailed in the report, the linear parameter-varying MPC (LPV-MPC) exhibited suboptimal performance.
To address this, the quadcopter is controlled via nonlinear MPC (NMPC), with the weighting parameters optimized through a genetic algorithm to minimize energy consumption while ensuring successful mission completion. Various formulations of the MPC objective function were evaluated, all yielding significantly superior results compared to LPV-MPC. Ultimately, the most effective objective function was selected, along with finely tuned parameters that deliver optimal trajectory tracking.
For a high-level overview, refer to the accompanying short PowerPoint presentation.
The codes for implementing the nonlinear MPC are provided as follows:

1.Use GPOPS software to execute the trajectory generation code (leastenergymain.m).
2.Run the genetic algorithm (ga_code.m) to obtain the optimal parameters (Q and R weights, along with the prediction horizon N).
3.Finally, employ CASADI software to run the controller code (mtpmpc3tuning.m) and visualize the drone's trajectory.
