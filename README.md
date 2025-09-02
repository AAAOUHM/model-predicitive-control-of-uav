# MTech Thesis Project: Energy-Optimal UAV Quadcopter Control

This project focuses on determining the minimum-energy trajectory for UAV quadcopters using **Pontryagin's Principle** to derive the **optimal control law**. A closed-loop control system is then designed using **Model Predictive Control (MPC)**.

Initial experiments with **Linear Parameter-Varying MPC (LPV-MPC)** showed suboptimal performance. To improve this, we implemented **Nonlinear MPC (NMPC)**, optimizing the weighting parameters with a **genetic algorithm** to minimize energy consumption during mission completion.

Multiple formulations of the MPC objective function were tested, all outperforming LPV-MPC. The best objective function, along with tuned parameters, was selected for optimal trajectory tracking.

For a high-level overview, refer to the accompanying [PowerPoint presentation](link-to-ppt).

## Code Implementation

To run the nonlinear MPC, follow these steps:

1. Use **GPOPS** software to execute the trajectory generation code (**leastenergymain.m**).  
2. Run the genetic algorithm (**ga_code.m**) to obtain optimal parameters (Q and R weights, and prediction horizon N).  
3. Use **CASADI** software to run the controller code (**mtpmpc3tuning.m**) and visualize the drone's trajectory.
