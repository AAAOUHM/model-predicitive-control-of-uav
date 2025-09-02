# MTech Thesis Project: Energy-Optimal UAV Quadcopter Control

This project focuses on determining the minimum-energy trajectory for UAV quadcopters using **Pontryagin's Principle** to derive the **optimal control law**. A closed-loop control system is then designed using **Model Predictive Control (MPC)**.

Initial experiments with **Linear Parameter-Varying MPC (LPV-MPC)** showed suboptimal performance. To improve this, we implemented **Nonlinear MPC (NMPC)**, optimizing the weighting parameters with a **genetic algorithm** to minimize energy consumption during mission completion.

Multiple formulations of the MPC objective function were tested, all outperforming LPV-MPC. The best objective function, along with tuned parameters, was selected for optimal trajectory tracking along with minimal energy consumption.

For a high-level overview, refer to the accompanying [PowerPoint presentation](https://view.officeapps.live.com/op/view.aspx?src=https%3A%2F%2Fraw.githubusercontent.com%2FAAAOUHM%2Fmodel-predicitive-control-of-uav%2Frefs%2Fheads%2Fmain%2Farya_mtp_iitg_FINAL_PPT.pptx&wdOrigin=BROWSELINK).

## Code Implementation

To run the nonlinear MPC, follow these steps:

1. Use **GPOPS** software to execute the trajectory generation code (**leastenergymain.m**).  
2. Run the genetic algorithm (**ga_code.m**) to obtain optimal parameters (Q and R weights, and prediction horizon N).  
3. Use **CASADI** software to run the controller code (**mtpmpc3tuning.m**) and visualize the drone's trajectory.
