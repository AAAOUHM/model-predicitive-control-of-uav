This MTech project is about the UAV quadcopters control using model predictive control. the mtp progress report has been uploaded where linear mpc was used where performance wasnt great .
so the final report will soon be uploaded where a quadcopter is controlled by nonlinear mpc and the weights are tuned by genetic algorithm such that energy consumption for a mission completion is minimized/optimized. we get much better results than linear mpc.
all codes for the nonlinear mpc are given:
1) first u need gpops software to run the trajectory generation code(leastenergymain.m) .
2)run the genetic algorithm (ga_code.m) to get the optimal parameters.(Q,R weights and N(prediction horizon))
3) then u need casadi software to the  run (mtpmpc3tuning.m) code (controller) and check the drone's trajectory. 
