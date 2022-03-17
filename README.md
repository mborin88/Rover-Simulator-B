# Rover-Simulator-Zero
A rover simulator for hilly forest environments
This repository Rover Simulator Zero is a swarm rover simulator designed for hilly, 
forested environments where swarm members communicate via LoRa PHY.

The directly runnable script 'rover_swarm_simulation.py' serves as the main function
where the simulation runs and the interface where users can change some settings.

The changebale settings to the user are introduced below. For any questions, please 
look into the dissertation written by the author Yishi Fu, supervised by Dr. D. Tarapore, 
"A simulator of sparse rover swarms in communication-constrained forest environments".

Third party library requirments: Numpy, Matplotlib, Pillow.

'area' - Simulation area.

'N' - Total rover numbers.

'dist' - Distance between rovers.

'x_offset' - Distance from the left boundary which is the location of the first rover.

'y_offset' - Distance from the baseline which is the location of the first rover.

'goal_offset' - Distance to the goal below which the goal is assumed reached.

'steps' - Maximum time steps.

't_sampling' - Sampling period.

'len_interval' - Interval between transmissions.

'user_f' - Center frequency of carrier.

'user_bw' - Bandwidth.

'user_sf' - Spreading factor.

'user_cr' - Coding rate.

'user_txpw' - Transmitting power.

'Q' - The state noise.

'R' - The measurement noise.

'ctrl_policy' - The control policy used (0, 1, 2).

'K' - Gain of controller.

NOTE: Landcovers are loaded from top left to bottom right, so when we load landcover map columns
and rows are inverted. e.g. if position = (x, y) 
Then if we reference the data directly from the landcover map then we refernce .data(y, x)
However this is already accounted for in the get_data function where easting and northing are swapped. 
Representing this switch the variable names are psuedo_northing, and psuedo_easting
So we can normally call this function as get_data(x, y)
