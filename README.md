Hinge Tenson Elastic Complex (HTEC) foot, we construct the dynamics. This is a full-passive foot structure. 
This is an 2D example, which can show you the mechanism of the foot structure.
Three sets of stiffness parameter was used in the foot, so that we can compare the difference of the foot performence.
Stablity and the Adapatability is a trade-off as for a foot structure:
The foot too hard will lead to good stable, it will have a good capability in balance the body, but it will not have the ability to deformation and shock absorbing. 
The foot too soft will lead to good adapatability, it will deform when stepping in to complex terrain, but will not to have good ability to stablize the body weight. 

We proposed a optimization method to solve this trade-off. (in: https://doi.org/10.1016/j.dt.2024.08.010)

It is noted that the code committed here not including the optimization code. There are just the 2D structure kinematics and dynamics, and the simulation enviornment in 2D plane.

You can tune the spring parameters and try to find the best configuration. (Although it is hard to tuning manully...) 

Here is the example:
|Hard parameter|Soft parameter|Almost optimal|
|---|---|---|
|<img src="https://github.com/user-attachments/assets/391c77f8-33ad-4020-abf8-0ba8add52f83" width="200px" />|<img src="https://github.com/user-attachments/assets/d77733d0-05e3-40ac-8af9-372c10b08cfb" width="200px" />|<img src="https://github.com/user-attachments/assets/ebc6f740-7ee7-4cb5-9e4c-60da2cb9a4da" width="200px" />|
