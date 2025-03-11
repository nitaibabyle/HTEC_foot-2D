In the construction of the Hinge Tenson Elastic Complex (HTEC) foot structure, we develop a fully passive foot dynamics environment. The code serve as a 2D example illustrating the mechanism of this foot structure.
Three sets of stiffness parameter was used in the foot, so that we can compare the difference of the foot performence.
We utilized three sets of stiffness parameters within the foot to enable a comparison of performance differences. The balance between stability and adaptability is a crucial consideration for a foot structure:

A foot that is too rigid will excel in stability, offering excellent body balance capabilities. However, it may lack the flexibility required for deformation and shock absorption.
Conversely, a foot that is too soft will prioritize adaptability, deforming effectively when encountering complex terrain. Yet, it might struggle to adequately stabilize the body weight.

We proposed a optimization method to solve this trade-off. (in: https://doi.org/10.1016/j.dt.2024.08.010)

It is noted that the code committed here not including the optimization code. There are just the 2D structure kinematics and dynamics, and the simulation enviornment in 2D plane.

You can tune the spring parameters and try to find the best configuration. (Although it is difficult fot tuning manully...) 

Here is the example:
|Soft parameter|Hard parameter|Almost optimal|
|---|---|---|
|<img src="https://github.com/user-attachments/assets/d77733d0-05e3-40ac-8af9-372c10b08cfb" width="200px" />|<img src="https://github.com/user-attachments/assets/391c77f8-33ad-4020-abf8-0ba8add52f83" width="200px" />|<img src="https://github.com/user-attachments/assets/ebc6f740-7ee7-4cb5-9e4c-60da2cb9a4da" width="200px" />|
