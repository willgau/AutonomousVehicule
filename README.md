This program control an autonomous vehicle in a multi-core & multi-thread environment.
The purpose of the vehicle is to take photo of the environment, here the genMap generate the image rendering and the output of the road the vehicle took while traveling.
A* algorithm was implemented to avoid obstacle that will be discovered by the vehicle, it does NOT know the location in advance. 

An infinite number of vehicle can be implemented but I had access to 8 core, it's optimize for 8 vehicles.
