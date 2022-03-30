# Swarm-box-pushing-Webots-Simulation 

This is a work in progress.</br>
The goal of this project is to implement Collaborative Box pushing using multiple "e-puck" robots.</br>
The robots of a swarm robotic system are relatively simple and
incapable when compared to the tasks that they are expected to accomplished. Thus, it is important to have
enough units and a distributed control law that is effective.
Because the individual behaviour of social insects often inspires the control law design in swarm robotic systems, the
control law of individual robots is usually referred to as individual behaviour. The behaviours of the robots are usually
identical and make use of only local information. Combining
these features, a swarm robot system offers potential advantages in robustness, 
exibility and scalability.</br>
All robots share the same controller hence behaviour based control approach has been used,  each robot recognises which state it is in and then decides which behaviour to follow,

The algorithm is as follws </br>
![FlowDiagram png](https://user-images.githubusercontent.com/15308488/160833026-b45e6626-4e49-4df2-b2a1-129522b20c47.jpg)</br>
Current state of project : </br>
Each robot can identify and distinguish between other robots, objective and goal.</br>![Screenshot 2022-03-30 140915](https://user-images.githubusercontent.com/15308488/160834348-6ea4d97f-99ea-4da3-b569-52c64b7bf1f5.jpg)</br>

Each robot uses distance sensors to avoid collision with other robots.</br>
Each robot can align itself to the box.</br></br>


Future work :</br>
swarm formation around the box needs to be worked on. each robot should be able to find a vacant spot around the object and align itself to push the box.</br>
