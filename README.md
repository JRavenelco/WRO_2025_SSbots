# Introduction

We are the SSBots team from Mexico, competitors in the WRO Future Engineers category. This is our first year competing in this category, exploring further into WRO after competing for years in the Robo Mission Category. The purpose of this repository is to document our efforts during the 2025 season.

## The team
>Team members
- Sebastian Esquivel
- Santiago Mansilla
>Coach
- Luis Ernesto Fernandez Rodriguez

## Past experiences

  **Our team has participated in WRO Robo Mission competitions for a few years,** as to explore further into new challenges like autonomous vehicles and artificial vision,  we could carry over developments from **RoboMission** . Some of the topics were:
  - Vehicle design and instrumentation, (embedding sensors and actuators while designing purpose built hardware for the specific task.
  - Coding experience, like making a framework, carrying over and porting Python and C code for the LEGO Mindstorms.
  - PID control, implementation and tuning.
  - Path planning, obstacle detection and avoidance. 

Even though we faced multiple difficulties with development and integration of new technologies onto our Robot, while exploring onto the use of electronics for hardware design and training our own AI model for real-time vision, object detection and labeling, and decision making, in addition to all the difficulties that are still to come, we have managed to overcome these difficulties and solve this challenges. We plan on innovating and improving even further before participating in Mexico´s national WRO Future Engineers 2025 competition.

# Engineering Materials

This repository holds the engineering materials developed for a self-driving vehicle with Ackerman drive train for the WRO Future Engineers competition for the 2025 season, documentation will be updated as development continues.   

# Content

* `t-photos` includes team photos during the weekly development workshops
* `v-photos` includes photos of the vehicle, for reference all sides are documented along with a 3D model rendering.
* `video` includes test videos of the vehicle in action and of the development process.
* `schemes` contains schematic diagrams of the electromechanical components, connection diagrams and list of components used in case any other team wants to take inspiration or reproduce the vehicle after the season.
* `src` contains the control software code for all components programmed for the competition.
* `models` contains 3D models of custom components developed, all can be fabricated by use of a 3D printer or laser cutter. As reference a Brick link Studio Model and the STL files for the Technic compatible elements is included.
* `other` contains additional reference files and documentation used for development.

# Design
Our robot design for this competition started with the base platform, at first we thought about using a prebuilt chassis with integrated motors as those found [online](https://www.amazon.com.mx/Yahboom-completo-neum%C3%A1ticos-superiores-opcionales/dp/B0BR9PGZWN?th=1) since this would solve the issue of mechanical design and more time would be allocated onto software development and integration of the other hardware components. As we progressed we realized that we would still require the design to include more custom components and mounts for the hardware that was not designed for it.
In addition, the back traction system on the previous prebuilt chassis was operated by two direct coupled motors on each side of the vehicle to provide the forward and backward propulsion with the shift in direction provided by a third motor which control the [Ackerman steering](https://en.wikipedia.org/wiki/Ackermann_steering_geometry). According to the WRO rules for the 2025 season:

> 11.12. Teams can use any electrical DC motors and/or servo motors of their choice – there are no restrictions on brand of motors and/or servos used.
> 11.13. A maximum of two motors may be used to make the vehicle move forward or backward (i.e., driving the robot, these are the driving motors). The driving motors must all be connected directly to the axle turning the wheels, or indirectly through a gearing system. The two driving motors may not be connected independently of each other to the driving wheels.

That´s why after some more design and brainstorming sessions we concluded we would require more work than initially planned. But still, this kind of design and system to mount the components was considered a great idea, which could be replicated with our own design.

![Ackerman Chassis](image.png)

## First design Iteration (Full LEGO Chassis)
One of the skills we have developed from our time in Robomission is the design of custom structures and mechanisms with the LEGO Technic system. Even though it is limiting because of how tight some components can be when integrated, and the pieces are predesigned, it allows for a standard system to be employed for integration with the definition of base Technic units well documented and the use of many prebuilt elements for rapid prototyping.
[https://i.sstatic.net/83RL2.png](url)
This left us with the decision to either integrate custom or commercial electronic components or use those available in the LEGO Mindstorms system. We decided to implement two strategies:
- **Short term:** use the LEGO Mindstorms components to have an available robot platform to progress with software development.
- **Explore the development of custom hardware:** for electronics (power delivery and progress onto the embedded systems) and mechanical components to integrate in future iterations of the robot.

This iteration of the design was built onto a custom LEGO Technic platform with Ackerman steering with one [Technic Large Angular Motor](https://assets.education.lego.com/v3/assets/blt293eea581807678a/bltb9abb42596a7f1b3/5f8801b5f4c5ce0e93db1587/le_spike-prime_tech-fact-sheet_45602_1hy19.pdf?locale=es-mx) for propulsion and one [Technic Medium Angular Motor](https://assets.education.lego.com/v3/assets/blt293eea581807678a/blt692436dd1e8fa71c/5f8801d5c8a27c1d9614c27e/techspecs_technicmediumangularmotor.pdf?locale=es-mx) for steering, initially we planned to use one Technic Small Angular Motor, but additional torque was required by the transmission. For reference the website [Philo Home](https://www.philohome.com/motors/motorcomp.htm) provides a great reference and comparison of the entire catalog of LEGO motors available. Sensing for obstacles is handled by an array of sensors similar to the strategies currently used in commercial vehicles:
- Short range with the use of [Technic Distance Sensors](https://assets.education.lego.com/v3/assets/blt293eea581807678a/blt64c2b9534cf10f68/5f8801b8bc43790f5c4389ea/techspecs_technicdistancesensor.pdf?locale=es-mx) .
- Front range mapping with the use of [LiDAR](https://www.slamtec.com/en/lidar/a1).
- Obstacle and object detection with the use of a [camera](https://docs.luxonis.com/hardware/products/OAK-D%20Lite).

The camera from Luxonis was selected due to the prospective capabilities it presents. The embedded TPU Mobidius from Intel allows to carry out the processing directly on the device reducing the workload on the Raspberry Pi and allocating resources better designed for this task. The camera also includes and array of three lenses with two being monochromatic for stereo vision and depth perception and the third for color vision. This results in the data being represented in a 4D array for RGB and the distance value for the object present in each pixel. Each sensor is specialized for certain operating conditions covering the weaknesses of each other and providing possible fallback options in case one is not operating properly (to be implemented).

The compute and processing side is handled by a Master-slave controller with the [Technic Large HUB](https://assets.education.lego.com/v3/assets/blt293eea581807678a/bltf512a371e82f6420/5f8801baf4f4cf0fa39d2feb/techspecs_techniclargehub.pdf?locale=es-mx) in charge of the lower level control and interface onto the rest of the Mindstorms components, executing commands received through a serial communication using a USB link in interpreter mode and a Raspberry Pi 5 in charge of data processing and decision making.

Power delivery is handled by a combination of the integrated power delivery network of the Large HUB and a custom one with a 4S Li-Ion battery pack and a commercial regulator from [Traco Power](https://www.tracopower.com/int/series/tmdc-20) to provide a steady 5V for the Raspberry Pi and the camera.
Based on our tests for power consumption each battery pack with 2200mAh is capable of providing a runtime of 3 hours with the steady state current consumption of 650mA and up to 1 hour with full load at the average peak current consumption of 1880mA. This would enable us to go through a round without having a degradation of performance due to the battery discharge.

## Second Design Iteration
After the regional friendly competition, checking out the other teams designs and progressing more onto the challenges we could not solve at that time, we decided to completely redesign the robot. We needed to asses the simultaneous object detection and navigation aspects for mapping the location of the objects not just detecting them with the camera and improve on the weight distribution and traction system. We decided to include a LIDAR sensor at the front and TOF sensors arround the robot to map the objects in the track and detect the proximity of the walls while reverse parcking. For the mechanical redesign the parts where iteratively customized as to optimize weight distribution and remove unnecesary weight and volume from the robot.

The key aspect for this redesign is a unibody chassis onto which all the other components are mounted, this allowed to have a lighter design with only the required mounting points and customized for each of the components.
TODO: IMAGE UNIBODY CHASIS
For this point we initially started by recerating in CAD the main component 



## Teamwork

We usually met up once a week to work on this project, each taking on one part of the project to develop, even if we were not all available, we established targets and tasks to achieve for the current and next couple of weeks. We strived to have everyone involved on all areas of the project, with each dedicated onto the parts they were the most familiar or wanted to further explore and the rest supporting and keeping on the loop as to have the complete picture of the development process. Efforts were documented locally on the computers dedicated for each teams use and pushed onto **GitHub** when significant advances were made or a stable version was reached and frequent communication over the teams chat group. As each part needed to be integrated onto the robot, all our efforts needed to be integrated as a team.  
  
# Conclusion
Looking back on this project, **it’s crazy to see how much we’ve learned.** We started off feeling a little lost, but as we worked through the challenges, we gained a better understanding of how to use these new technologies and integrating them together. We definitely faced some tough moments, but pushing through those helped us grow as a team and as individuals. Even though there’s still a lot more to learn, we’re proud of what we accomplished and excited to take what we’ve learned into future projects.
