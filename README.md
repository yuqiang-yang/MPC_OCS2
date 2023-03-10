# This is a MPC controller project for mobile manipulators.

## working plan
### 2023.1.30   
+ add the loadMap and SaveMap ros service. For simplication, the two functions only work when using only the array to save the map!!! 
+ Problems: The MPC seems to be weak. It cannot handle hard task and find a reasonable solution. If you give a target position in the sideway, it will probably be stuck near the obstacles.

### 2023.1.31
+ Install the trajOpt, the main difficulty of which exist in the installation of OPENRAVE( use the Installation Script rather than the github source). The website of trjaOpt says that the openrave version should be older than 0.8. But if you install the OpenRave through installation script, the version for 18.04 is 0.53. I dont know whether there exist some bug. 
+ It use the V-HACD library to get the convex decomposition from a mesh.

## Bug report
+ ImportError: dynamic module does not define module export function (PyInit_ctrajoptpy) 
    + `.bss' can not be used when making a PIE object; recompile with -fPIC -> set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie")

    + use the nm -C ctrajoptpy.so grep | nit. And we can see that the xxxinit funcition have a prefix t not T. That means the symbol is hidden. So...
    + change  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hiddem") to  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=default")
    + But annother error occurs.TypeError: No registered converter was able to produce a C++ rvalue of type std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > from this Python object of type unicode
    + It seems that the bug exist in "string pyversion = py::extract<string>(openravepy.attr("__version__"));". Just comment it.
    + No handlers could be found for logger "openravepy.ikfast" -> it may be a problem in sympy. Check here and install.
    + The imported target "vtkRenderingPythonTkWidgets" references the file "/usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so" but this file does not exist. -> locate the library and make a softlink.
    + comment VTK_GRAPHICS_EXPORT in vtkQuadricDecimation2.h
    + SetInput() -> SetInputData()
    + cannot declare ‘::main’ to be a global variable. -> change main to another name e.g.main1
### 2023.2.1   
+ change the OMPL frontend to multiobject of clearance and distance. Change the range to a fixed size( This will accelarate the solution finding time).
+ unfortunately, FIESTA can only construct the ESDF around the obstacles. The ESDF and distance_buffer of the place that is far from the obs.( just about 5cm) is UNDEFINED. So the MPC planner cannot acheive the plan because it would react to the obs. only when the arm really close to them.
+ The ESDF of faster-planner is built based on . F. Felzenszwalb and D. P. Huttenlocher, “Distance transforms of sampled functions,” Theory of computing, vol. 8, no. 1, pp. 415–428, 2012, which is just the approach CHOMP uses. 
    + I think, compared to FIESTA, this approach is time-comsuming with time-complexity O(n^3) because it compute ESDF again and again from scratch, while the FIESTA incrementally update ESDF. Howerver, the EDT approach can natually get the whole ESDF in the range of interest.
    + For more details, more experiments should be done later. 
### 2023.2.2  
+ fast-planner reading
    + How to guild the MPC solution like the path guided optimazation? If not, the MPC is easy to fail in the local minima. How to get a warmup trajectory? I think, the front-end need to be designed in detail instead of just using OMPL. Can an potential field or dynamical system help?
+ FIESTA Problem Solved
    + Change the code of FIESTA. When initialize an ESDF map, we set all voxels to free while the default value is undefined. By this, the FIESTA will update all voxel in the range and we can get the gradient and distance infomation of the place of interest.
    + The ESDF construction algorithm of Fast-planner is naive and it cannot handle a small resolution. I try a resolution of 1cm and 2cm, then the program crash. While in Fiesta, a resolution of 3cm in the range (-2.5,-2.5,-0.5)->(3.5,2.5.1.5) still works at 10 hz.
    + The VBD-EDT algorithm also incrementally update the ESDF and has a parameter to restrict the BFS depth. I guess it's also a good choice.
+ Not-Solved Problem : FrontEnd. The parameter of barrier function. The strategy of obstacles avoidance.
### 2023.2.3 
+ FrontEnd
    + THe RRT of OMPL is able to find a initial path in about 1ms. But the RRT* seems to be bad at the convergence(sometimes it cannot find a solution in 4s.) and the quality!!!

### 2023.2.7
+ install openrave 0.9.0 in 18.04
    + https://robots.uc3m.es/installation-guides/install-openrave.html#install-openrave-090-ubuntu-1804-bionic
    + Successfully install trajopt
### 2023.2.8
+ Openrave has a ik function manip.FindIKSolutions(T_w_ee, openravepy.IkFilterOptions.CheckEnvCollisions) that can return a collision free ik. I think it's based on ikfast, so it's not useful for manipulator with Dof > 7;
+ the collision checker of trajopt is done by openrave. The convex decomposition is pointcloud -> filtered -> mesh -> covex hull -> add to openrave. Handle static case. The GJK algorithm may be implemented by BULLET LIBRARY. It's too difficult.
+ PCL and Open3D implement the algorithm to convert pointclouds into convex hull through qhull(quick hull). But there is no convex decomposition.
+ sample in constraint configuration space: planning_with_approximated_constraint_manifolds(OMPL)
    + https://ros-planning.github.io/moveit_tutorials/doc/planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial.html
    + Offlinely construct a approximation graph(edge + vetex) that lies in the task constrains manifolds(different from collision avoidance space.). All edges and vetexes are constraint, the construction of which uses some tricks such as rejection sampling and Jacobian rejection.
    + The approximation graph can be used for multiple times because it will be same for different scene with same manipulators.
+ How does move it do self-collision checking?(without padding or inflation)
    + The fcl library has a Collision-manager, which use AABB for the collision for multiple objects(from rough to concrete). It can also check the collision or distance between two Collision-manager(such as robot links and the world obstacles.)
    + stl is a simple mesh file that only includes the geometry information while dae include the color, texture and so on.
+ The gradient infomation in trajopt is local-linearized(Assume that the contact point and the normal vector remain the same in an short peroids). I think it's not graceful way to handle the obstacles. It's also time-comsuming.

### 2023.2.10
+ Problem： The RRT star converge too slow and it seems not to optimize for a better solution at all
    + solved: The reason exists in the concept i misunderstand OMPL optimizingPlanner_->setRange. At first i consider it as a parameter that determines the resolution of the collision checker, so i set it to a relatively small value (e.g. 0.02). In this case the RRT*(or RRTX) is hard to optimize. Acually, the collision checker resolution is determined by the si_->setStateValidityCheckingResolution()[default 0.01]. So, just remove the sentence of optimizingPlanner_->setRange and keep the range to the default value.
+ change signed fiesta, but not tested

### 2023.2.13
+ Highly related work:A Collision-Free MPC for Whole-Body Dynamic Locomotion and Manipulation
    + Fiesta, FCL and OCS2. 
### 2023.2.14
+ Make some improvement to the project. In simulation it can finish the job.
    + wait for the FIESTA to be stable. Then start the MPC loop. Remember to reset the obseravation time before initial run MPC update(the first update determines the start of the horizon windows.) If not, the horizon time will be smaller than current time and the program cannot work
    + record the video of 0.4m/s
### 2023.2.15
+ tracIK and KDL. The core of these two algorithms is easy. That is, solve a optimization problem about the error in Cart. space while maintaining the joint limits contraint through NewTon's method(q_next = q + j^-1*error) or SQP(through NLopt). Their drawback lies in the optimality and the collision avoidance. TracIK return only one solution based on some metics(such as manipulability or speed). I think we also need to do collision check after a solution if found. Since it can found a solution in ms, the Collision-free-IK can be realized in realtime. For more detail:https://bitbucket.org/traclabs/trac_ik/src/master/trac_ik_lib/
+ Some bug:
    + sometimes the program crash because the rollout unstable or memory leak(?). I think the reason is that some component such as ESDF are not well-ready after we start the mpc update. When we turn on the manipulator cost or the esdf waiting, it rarely happens.
+ use sphere to tightly surround the mobile base and the end-effector(camera). The radius of the sphere is defined in the voxblox.yaml. The position is fixed in the URKinematics.cpp

+ Not solved problem:
    + the mpc are unstable when the esdf is updating.

### 2023.2.16
+ In realworld experiment, when we turn off the code start flag (It means the initial value of MPC will be value that are got from the last update. If we use cold start, the initial value of MPC will always be the cuurent joint configuration), the system will often be unstable. But this doesnot happen when we conduct the simulation.
+ In realword experiment, the MPC rate is low. It takes about 50ms one cycle. But if we use the same setting to conduct the simulation, only 20ms is needed.
+ the ostream object of C++ cannot be created. But we can create a stringstream object to initialize an ostream object. Then we can concat different string through <<.
+ OCS2 core use the CppAD CG to get the function value and jacobian. The CppADCG can load dynamicLib via a class and get the CppAD model about the cost value and jacobian. I think this facilates the extension of the program. If the cost functions(or system dynamics) don't change, we can reuse the library without recompilation. 
    + If want to use the dynamic weighting, we can change the getIntermediateParameters Function (including the template parameters about the parameter dims) to get a dynamic weighted cost.
+ Something that is hard to explain happens. When I wait for esdf stable in real experiment.  In about 1/2 cases the mpc update rate is slow(about 40ms), otherwise it's 16ms. The former cases are usually unstable. -> I change the execution order to make the MPC initilize at last and then the program seems to work well.
+ If we use coldStart, the mobile manipulator is hard to stay in a fixed point. It tend to oscillate around a point and gradually become unstable. Some i change to the warmStart mode and beg the mpc to work fine.

### 2023.2.17
+ If you start a thread in a function, you should use thread.join() or thread.detach(). Otherwise, when this function exits, segement fault will happen. Use thread.detach will make the thread independent. Until the main function return, the thread keeps working.
+ the static variable in the member function belongs to all instance of the class. They share the same static variable.

+ We'd better avoid change in the header file because it will take a much longer build time. 

+ Try to change the fiesta to multi-thread version. Fail!
    + The most time-comsuming part of FIESTA is updateESDF and the most time-comsuming part of the updateESDF is the updateQueue. So i try to change the BFS of updateQueue to accelelate. The problem lies in:
        + The queue in STL is not thread-safe. So we should first change to a thread-safe queue, which frequently lock the thread when push or pop from the queue. The lock will make the multi-thread slow because in most time the update procedure frequently pushes and pops element to the queue. The lock will make the multi-thread behave just like a single-thread.
        + If we don't lock the thread when push or pop from the queue, the program crash!!
        + After we add multi-thread, the time comsuption don't decrease in a satisfactory maner. Howerver, it will take more CPU resource.
### 2023.2.19
+ change graceful_mpc

### 2023.2.20
+ the mobile base will shift with a low velocity. And it's easy to oscillate.
+ If we change the system dynamics, it will be unstable. The problem remain in the CppAD sqrt.
+ with x^dot = x_b_dot, the non-ho.. constraint will not be satisfied. i.e. the y_dot/x_dot not equal to the tan(theta).

### 2023.2.21
+ Reading the papers about the replanning of the mobile manipulator
    + They need an initial trajectory in the configuration spaces. But it's quite difficult for the mobile manipulator with non-holonomic constraints to generate trajectory in realtime.
    + Why the condition i handle is difficult?
        + For many applications, the mobile manipulator only need to keep far away from the obs. But if you want to conduct specific tasks such as picking, you must approach the obs., which makes some costs have conflict with each other.
        + In the case of making the obstacles avoidance a hard constraint, the method is like a hiretical controller. We should ensure the safety first. But how to guide the manipulator out of the obs. ?
### 2023.2.22->26
+ learing MINCO 

### 2023.2.27
+ The incorrect initial value make the system become unstalble not the sqrt operate. Just make the x_dot a small value can make it work. But the system dynamics about x_dd = a_v cos y_dd = a_v sin don't have a phsical meaning. So i would change to the x y theta v w to be the state.
+ The FrontOrientation cost is not set correctly. Because i just use the UR state to get the cost, so when the mpc optimize the problem, it can only get the gradient of the UR state. So it would not make the orientation keep front in a wholebody-manner.
+ The planner cannot stop and the front orientation doesnot work.

### 2023.2.28
+ If the limits are enforced, the mobile manipulator will shake. I think the reason is that the velocity or accelation are beyond the range so the optimazer will pull them back!.
+ There always exist a contraction bettween the hard constraints and the soft constraints of the obstacle avoidance.
+ The mix between Matrix3Xd and MatrixX3d stuck me.

### 2023.3.1
+ The intial trajectory of TEB-Planner is from the PRM. It use PRM to connect the start and goal, generating N way point. The h-signature is used to check if a holologous path already exist in the database. At each iteration, it will reuse the PRM node by deleting some unuseful point( The edge should point from the start to the goal. And they must stay in the rectangle surrounded by the start and goal. The main problem is how to reuse the PRM map and the honologous path.
### 2023.3.6
+ I want to add the curveCost to penalize the high curtivature when turning( If the mobile manipulator turn in place, the curtivature will be infinite). But it's a cost related to the w^2/v^2. It's easy to be unstable beacause of the division to a small value when starting and ending.
### 2023.3.7
+ The dubin state space is a SE2 space. StateSpace <- compoundStateSpace <- SE2StateSpace <- dubins/reedSharp statespace. The stateSpace class of OMPL don't have the concept of steering function. It only need to define the distance function beween two states.(The distance in a compoundStateSpace is weighted by a cofficient. when we call the add compounent function we give the weight. And the final distance is calculated via the weighted sum of the distance in seperate stateSpace.)
### 2023.3.8
+ writing the code for the wholebody plan using geometric RRT. The state are compounded by ReedSheep and R(6).
+ writing the code for the wholebody plan using control SST. The state are compounded by SE2 and R(6);
### 2023.3.9
+ test the reedshepp FrontEnd. But it's still hard for it too find a path in realtime. The reedshepp statespace can find a initial solution quckily for the 2 dof plaform in serveral ms.

