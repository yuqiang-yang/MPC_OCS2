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
