# This is a MPC controller project for mobile manipulators.

## working plan
### 2023.1.30   
+ add the loadMap and SaveMap ros service. For simplication, the two functions only work when using only the array to save the map!!! 
+ Problems: The MPC seems to be weak. It cannot handle hard task and find a reasonable solution. If you give a target position in the sideway, it will probably be stuck near the obstacles.

### 2023.1.30   
+ Install the trajOpt, the main difficulty of which exist in the installation of OPENRAVE( use the Installation Script rather than the github source). The website of trjaOpt says that the openrave version should be older than 0.8. But if you install the OpenRave through installation script, the version for 18.04 is 0.53. I dont know whether there exist some bug. 
+ It use the V-HACD library to get the convex decomposition from a mesh.

## Bug report
+ ImportError: dynamic module does not define module export function (PyInit_ctrajoptpy) 
    + `.bss' can not be used when making a PIE object; recompile with -fPIC -> set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie")

    + use the nm -C ctrajoptpy.so grep | nit. And we can see that the xxxinit funcition have a prefix t not T. That means the symbol is hidden. So...
    + change  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hiddem") to  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=default")
    + But annother error occurs.TypeError: No registered converter was able to produce a C++ rvalue of type std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > from this Python object of type unicode
    + It seems that the bug exist in "string pyversion = py::extract<string>(openravepy.attr("__version__"));". Just comment it.
### 2023.1.30   
+ change the OMPL frontend to multiobject of clearance and distance. Change the range to a fixed size( This will accelarate the solution finding time).
+ unfortunately, FIESTA can only construct the ESDF around the obstacles. The ESDF and distance_buffer of the place that is far from the obs.( just about 5cm) is UNDEFINED. So the MPC planner cannot acheive the plan because it would react to the obs. only when the arm really close to them.
+ The ESDF of faster-planner is built based on . F. Felzenszwalb and D. P. Huttenlocher, “Distance transforms of sampled functions,” Theory of computing, vol. 8, no. 1, pp. 415–428, 2012, which is just the approach CHOMP uses. 
    + I think, compared to FIESTA, this approach is time-comsuming with time-complexity O(n^3) because it compute ESDF again and again from scratch, while the FIESTA incrementally update ESDF. Howerver, the EDT approach can natually get the whole ESDF in the range of interest.
    + For more details, more experiments should be done later. 
