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
    + use the nm -C ctrajoptpy.so grep | nit. And we can see that the xxxinit funcition have a prefix t not T. That means the symbol is hidden. So...
    + change  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hiddem") to  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=default")
    + But annother error occurs.TypeError: No registered converter was able to produce a C++ rvalue of type std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > from this Python object of type unicode

