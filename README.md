# This is a MPC controller project for mobile manipulators.

## working plan
### 2023.1.30   
+ add the loadMap and SaveMap ros service. For simplication, the two functions only work when using only the array to save the map!!! 
+ Problems: The MPC seems to be weak. It cannot handle hard task and find a reasonable solution. If you give a target position in the sideway, it will probably be stuck near the obstacles.


