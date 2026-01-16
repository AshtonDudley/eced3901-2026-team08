Group 8

Commands for operation

Start Robot
colcon build

```bash
ros2 launch dalmotor robot.launch.py
ros2 launch eced3901 lab5dt2.launch.py
ros2 run eced3901dt1
```

Test of DT1 done on 16/01/2026
Ashton, Lorne, Lucas, Liam
Test #1 - Speed Turn Radius set as same
Test #2 - Speed Turn Radius + Speed increased
 - > x_vel = 0.5, th_vel = 0.5
Speed was too fast on the straight line, overshot

Test #3 - Speed Turn Radisu + Speed decreased
 - > x_vel = 0.3, th_vel = 0.3

Saw a lot of overshoot/off angle with faster speed, will need ramping speed to not have this issue
