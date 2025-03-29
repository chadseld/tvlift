# tvlift

This initial commit of the TV Lift firmware is uncompleted. The positioning algorithm does its best to calculate position and velocity, but is doing so in a way that results in oscillation around the goal position.

The hall effect shaft encoders are configured to trigger interrupts on the Arduino. However, at full speed some number of these signals are missed. Is an interrupt fired during interrupt handling?? This causes the known position to drift from the actual position over time.

Improvements are needed to both of these issues. The homing algorithm can be improved by using a Kalman filter.

The TV Lift hardware itself is a bit of a mystery. Though there are is a shaft encoder, there are no limit switches. The home position is found by driving the mechanism past the hardware limit and watching for motor stall (this is wonderful for the gears). The extended limit position is found by running the mechanism all the way out until your lift separates and your TV falls to the floor. (I don't actually know if the lift will separate past the full extension position -- I've not been brave enough to find out.)