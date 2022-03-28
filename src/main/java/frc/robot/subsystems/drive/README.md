# Drive Subsystem
## Pid and Profiled control
### Linear (straight driving)
There are two PID controllers, one for the left and one for the right side motors.  These PID controllers are <b>incremental</b>.  That is, they measure
error between the current position and the <b>next</b> position, not the final position.  This affects the kP constant gain which would be greater in this case since
errors will be smaller in scale.

A feedforward object is provided for each side to provide open loop control so that the PID need only make minor adjustments to keep the encoders at the desired position.
To run both left and right motors with this incremental PID controller, the method <b>setPositionReferenceWithVelocity</b> is provided.  This method takes the desired position
and velocity and uses the feedforward and PID controllers to set the motor voltage.  Use of 0 velocity will remove the open loop control and put all the work on the PID controller
which will introduce lag.  This method is suitable for use with a trapezoid profile that a command may use to generate the position and velocity data points.

Correct feedforward tuning is important for trapezoid control to guarantee the drive will reach the desired position in the time alloted for the trapezoid.  You can verify the 
feedfoward tuning by watching the PID output during operation and seeing how much it needs to correct.  Tune the feedforward such that PID correction is minimal.

#### Units of measure
The drive motor encoders are configured to read out in meters and meters/s.  Note that setting the velocity conversion factor is separate from position because the encoders
natively read out revolutions per minute instead of second.  The code here just divides the position conversion factor by 60 to get meters/s instead of meters/minute.

### Turning
Turn PID control is also implemented with an <b>incremental</b> PID controller.  However, in this case, the drive subsystem does not know the measurement since that is in the gyro subsystem.  The command implementing the profiled turn must supply the measurement from the NavX subsystem along with the desired position (degrees) and velocity (degrees/s).  The 
drive subsystem has the tuning for PID and feedforward for turning and sends equal but opposite voltage to the left and right motors to achieve the turn.  This assumes that the left and right side will move at roughly the same speed to get a turn in place.  It may become necessary to have different turn feed forwards for left and right to maintain a turn in place, but it seems so far this is not necessary.

A method, <b>setTurnPositionReferenceWithVelocity</b> provides the access to the PID controller to implement a turn based on a position/velocity profile.

#### Units of measure
Turns are measured in degrees and degrees/s.
