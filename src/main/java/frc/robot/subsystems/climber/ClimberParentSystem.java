package frc.robot.subsystems.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LRSpeeds;

public abstract class ClimberParentSystem extends SubsystemBase {

    protected LRSpeeds speeds;
    protected boolean setSpeed = true;
    protected double pidOutput = 0;
    protected double feedForwardOutputs = 0;

  protected ClimberParentSystem() {
        speeds = new LRSpeeds();
    }

    public abstract LRSpeeds getSpeed();

    public abstract void setSpeed(LRSpeeds lrSpeed);

    public abstract void zero();

    public abstract void disableLimits();

    public abstract void enableLimits();

    public abstract void setLimits(
            float forwardLeft, float reverseLeft, float forwardRight, float reverseRight);

    @Override
    public abstract void initSendable(SendableBuilder builder);

    public abstract double leftPosition();

    public abstract double rightPosition();

    public abstract void leftPID(TrapezoidProfile.State state);

    public abstract void rightPID(TrapezoidProfile.State state);
}
