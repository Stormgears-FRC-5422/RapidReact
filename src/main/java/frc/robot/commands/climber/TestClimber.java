package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimberParentSystem;
import frc.utils.LRSpeeds;
import frc.utils.joysticks.StormXboxController;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.kClimberSpeed;

public class TestClimber extends CommandBase {
  private final ClimberParentSystem subsystem;
  private final StormXboxController joystick;
  private final DoubleSupplier joystickInput;
  double holdPosition = 0;
  double lastJoyVal = 0;

  public TestClimber(
      ClimberParentSystem subsystem, StormXboxController joystick, DoubleSupplier joyStickInput) {
        System.out.println("TestClimber()");
    this.subsystem = subsystem;
        this.joystick = joystick;
    this.joystickInput = joyStickInput;

    addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("TestClimber.initialize()");
    lastJoyVal = joystickInput.getAsDouble();
    }

    @Override
    public void execute() {
        if (joystick.getAisPressed()) {
      subsystem.zero();
            return;
        }

    if (joystick.getXisPressed()) subsystem.disableLimits();

    if (joystick.getYisPressed()) subsystem.enableLimits();

    double joyVal = joystickInput.getAsDouble();
    LRSpeeds speeds = new LRSpeeds(joyVal * kClimberSpeed, joyVal * kClimberSpeed);

        // Move only the one on the side with the bumper held
        if (joystick.getLeftBumperIsHeld()) {
      speeds.disableRight();
        } else if (joystick.getRightBumperIsHeld()) {
      speeds.disableLeft();
        }

    if (joyVal == 0 && lastJoyVal > 0) setHoldPosition();
    if (joyVal == 0) hold();
    else subsystem.setSpeed(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TestClimber.end( interrupted = " + interrupted + " )");
    subsystem.stop();
    }

  private void hold() {
    subsystem.leftPID(new TrapezoidProfile.State(holdPosition, 0));
    subsystem.rightPID(new TrapezoidProfile.State(holdPosition, 0));
  }

  private void setHoldPosition() {
    holdPosition = subsystem.leftPosition();
  }
}
