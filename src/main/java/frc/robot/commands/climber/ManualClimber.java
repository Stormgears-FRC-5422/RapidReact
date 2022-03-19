package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import frc.utils.LRSpeeds;
import frc.utils.joysticks.StormXboxController;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.kClimberSpeed;

public class ManualClimber extends CommandBase {
  private final ClimbingSubsystem subsystem;
  private final StormXboxController joystick;
  private final DoubleSupplier joystickInput;
  double lastJoyVal = 0;

  public ManualClimber(
      ClimbingSubsystem subsystem, StormXboxController joystick, DoubleSupplier joyStickInput) {
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

    if (joystick.getXisPressed()) {
      subsystem.disableAllLimits();
    }
    if (joystick.getYisPressed()) subsystem.enableAllLimits();

    double joyVal = joystickInput.getAsDouble();
    LRSpeeds speeds = new LRSpeeds(joyVal * kClimberSpeed, joyVal * kClimberSpeed);

    // Move only the one on the side with the bumper held
    if (joystick.getLeftBumperIsHeld()) {
      speeds.disableRight();
    } else if (joystick.getRightBumperIsHeld()) {
      speeds.disableLeft();
    }

    //    if (joyVal == 0 && lastJoyVal > 0) setHoldPosition();

    if (joyVal == 0) hold(subsystem.leftPosition(), subsystem.rightPosition());
    else subsystem.setSpeed(speeds);
    this.lastJoyVal = joyVal;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("TestClimber.end( interrupted = " + interrupted + " )");
    subsystem.stop();
  }

  private void hold(double left, double right) {
    subsystem.leftPID(new TrapezoidProfile.State(left, 0));
    subsystem.rightPID(new TrapezoidProfile.State(right, 0));
  }
}
