package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import frc.utils.LRSpeeds;
import frc.utils.joysticks.StormXboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.kClimberSpeedScale;

@Log.Exclude
public class ManualClimber extends CommandBase implements Loggable {
  @Log.Exclude private final ClimbingSubsystem subsystem;
  private final StormXboxController joystick;
  private final DoubleSupplier joystickInput;
  private boolean holding = false;
  private double leftPosition = 0;
  private double rightPosition = 0;

  public ManualClimber(
      ClimbingSubsystem subsystem, StormXboxController joystick, DoubleSupplier joyStickInput) {
    System.out.println("ManualClimber()");
    this.subsystem = subsystem;
    this.joystick = joystick;
    this.joystickInput = joyStickInput;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    System.out.println("ManualClimber.initialize()");
    leftPosition = subsystem.leftPosition();
    rightPosition = subsystem.rightPosition();
  }

  @Override
  public void execute() {
    if (joystick.getAButtonIsHeld()) {
      subsystem.zero();
      return;
    }

    if (joystick.getXButtonIsHeld()) {
      subsystem.disableAllLimits();
      return;
    }

    if (joystick.getYButtonIsHeld()) {
      subsystem.enableAllLimits();
      return;
    }

    double joyVal = joystickInput.getAsDouble();
    LRSpeeds speeds = new LRSpeeds(joyVal * kClimberSpeedScale, joyVal * kClimberSpeedScale);

    // Move only the one on the side with the bumper held
    if (joystick.getLeftLittleButtonIsHeld()) {
      speeds.disableRight();
    } else if (joystick.getRightLittleButtonIsHeld()) {
      speeds.disableLeft();
    }

    // TODO - re-enable?
    //    if (joyVal == 0 && lastJoyVal > 0) setHoldPosition();
    if (joyVal == 0) {
//      if (!holding) {
//        holding = true;
//        leftPosition = subsystem.leftPosition();
//        rightPosition = subsystem.rightPosition();
//      }
//      hold(leftPosition, rightPosition);
      subsystem.setSpeed(LRSpeeds.stop());
    } else {
      holding = false;
      subsystem.setSpeed(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("ManualClimber.end( interrupted = " + interrupted + " )");
    subsystem.stop();
  }

  private void hold(double left, double right) {
    subsystem.leftPID(new TrapezoidProfile.State(left, 0));
    subsystem.rightPID(new TrapezoidProfile.State(right, 0));
  }
}
