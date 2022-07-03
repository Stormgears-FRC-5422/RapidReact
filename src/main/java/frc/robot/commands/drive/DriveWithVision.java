package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.sensors.Vision;
import frc.utils.drive.StormDrive;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class DriveWithVision extends CommandBase implements Loggable {
  private final Vision vision;

  @Config.PIDController(tabName = "Vision")
  private final PIDController turnController =
      new PIDController(kVisionDriveP, kVisionDriveI, kVisionDriveD);

  private final DoubleSupplier joystickZ;
  private final SlewDrive slewDrive;

  public DriveWithVision(StormDrive drive, DoubleSupplier X, DoubleSupplier Z, Vision vision) {
    this.slewDrive = new SlewDrive(drive, X, Z);
    this.joystickZ = slewDrive.getZ();
    this.vision = vision;
    addRequirements(slewDrive.getRequirements().toArray(new Subsystem[0]));
  }

  @Override
  public void initialize() {
    slewDrive.initialize();
  }

  @Override
  public void execute() {
    if (!vision.hasTarget()) {
      slewDrive.setZ(joystickZ);
    } else {
      slewDrive.setZ(this::pidOut, true);
    }
    slewDrive.execute();
  }

  @Override
  public void end(boolean interrupted) {
    slewDrive.end(interrupted);
  }

  private double pidOut() {
    double pidOut =
        MathUtil.clamp(turnController.calculate(kVisionShootingOffset, vision.getYaw()), -.5, .5);
    return Math.abs(pidOut) > kVisionAlignMinimumPercentOutput ? pidOut : 0;
  }
}
