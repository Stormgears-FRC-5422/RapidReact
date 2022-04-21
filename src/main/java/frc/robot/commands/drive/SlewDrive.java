package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.utils.drive.StormDrive;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class SlewDrive extends CommandBase {
  private final StormDrive drive;
  private final DoubleSupplier X;
  private final DifferentialDrive differentialDrive;
  private final double prevTurnSlewRate;
  protected SlewRateLimiter limiter;
  protected SlewRateLimiter turnLimiter;
  private double prevSlewRate;
  private DoubleSupplier Z;
  private boolean ignoreZSquaring = false;

  public SlewDrive(StormDrive drive, DoubleSupplier X, DoubleSupplier Z) {
    System.out.println("Just created the SlewDrive command");
    addRequirements(drive);

    this.drive = drive;
    this.X = X;
    this.Z = Z;
    differentialDrive = drive.getDifferentialDrive();

    prevSlewRate = drive.getSlewRate();
    prevTurnSlewRate = drive.getTurnSlewRate();
    System.out.println(
        "Initial slewRate: " + prevSlewRate + "  initial turnSlewRate: " + prevTurnSlewRate);

    limiter = new SlewRateLimiter(prevSlewRate);
    turnLimiter = new SlewRateLimiter(prevTurnSlewRate);
  }

  @Override
  public void initialize() {
    System.out.println(
        "INITIALINITIALINITIALINITIALINITIALINITIALINITIALINITIALINITIALINITIALINITIAL");
  }

  @Override
  public void execute() {
    double targetSpeed = X.getAsDouble();
    double targetZRotation = Z.getAsDouble();

    if (drive.getSlewRate() != prevSlewRate) {
      System.out.println("updated slewRate: " + prevSlewRate);
      prevSlewRate = drive.getSlewRate();
      limiter = new SlewRateLimiter(prevSlewRate);
    }

    //    if (drive.getTurnSlewRate() != prevTurnSlewRate) {
    //      System.out.println("updated turnSlewRate: " + prevSlewRate);
    //      prevTurnSlewRate = drive.getTurnSlewRate();
    //      turnLimiter = new SlewRateLimiter(prevTurnSlewRate);
    //    }

    targetSpeed = limiter.calculate(targetSpeed);
    //    targetZRotation = turnLimiter.calculate(targetZRotation);

    if (kSquareDriveInputs) {
      targetSpeed = Math.copySign(targetSpeed * targetSpeed, targetSpeed);
      targetZRotation =
          !ignoreZSquaring
              ? Math.copySign(targetZRotation * targetZRotation, targetZRotation)
              : targetZRotation;
    }

    if (kDriveStyle.equalsIgnoreCase("curvature"))
      differentialDrive.curvatureDrive(
          (drive.getReverse() ? -1 : 1) * (drive.getPrecision() ? kXPrecision : 1) * targetSpeed,
          (drive.getReverse() ? -1 : 1)
              * (drive.getPrecision() ? kZPrecision : 1)
              * targetZRotation,
          true);
    else
      differentialDrive.arcadeDrive(
          (drive.getReverse() ? -1 : 1) * (drive.getPrecision() ? kXPrecision : 1) * targetSpeed,
          (drive.getReverse() ? -1 : 1)
              * (drive.getPrecision() ? kZPrecision : 1)
              * targetZRotation); // inputs already squared above
  }

  public void setZ(DoubleSupplier newZ, boolean ignoreZSquaring) {
    Z = newZ;
    this.ignoreZSquaring = ignoreZSquaring;
  }

  public DoubleSupplier getZ() {
    return Z;
  }

  public void setZ(DoubleSupplier newZ) {
    setZ(newZ, false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
