package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SafeDrive;

import java.util.function.DoubleSupplier;

public class DiagnosticDrive extends CommandBase {
  private static final double kNullZone = 0.2;
  private final SafeDrive drive;
  private final DoubleSupplier leftMasterSupplier;
  private final DoubleSupplier leftSlaveSupplier;
  private final DoubleSupplier rightMasterSupplier;
  private final DoubleSupplier rightSlaveSupplier;

  public DiagnosticDrive(
      SafeDrive drive,
      DoubleSupplier leftMasterSupplier,
      DoubleSupplier leftSlaveSupplier,
      DoubleSupplier rightMasterSupplier,
      DoubleSupplier rightSlaveSupplier) {

    addRequirements(drive);

    this.drive = drive;
    this.leftMasterSupplier = leftMasterSupplier;
    this.rightMasterSupplier = rightMasterSupplier;
    this.leftSlaveSupplier = leftSlaveSupplier;
    this.rightSlaveSupplier = rightSlaveSupplier;
  }

  @Override
  public void execute() {
    double leftMaster =
        Math.abs(leftMasterSupplier.getAsDouble()) > kNullZone
            ? leftMasterSupplier.getAsDouble()
            : 0;
    double rightMaster =
        Math.abs(rightMasterSupplier.getAsDouble()) > kNullZone
            ? rightMasterSupplier.getAsDouble()
            : 0;
    double leftSlave =
        Math.abs(leftSlaveSupplier.getAsDouble()) > kNullZone ? leftSlaveSupplier.getAsDouble() : 0;
    double rightSlave =
        Math.abs(rightSlaveSupplier.getAsDouble()) > kNullZone
            ? rightSlaveSupplier.getAsDouble()
            : 0;

    drive.diagnosticDrive(leftMaster, leftSlave, rightMaster, rightSlave);
  }
}
