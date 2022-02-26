package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SafeDrive;
import java.util.function.DoubleSupplier;

import static java.lang.Math.*;

public class DiagnosticDrive extends CommandBase {
    private final SafeDrive drive;
    private static final double kNullZone = 0.2;

    private DoubleSupplier leftMasterSupplier;
    private DoubleSupplier leftSlaveSupplier;
    private DoubleSupplier rightMasterSupplier;
    private DoubleSupplier rightSlaveSupplier;

    public DiagnosticDrive(SafeDrive drive, DoubleSupplier leftMasterSupplier, DoubleSupplier leftSlaveSupplier,
                                        DoubleSupplier rightMasterSupplier, DoubleSupplier rightSlaveSupplier){

        addRequirements(drive);

        this.drive = drive;
        this.leftMasterSupplier = leftMasterSupplier;
        this.rightMasterSupplier = rightMasterSupplier;
        this.leftSlaveSupplier = leftSlaveSupplier;
        this.rightSlaveSupplier = rightSlaveSupplier;
    }

    @Override
    public void execute() {
        double leftMaster = abs(leftMasterSupplier.getAsDouble()) > kNullZone ? leftMasterSupplier.getAsDouble() : 0;
        double rightMaster = abs(rightMasterSupplier.getAsDouble()) > kNullZone ? rightMasterSupplier.getAsDouble() : 0;
        double leftSlave = abs(leftSlaveSupplier.getAsDouble()) > kNullZone ? leftSlaveSupplier.getAsDouble() : 0;
        double rightSlave = abs(rightSlaveSupplier.getAsDouble()) > kNullZone ? rightSlaveSupplier.getAsDouble() : 0;

        drive.diagnosticDrive(leftMaster, leftSlave, rightMaster, rightSlave);
    }
}
