package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.NavX;
import frc.utils.drive.Drive;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class NavXAlign extends PIDCommand {

    private final double P = 0.05;
    private final double I = 0;
    private final double D = 0;

    private final Drive drive;
    private final NavX navX;


    public NavXAlign(NavX navX, Drive drive, double targetAngle) {
        super(
                new PIDController(0.05, 0, 0),
                (DoubleSupplier) navX::getAngle,
                targetAngle,
                (DoubleConsumer) angle -> drive.getDifferentialDrive().arcadeDrive(0, angle),
                drive
        );
        this.drive = drive;
        this.navX = navX;
    }
}
