package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;
import frc.robot.subsystems.ballHandler.Shooter;

import static frc.robot.Constants.shooterLowRPM;
import static frc.robot.subsystems.ballHandler.DiagnosticIntake.TestMode.intake;

public class Shoot extends CommandBase {
    private final Feeder feeder;
    private final Shooter shooter;
    private final Intake intake;

    public Shoot(Feeder feeder, Shooter shooter, Intake intake) {
        this.feeder = feeder;
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(feeder, shooter, intake);
    }

    @Override
    public void initialize() {
        feeder.setLimit(true);
        intake.on();
    }

    @Override
    public void execute() {
        feeder.setLimit(!shooter.isReady());
        feeder.on();
        shooter.setSpeed(shooterLowRPM);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.off();
        shooter.off();
        intake.off();
    }

}
