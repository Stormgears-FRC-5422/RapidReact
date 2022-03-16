package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class HomeClimber extends CommandBase {
    Climber climber;

    public HomeClimber(Climber climber) {
        System.out.println("HomeClimber()");
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.disableLimits();
    }

    @Override
    public void execute() {
        climber.goHome();
    }

    @Override
    public boolean isFinished() {
        return climber.isHome();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("HomeClimber.end()");
        // unconditionally - cross your fingers - this really shouldn't be interrupted.
        climber.enableLimits();

        // We are only ready to zero if the command sequence completes normally
        if (!interrupted) {
            System.out.println("Not interrupted");
            climber.zero();
        }
    }
}
