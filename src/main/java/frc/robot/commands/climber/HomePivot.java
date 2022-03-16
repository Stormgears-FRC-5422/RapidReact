package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Pivot;

public class HomePivot extends CommandBase {
    Pivot pivot;

    public HomePivot(Pivot pivot) {
        System.out.println("HomePivot()");
        this.pivot = pivot;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.disableLimits();
    }

    @Override
    public void execute() {
        pivot.goHome();
    }

    @Override
    public boolean isFinished() {
        return pivot.isHome();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("HomePivot.end()");
        // unconditionally - cross your fingers - this really shouldn't be interrupted.
        pivot.enableLimits();

        // We are only ready to zero if the command sequence completes normally
        if (!interrupted) {
            System.out.println("Not interrupted");
            pivot.zero();
        }
    }
}
