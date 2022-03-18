package frc.robot.commands.ballHandler.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ballHandler.Load;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Intake;

public class LoadOne extends Load {

    final private Feeder feeder;

    public LoadOne(Intake intake, Feeder feeder) {
        super(intake, feeder);
        this.feeder = feeder;
    }

    @Override
    public boolean isFinished() {
        return feeder.getLimit();
    }
}
