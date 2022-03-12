package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Pivot;
import frc.utils.joysticks.StormXboxController;

public class PositionPivot extends CommandBase {
    private Pivot pivot;
    private StormXboxController joystick;

    public PositionPivot(Pivot pivot, StormXboxController joystick) {
        System.out.println("TestClimber()");
        this.pivot = pivot;
        this.joystick = joystick;

        this.addRequirements(pivot);
    }

    @Override
    public void initialize(){

    }

    public void execute() {

    }

    public boolean isFinished() {
        return true;
    }

}
