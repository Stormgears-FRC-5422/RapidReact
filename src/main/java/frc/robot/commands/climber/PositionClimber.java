package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Pivot;
import frc.utils.joysticks.StormXboxController;

public class PositionClimber extends CommandBase {
    private Climber climber;
    private StormXboxController joystick;

    public PositionClimber(Climber climber, StormXboxController joystick) {
        System.out.println("TestClimber()");
        this.climber = climber;
        this.joystick = joystick;

        this.addRequirements(climber);
    }

}
