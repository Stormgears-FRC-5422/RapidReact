package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberParentSystem;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class PositionClimber extends MoveCommand {

    private final Climber climber;

    public PositionClimber(Climber climber) {
        this.climber = climber;

        this.leftController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new State(climber.leftPosition(), 0)), this::leftPID);
        this.rightController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new State(climber.rightPosition(), 0)), this::rightPID);

        this.addRequirements(climber);

        Shuffleboard.getTab("Climber").add(this);
    }

    @Override
    public ClimberParentSystem subsystem() {
        return climber;
    }
}
