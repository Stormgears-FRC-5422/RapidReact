package frc.robot.commands.climber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.climber.ClimberParentSystem;
import frc.robot.subsystems.climber.Pivot;

public class PositionPivot extends MoveCommand {

    private final Pivot pivot;

    public PositionPivot(Pivot pivot) {
        this.pivot = pivot;

        this.leftController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new TrapezoidProfile.State(pivot.leftPosition(), 0)), this::leftPID);
        this.rightController = new TrapezoidProfileCommand(new TrapezoidProfile(constraints, goal.state, new TrapezoidProfile.State(pivot.rightPosition(), 0)), this::rightPID);

        this.addRequirements(pivot);

        Shuffleboard.getTab("Climber").add(this);
    }

    @Override
    public ClimberParentSystem subsystem() {
        return pivot;
    }
}
