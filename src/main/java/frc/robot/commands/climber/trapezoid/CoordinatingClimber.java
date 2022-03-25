package frc.robot.commands.climber.trapezoid;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.ClimbingSubsystem;
import frc.robot.subsystems.climber.HangerConstraints;
import frc.utils.LRSpeeds;
import frc.utils.joysticks.StormXboxController;

import static frc.robot.Constants.kClimberSpeed;
import static java.lang.Math.copySign;

public class CoordinatingClimber extends CommandBase {
    ClimbingSubsystem climber;
    ClimbingSubsystem pivot;
    StormXboxController joystick;
    double leftClimberPosition;
    TrapezoidProfile.State climberState;
    TrapezoidProfile.State pivotState;

    public CoordinatingClimber(ClimbingSubsystem climber, ClimbingSubsystem pivot, StormXboxController joystick) {
        this.climber = climber;
        this.pivot = pivot;
        this.joystick = joystick;

        addRequirements(climber, pivot);
        climberState = new TrapezoidProfile.State(0,0);
        pivotState = new TrapezoidProfile.State(0,0);
    }

    @Override
    public void initialize() {
        System.out.println("CoordinatingClimber.initialize()");

    }

    @Override
    public void execute() {
        leftClimberPosition = climber.leftPosition();
        double joyVal = -joystick.getLeftJoystickY();
        double climbTarget = leftClimberPosition + ( joyVal == 0.0 ? 0.0 : 40 * copySign(1.0, joyVal) );

        climberState.position = climbTarget;
        pivotState.position = HangerConstraints.getPivotPosition(climbTarget);

        climber.leftPID(climberState);
        climber.rightPID(climberState);
        pivot.leftPID(pivotState);
        pivot.rightPID(pivotState);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TestClimber.end( interrupted = " + interrupted + " )");
        climber.stop();
        pivot.stop();
    }

}