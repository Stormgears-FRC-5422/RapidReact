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

    public CoordinatingClimber(ClimbingSubsystem climber, ClimbingSubsystem pivot, StormXboxController joystick) {
        this.climber = climber;
        this.pivot = pivot;
        this.joystick = joystick;
    }

    @Override
    public void initialize() {
        System.out.println("CoordinatingClimber.initialize()");
    }

    @Override
    public void execute() {
        leftClimberPosition = climber.leftPosition();

        double joyVal =-joystick.getLeftJoystickY();

        double climbTarget = leftClimberPosition + ( joyVal == 0.0 ? 0.0 : 10 * copySign(1.0, joyVal) );
        double pivotTarget = HangerConstraints.getPivotPosition(climbTarget);

        System.out.println("joyVal: " + joyVal + " climbTarget: " + climbTarget + " pivotTarget: " + pivotTarget);
        climber.simpleMotion(climbTarget);
        //pivot.simpleMotion(pivotTarget);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TestClimber.end( interrupted = " + interrupted + " )");
        climber.stop();
        pivot.stop();
    }

}
