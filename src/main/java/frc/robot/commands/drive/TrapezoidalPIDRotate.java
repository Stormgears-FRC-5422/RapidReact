package frc.robot.commands.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SparkDrive;

public class TrapezoidalPIDRotate extends TrapezoidProfileCommand {

    public static final SimpleMotorFeedforward FF =
            new SimpleMotorFeedforward(Constants.FFRadiansToPOutks, Constants.FFRadiansToPOutkv);

    private NavX navX;
    private TrapezoidProfile.State goal;

    public TrapezoidalPIDRotate(SparkDrive drive, NavX navX, TrapezoidProfile.State goal) {
        super(
                new TrapezoidProfile(new TrapezoidProfile.Constraints(
                        Constants.kMAXVELROTATE, Constants.kMAXACCROTATE
                ), goal),
                (TrapezoidProfile.State nextState) -> {
                    SmartDashboard.putNumber("Position", nextState.position);
                    SmartDashboard.putNumber("Velocity", nextState.velocity);
                    double pidOff = navX.calculateRotateVel(nextState.position);
                    SmartDashboard.putNumber("PID Offset", pidOff);
                    double trapPlusFF = FF.calculate(nextState.velocity);
                    SmartDashboard.putNumber("Trapezoidal plus Feedforward value", trapPlusFF);
                    double actualOut = trapPlusFF + pidOff;
                    drive.rotate(-actualOut);
                },
                navX
        );
        addRequirements(drive);
        this.navX = navX;
        this.goal = goal;
    }


    @Override
    public void initialize() {
        super.initialize();
        navX.resetAngle();
        SmartDashboard.putNumber("Goal position", goal.position);
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Rotation is done!", super.isFinished());
        //boolean angleIsNear = navX.getAngleDegrees() <= goal.position + 0.3 || navX.getAngleDegrees() >= goal.position + 0.3;
        return super.isFinished();
    }
}
