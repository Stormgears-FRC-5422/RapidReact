package frc.robot.commands.ballHandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.DiagnosticIntake;
import frc.utils.joysticks.DriveJoystick;

public class TestIntake extends CommandBase {
    private final DiagnosticIntake diagnosticIntake;
    private final DriveJoystick joystick;

    public TestIntake(DiagnosticIntake diagnosticIntake, DriveJoystick joystick) {
        System.out.println("Creating Intake Command");
        addRequirements(diagnosticIntake);

        this.diagnosticIntake = diagnosticIntake;
        this.joystick = joystick;
    }

    @Override
    public void execute() {
        double r = joystick.getRightTrigger();
        double l = joystick.getLeftTrigger();

        //System.out.println("Right: " + r + " left: " + l);
        diagnosticIntake.set(r - l);
    }
}
