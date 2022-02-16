package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import frc.utils.joysticks.DriveJoystick;

public class TestIntake extends CommandBase {
    private final Intake intake;
    private DriveJoystick joystick;

    public TestIntake(Intake intake, DriveJoystick joystick){
        System.out.println("Creating Intake Command");
        addRequirements(intake);

        this.intake = intake;
        this.joystick = joystick;
    }

    @Override
    public void execute() {
        double r = joystick.getRightTrigger();
        double l = joystick.getLeftTrigger();

        //System.out.println("Right: " + r + " left: " + l);
        intake.set(r - l);
    }
}
