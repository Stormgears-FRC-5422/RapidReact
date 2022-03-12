package frc.robot.commands.ballHandler;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.utils.joysticks.StormXboxController;

public class LiftIntake extends CommandBase {

    private final Feeder feeder;
    private final StormXboxController joystick;

    public LiftIntake(Feeder feeder, StormXboxController joystick) {
        System.out.println("LiftIntake()");
        this.feeder = feeder;
        this.joystick = joystick;
        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        System.out.println("LiftIntake initialize()");
        feeder.initReverse();
    }

    @Override
    public void execute() {
        feeder.setLift(joystick.getRightTrigger());
    }

    @Override
    public void end(boolean interrupted) {
        feeder.off();
    }



}
