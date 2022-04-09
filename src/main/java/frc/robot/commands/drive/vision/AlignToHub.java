package frc.robot.commands.drive.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.VisionDrive;
import frc.utils.drive.StormDrive;

import java.util.function.Supplier;

public class AlignToHub extends SequentialCommandGroup {
    /**
     *
     * @param hasTarget expects the method reference for if we have any targets
     * @param getYaw expects the method reference for the current yaw/error to target
     */
    public AlignToHub(VisionDrive drive, Supplier<Boolean> hasTarget, Supplier<Double> getYaw) {
        FindStep findStep = new FindStep(hasTarget, drive);
        AlignStep alignStep = new AlignStep(getYaw, drive);
        // find target to make sure we don't get null pointer -> align to hub
        addCommands(findStep, alignStep);
    }
}
