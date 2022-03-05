package frc.robot.commands.ballHandler;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;

import static frc.robot.Constants.kShooterHighRPS;
import static frc.robot.Constants.kShooterLowRPS;

public class Shoot extends CommandBase {

    public enum Height{
        LOW(kShooterLowRPS),
        HIGH(kShooterHighRPS);

        public double rps;

        Height(double rps) {
            this.rps = rps;
        }
    }


    private final Feeder feeder;
    private final Shooter shooter;
    private Height mode = Height.LOW;

    public Shoot(Feeder feeder, Shooter shooter) {
        this.feeder = feeder;
        this.shooter = shooter;
        addRequirements(feeder, shooter);
        Shuffleboard.getTab("Shooter").add(this);
    }

    @Override
    public void initialize() {
        feeder.setLimit(true);
    }

    @Override
    public void execute() {
        feeder.setLimit(!isReady());
        feeder.on();
        shooter.setSpeed(mode.rps);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.off();
        shooter.off();
    }


    private boolean isReady() {
        return shooter.getSpeed() >= 0.95 * mode.rps;
    }

    public void toggleMode() {
        if (mode == Height.LOW) {
            mode = Height.HIGH;
        } else {
            mode = Height.LOW;
        }
        System.out.println("Shooter mode: " + mode);
    }

    private void setHigh(double rps) {
        Height.HIGH.rps = rps;
    }

    private boolean isHigh(){
        return mode == Height.HIGH;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Speed", shooter::getSpeed, null);
        builder.addDoubleProperty("High Height", () -> Height.HIGH.rps, this::setHigh);
        builder.addBooleanProperty("High", this::isHigh, null);
    }
}
