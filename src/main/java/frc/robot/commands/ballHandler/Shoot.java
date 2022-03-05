package frc.robot.commands.ballHandler;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ballHandler.Feeder;
import frc.robot.subsystems.ballHandler.Shooter;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.ballHandler.Shooter.Height;

public class Shoot extends PIDCommand {

    private final Feeder feeder;
    private final Shooter shooter;

    public Shoot(Feeder feeder, Shooter shooter) {
        super(
                new PIDController(kShooterP, kShooterI, kShooterD),
                shooter::getSpeed,
                shooter::setpoint,
                shooter::runToSpeed,
                shooter, feeder
        );
        this.feeder = feeder;
        this.shooter = shooter;
        Shuffleboard.getTab("Shoot Command").add(this);
    }

    @Override
    public void initialize() {
        feeder.setLimit(true);
    }

    @Override
    public void execute() {
        super.execute();
        if (!isReady()){
            feeder.setLimit(false);
            getController().reset();
        } else feeder.setLimit(true);
        feeder.on();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.off();
        shooter.off();
    }

    private boolean isReady() {
        return shooter.getSpeed() >= (0.98 * shooter.mode.rps) && shooter.getSpeed() <= (1.02 * shooter.mode.rps);
    }

    public void toggleMode() {
        if (shooter.mode == Height.LOW) shooter.mode = Height.HIGH;
        else shooter.mode = Height.LOW;
        System.out.println("Shooter mode: " + shooter.mode);
    }

    private void setHigh(double rps) {
        Height.HIGH.rps = rps;
    }

    private boolean isHigh(){
        return shooter.mode == Height.HIGH;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Speed", shooter::getSpeed, null);
        builder.addDoubleProperty("High Height", () -> Height.HIGH.rps, this::setHigh);
        builder.addBooleanProperty("High", this::isHigh, null);
        builder.addDoubleProperty("P Value", this.getController()::getP, this.getController()::setP);
        builder.addDoubleProperty("I Value", this.getController()::getI, this.getController()::setI);
        builder.addDoubleProperty("D Value", this.getController()::getD, this.getController()::setD);
        builder.addDoubleProperty("adjust", shooter::getAdjust, shooter::setAdjust);
        builder.addDoubleProperty("voltage", () -> shooter.output, null);
        builder.addDoubleProperty("pidOutput", () -> shooter.pidOutput, null);
    }
}
