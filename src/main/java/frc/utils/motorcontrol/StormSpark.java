package frc.utils.motorcontrol;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class StormSpark extends CANSparkMax {
    private static final int currentLimit = Constants.kCurrentLimit;
    private static final double temperatureRampThreshold = Constants.kTemperatureRampThreshold;
    private static final double temperatureRampLimit = Constants.kTemperatureRampLimit;
    private final double delta;
    private double scale = 1.0;

    public StormSpark(int deviceID, MotorType type) {
        super(deviceID, type);

        restoreFactoryDefaults();
        setSmartCurrentLimit(currentLimit);

        delta = Math.min(temperatureRampLimit - temperatureRampThreshold, 1.0); // Safety margin - don't want divide by 0!
    }

    @Override
    // We should probably have a way to coordinate this ramp among multiple motors
    // The drive class already does this for its set speed, but the robot will pull left or right if
    // left and right motors are getting different results in this method.
    public void set(double speed) {
        // if the temperature is too high, start ramping down.
        // reduce speed to 0 at temperatureRampLimit
        // start ramping down at temperatureRampThreshold

        double temp = getMotorTemperature();

        if (temp > temperatureRampThreshold) {
            speed *= Math.max((temperatureRampLimit - temp) / delta, 0.0);
            System.out.println("Speed safety control - factor " + speed);
        }

        super.set(scale * speed);
    }

    // Should be between 0.0 and 1.0 - to account for oddities in the drive train
    // e.g. two different gear ratios
    public void setScale(double scale) {
        this.scale = MathUtil.clamp(scale, 0, 1.0);
    }

    public static void check(REVLibError command) {
        if (command != REVLibError.kOk) {
            DriverStation.reportWarning("WARNING: SparkMax action failed!", true);
        }
    }

    public static void check(REVLibError command, String logMessage) {
        if (command != REVLibError.kOk) {
            DriverStation.reportWarning("WARNING: SparkMax action failed: " + logMessage, true);
        }
    }
}