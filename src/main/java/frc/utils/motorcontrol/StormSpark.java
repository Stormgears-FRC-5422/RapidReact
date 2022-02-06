package frc.utils.motorcontrol;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.utils.configfile.StormProp;

import java.lang.invoke.ConstantBootstraps;

public class StormSpark extends CANSparkMax {
    private static final int currentLimit = Constants.currentLimit;
    private static final double temperatureRampThreshold = Constants.temperatureRampThreshold;
    private static final double temperatureRampLimit = Constants.temperatureRampLimit;
    private double delta;
    private double temp;


    public StormSpark(int deviceID, MotorType type) {
        super(deviceID, type);

        restoreFactoryDefaults();
        setSmartCurrentLimit(currentLimit);

        delta = Math.min(temperatureRampLimit - temperatureRampThreshold, 1.0);  // Safety margin - don't want divide by 0!
    }

    @Override
    public void set(double speed) {
        // if the temperature is too high, start ramping down.
        // reduce speed to 0 at temperatureRampLimit
        // start ramping down at temperatureRampThreshold
        temp = getMotorTemperature();

        if(temp > temperatureRampThreshold) {
          speed *= Math.max((temperatureRampLimit - temp) / delta, 0.0);
          System.out.println("Speed safety control - factor " + speed);
        }

        super.set(speed);
    }

    public static void check(REVLibError command){
        if(command != REVLibError.kOk){
            DriverStation.reportWarning("WARNING: SparkMax action failed!", true);
        }
    }

    public static void check(REVLibError command, String logMessage){
        if(command != REVLibError.kOk){
            DriverStation.reportWarning("WARNING: SparkMax action failed: " + logMessage, true);
        }
    }
}