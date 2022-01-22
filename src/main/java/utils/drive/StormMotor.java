package utils.drive;

import frc.robot.Constants;

public class StormMotor {

    private StormMotor() {
        throw new IllegalStateException("Do not make a StormMotor object");
    }

    public static StormMotorType motorType() {
        if (Constants.MOTOR_TYPE.equals("Spark")) return StormMotorType.SPARK;
        else if (Constants.MOTOR_TYPE.equals("Talon")) return StormMotorType.TALON;
        return StormMotorType.SPARK;
    }
}
