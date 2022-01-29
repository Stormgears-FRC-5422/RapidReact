package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.configfile.StormProp;

public class NavX extends SubsystemBase {
    //AHRS is the thingy for connectivity and to access state information
    private AHRS ahrs = null;

    public enum AngleType {
        RADIANS,
        DEGREES
    }

    public AngleType getAngleType() {
        return angleType;
    }

    private AngleType angleType;

    public NavX(AngleType at) {
        if(StormProp.getBoolean("usingNavX", true)) {
            ahrs = new AHRS();
            ahrs.enableLogging(true);
        } else {
            System.out.println("NO NAVX IN USEEEEEEEEEEEE");
        }
        angleType = at;
    }

    public double getAngle() {
        if (angleType == AngleType.DEGREES) return ahrs.getAngle();
        float angle = (float) ahrs.getAngle();
        return angle * (Math.PI/180);
    }
}
