package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.configfile.StormProp;

public class NavX extends SubsystemBase {
    //AHRS is the thingy for connectivity and to access state information
    private AHRS ahrs = null;

    public NavX() {
        if(StormProp.getBoolean("hasNavX", true)) {
            ahrs = new AHRS();
            ahrs.enableLogging(true);
        } else {
            System.out.println("NO NAVX IN USEEEEEEEEEEEE");
        }
    }

    public double getAngle() {
        return ahrs.getAngle();
    }
}
