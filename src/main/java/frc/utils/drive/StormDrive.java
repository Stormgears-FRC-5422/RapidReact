package frc.utils.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StormDrive extends SubsystemBase {
    enum StormMotorType {
        SPARK, TALON
    }

    protected boolean reverse = false;
    protected boolean precision = false;

    //StormMotorType motorType();
    public abstract DifferentialDrive getDifferentialDrive();

    public boolean getReverse() {return this.reverse;}

    public void setReverse(boolean r) {this.reverse = r;}

    public void toggleReverse() {
        reverse = !reverse;
        System.out.println("reverse = " + reverse);
    }

    public boolean getPrecisions() {return this.precision;}

    public void setPrecision(boolean p) {this.precision = p;}

    public void togglePrecision() {
        precision = !precision;
        System.out.println("precision = " + precision);
    }

    protected abstract MotorController[] getMotors();

}
