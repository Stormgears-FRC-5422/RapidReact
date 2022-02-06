package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.configfile.StormProp;
import frc.utils.filters.ExponentialAverage;
import frc.utils.filters.Filter;
import frc.utils.motorcontrol.StormSpark;

import java.util.Map;

import static edu.wpi.first.wpilibj.DriverStation.reportWarning;


public class SafeDrive extends SubsystemBase {
    private StormSpark rightMaster;
    private StormSpark leftMaster;
    private StormSpark rightSlave;
    private StormSpark leftSlave;

    private Filter rightMasterCurrent;
    private Filter rightSlaveCurrent;
    private Filter leftMasterCurrent;
    private Filter leftSlaveCurrent;

    private Filter rightMasterTemp;
    private Filter rightSlaveTemp;
    private Filter leftMasterTemp;
    private Filter leftSlaveTemp;

    /*
     * Temp Limiting
     */
    private static final double temperatureRampThreshold = StormProp.getInt("SparkMaxTemperatureRampThreshold", 40);
    private static final double temperatureRampLimit = StormProp.getInt("SparkMaxTemperatureRampLimit", 55);
    private double temp;
    private double delta;
    private int tempWarningCount = 0;
    private NetworkTableEntry m_shuffle_slew_rate;
    private double m_slew_rate_value;
    private NetworkTableEntry m_shuffle_turn_slew_rate;
    private double m_turn_slew_rate_value;

    private DifferentialDrive differentialDrive;
    private CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.kCoast;

    private static final double xPrecisionMultiplier = StormProp.getNumber("xPrecision", 0.25);
    private static final double zPrecisionMultiplier = StormProp.getNumber("zPrecision", 0.35);

    private boolean reverse;
    private boolean precision;

    public SafeDrive() {
        System.out.println("In SafeDrive() constructor");

        leftMaster = new StormSpark(StormProp.getInt("leftMasterId", -1), MotorType.kBrushless);
        leftSlave = new StormSpark(StormProp.getInt("leftSlaveId", -1), MotorType.kBrushless);
        rightMaster = new StormSpark(StormProp.getInt("rightMasterId", -1), MotorType.kBrushless);
        rightSlave = new StormSpark(StormProp.getInt("rightSlaveId", -1), MotorType.kBrushless);

        StormSpark.check(rightMaster.setIdleMode(idleMode), "set idle mode");
        StormSpark.check(leftMaster.setIdleMode(idleMode), "set idle mode");
        StormSpark.check(leftSlave.setIdleMode(idleMode), "set idle mode");
        StormSpark.check(rightSlave.setIdleMode(idleMode), "set idle mode");

        StormSpark.check(leftSlave.follow(leftMaster), "set follower mode");
        StormSpark.check(rightSlave.follow(rightMaster), "set follower mode");

        rightMasterCurrent = new ExponentialAverage(() -> rightMaster.getOutputCurrent());
        rightSlaveCurrent = new ExponentialAverage(() -> rightSlave.getOutputCurrent());
        leftMasterCurrent = new ExponentialAverage(() -> leftMaster.getOutputCurrent());
        leftSlaveCurrent = new ExponentialAverage(() -> leftSlave.getOutputCurrent());

        rightMasterTemp = new ExponentialAverage(() -> rightMaster.getMotorTemperature());
        rightSlaveTemp = new ExponentialAverage(() -> rightSlave.getMotorTemperature());
        leftMasterTemp = new ExponentialAverage(() -> leftMaster.getMotorTemperature());
        leftSlaveTemp = new ExponentialAverage(() -> leftSlave.getMotorTemperature());

        delta = Math.max(temperatureRampLimit - temperatureRampThreshold, 1.0);  // Safety margin - don't want divide by 0!

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        differentialDrive.setSafetyEnabled(true);

        m_slew_rate_value = 0.5;
        m_shuffle_slew_rate = Shuffleboard.getTab("Control").add("Drive Slew Rate", 1.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", .5, "max", 2.5)).getEntry();
        m_shuffle_slew_rate.addListener(event -> { m_slew_rate_value = event.value.getDouble(); }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        m_turn_slew_rate_value = 5;
        m_shuffle_turn_slew_rate = Shuffleboard.getTab("Control").add("Turn Slew Rate", 1.5).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1.0, "max", 10.0)).getEntry();
        m_shuffle_turn_slew_rate.addListener(event -> { m_turn_slew_rate_value = event.value.getDouble(); }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drive RightS Current", rightSlaveCurrent.update());
        SmartDashboard.putNumber("Drive RightM Current", rightMasterCurrent.update());
        SmartDashboard.putNumber("Drive LeftM Current", leftMasterCurrent.update());
        SmartDashboard.putNumber("Drive LeftS Current", leftSlaveCurrent.update());

        SmartDashboard.putNumber("Drive RightM Temp", rightMasterTemp.update());
        SmartDashboard.putNumber("Drive RightS Temp", rightSlaveTemp.update());
        SmartDashboard.putNumber("Drive LeftM Temp", leftMasterTemp.update());
        SmartDashboard.putNumber("Drive LeftS Temp", leftSlaveTemp.update());

        temp = Math.max(Math.max(rightMaster.getMotorTemperature(), leftMaster.getMotorTemperature()),
                        Math.max(rightSlave.getMotorTemperature(), leftSlave.getMotorTemperature()));

        double multiplier = 1;
        if(temp > temperatureRampThreshold){
            multiplier = Math.max((temperatureRampLimit - temp) / delta, 0.0);
        }

        if(tempWarningCount++ % 100 == 0){
            if (multiplier != 1.0) reportWarning("Safety speed control - factor: " + multiplier, true);
//            StatusLights.getInstance().setAscending( (int)((1.0-multiplier) * 5));
        }


        differentialDrive.setMaxOutput(multiplier);
//
//        if(multiplier == 1) StatusLights.getInstance().setAscending(1);
//        else if(multiplier >= .9) StatusLights.getInstance().setAscending(2);
//        else if(multiplier >= .8) StatusLights.getInstance().setAscending(3);
//        else if(multiplier >= .7) StatusLights.getInstance().setAscending(4);
//        else StatusLights.getInstance().setAscending(5);
    }

    public void toggleReverse() {
        reverse = !reverse;
    }

    public void togglePrecision() {
        precision = !precision;
    }

    public void driveArcade(double x, double z) {
        SmartDashboard.putNumber("Drive X", x);
        SmartDashboard.putNumber("Drive Z", z);

        //TODO: These are sticky and never getting reset to 0. Test effect of resetting in else
        if(Math.abs(x) > 0.15 && Math.abs(z) > 0.1){
            StormSpark.check(leftMaster.setOpenLoopRampRate(0.1), "set ramp rate");
            StormSpark.check(rightMaster.setOpenLoopRampRate(0.1), "set ramp rate");
        }

        x = precision ? xPrecisionMultiplier * x : x;
        z = precision ? zPrecisionMultiplier * z : z;

        x = reverse ? -x : x;

        if(Math.abs(x) < 0.1){
            differentialDrive.arcadeDrive(x, z);
        }else{
            differentialDrive.curvatureDrive(x, z, false);
        }
    }

    public double get_slew_rate() {
        return(m_slew_rate_value);
    }
    public double get_turn_slew_rate() {
        return(m_turn_slew_rate_value);
    }

    public void diagnosticDrive(double leftM, double leftS, double rightM, double rightS){
        // We usually only want to run one motor at a time in this mode, or run them at the same rate.
        // We don't want one motor to drag on the other on either side of the chassis
        if (Math.abs(leftM) > Math.abs(leftS)) {
            leftS = idleMode == CANSparkMax.IdleMode.kCoast ? 0 : leftM;
        } else if (Math.abs(leftM) < Math.abs(leftS)) {
            leftM = idleMode == CANSparkMax.IdleMode.kCoast ? 0 : leftS;
        }

        if (Math.abs(rightM) > Math.abs(rightS)) rightS = 0;
        else if (Math.abs(rightM) < Math.abs(rightS)) rightM = 0;

        leftMaster.set(leftM);
        leftSlave.set(leftS);
        rightMaster.set(rightM);
        rightSlave.set(rightS);
    }

    public void tankDrive(double left, double right){
//        differentialDrive.tankDrive(left, right);
        rightMaster.set(right);
        leftMaster.set(left);
    }
}