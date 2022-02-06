package frc.utils.motorcontrol;

import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class PIDTunableSubsystem extends SubsystemBase {
    // private CANPIDController pidController;

    // private double kP;
    // private double kI;
    // private double kD;
    // private double kIz;
    // private double kFF;

    // public PIDTunableSubsystem(String name){
    //     super();
    //     setName(name);
    // }

    // //There were complications with a pid controller be passed into the constructor
    // protected final void initPidController(CANPIDController pidController){
    //     this.pidController = pidController;

    //     kP = pidController.getP();
    //     kI = pidController.getI();
    //     kD = pidController.getD();
    //     kIz = pidController.getIZone();
    //     kFF = pidController.getFF();

    //     SmartDashboard.putNumber(getName() + " -- P Gain", kP);
    //     SmartDashboard.putNumber(getName() + " -- I Gain", kI);
    //     SmartDashboard.putNumber(getName() + " -- D Gain", kD);
    //     SmartDashboard.putNumber(getName() + " -- I Zone", kIz);
    //     SmartDashboard.putNumber(getName() + " -- Feed Forward", kFF);
    // }

    // @Override
    // public void periodic() {
    //     double p = SmartDashboard.getNumber(getName() + " -- P Gain", 0);
    //     double i = SmartDashboard.getNumber(getName() + " -- I Gain", 0);
    //     double d = SmartDashboard.getNumber(getName() + " -- D Gain", 0);
    //     double iz = SmartDashboard.getNumber(getName() + " -- I Zone", 0);
    //     double ff = SmartDashboard.getNumber(getName() + " -- Feed Forward", 0);


    //     if(p != kP){
    //         pidController.setP(p);
    //         kP = p;
    //         System.out.println("Changed " + getName() + " kP to " + p);
    //     }
    //     if(i != kI){
    //         pidController.setI(i);
    //         kI = i;
    //         System.out.println("Changed " + getName() + " kI to " + i);
    //     }
    //     if(d != kD){
    //         pidController.setD(d);
    //         kD = d;
    //         System.out.println("Changed " + getName() + " kD to " + d);
    //     }
    //     if(iz != kIz){
    //         pidController.setIZone(iz);
    //         kIz = iz;
    //         System.out.println("Changed " + getName() + " kIz to" + i);
    //     }
    //     if(ff != kFF){
    //         pidController.setFF(ff);
    //         kFF = ff;
    //         System.out.println("Changed " + getName() + " kFF to" + ff);
    //     }
    // }
}