package frc.robot.subsytems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arms extends SubsystemBase {

    TalonSRX arm1 = new TalonSRX(27);

    private final PIDController extentionPID = new PIDController(0, 0, 0);

    // private
    public Arms() {
    }

    public void setArmLimitSetpoint(double setpoint) {
        extentionPID.setSetpoint(MathUtil.clamp(setpoint, 0, 0));
    }

    // public void upDown(boolean choice) {
    // if (choice) {
    // arm1.set(TalonSRXControlMode.PercentOutput, .5);
    // } else {
    // arm1.set(TalonSRXControlMode.PercentOutput, -.5);
    // }
    // }

    public void armsUp() {
        arm1.set(TalonSRXControlMode.PercentOutput, 0.8);
    }

    public void armsDown() {
        arm1.set(TalonSRXControlMode.PercentOutput, -0.8);
    }

    public void turnOff() {
        arm1.set(TalonSRXControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {

    }
    // public void down(boolean choice) {
    // if (choice) {
    // arm1.set(-.5);
    // arm2.set(-.5);
    // }
    // }

}
