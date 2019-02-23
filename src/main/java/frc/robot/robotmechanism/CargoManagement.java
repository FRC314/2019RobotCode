package frc.robot.robotmechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.pidloops.PIDLoop;

public class CargoManagement {
    private VictorSPX mtr_IntakeRotate;
    private VictorSPX mtr_Intake;

    private double m_Power = 0.0;
    private double m_RotationPower = 0.0;

    private DigitalInput pho_Uptake;
    private boolean isObject = false;

    public CargoManagement(VictorSPX mtr_IntakeRotate, VictorSPX mtr_Intake, DigitalInput pho_Uptake) {
        this.mtr_IntakeRotate = mtr_IntakeRotate;
        this.mtr_Intake = mtr_Intake;
        this.pho_Uptake = pho_Uptake;
    }

    public void setRotationPower(double rotationPower) {
        m_RotationPower = rotationPower;
    }

    public void setIntakePower(double power) {
        m_Power = power;
    }

    public boolean isObjectDetected() {
        return isObject;
    }

    public void update() {
        mtr_Intake.set(ControlMode.PercentOutput, m_Power);
        mtr_IntakeRotate.set(ControlMode.PercentOutput, m_RotationPower);
        isObject = pho_Uptake.get();
    }
}