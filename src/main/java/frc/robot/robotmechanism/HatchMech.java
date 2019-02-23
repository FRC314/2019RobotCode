package frc.robot.robotmechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.pidloops.EncoderLoop;
import frc.robot.pidloops.PIDLoop;

public class HatchMech {
    private VictorSPX mtr_Hatch;

    private double m_Power;
    
    private Encoder enc_Angle;
    private EncoderLoop pid_Angle;

    private DigitalInput lim_Hatch;

    public HatchMech(VictorSPX mtr_Hatch, Encoder enc_Angle, DigitalInput lim_Hatch) {
        this.mtr_Hatch = mtr_Hatch;
        this.enc_Angle = enc_Angle;
        this.lim_Hatch = lim_Hatch;
        pid_Angle = new EncoderLoop(0.1, 0.1, 0.1, enc_Angle);
    }

    public void ConfigureAnglePID(double P, double I, double D) { pid_Angle.setPID(P, I, D); }
	public void ConfigAngleOutputRange(double min, double max) { pid_Angle.setOutputRange(min, max); }
	public void GoToAngle(double angle) {
		//pid_Angle.setPID(0.024, 0.0, 0.0245);
		pid_Angle.setSetpoint(angle);
		pid_Angle.setAbsoluteTolerance(0.5);
        pid_Angle.enable();
    }

	public boolean isAnglePIDEnabled() { return pid_Angle.isEnabled(); }
	public boolean isAtAngle() { return pid_Angle.onTarget(); }
	public double getAngle() { return enc_Angle.get(); }
    public void disableAnglePID() { pid_Angle.disable(); }
    public double getP() { return pid_Angle.getP(); }
    public void resetAngle() { pid_Angle.resetSource(); }


    public void setHatchPower(double power) {
        m_Power = power;
    }

    public void open() {
        GoToAngle(90);
    }

    public void close() {
        GoToAngle(0);
    }

    public void moveToZeroAngle() {
        if (!lim_Hatch.get()) {
            setHatchPower(1.0);
        }
    }

    public void update() {
        if(isAnglePIDEnabled()){
            if(isAtAngle()) pid_Angle.disable();
            setHatchPower(pid_Angle.get());
        }
        mtr_Hatch.set(ControlMode.PercentOutput, m_Power);
    }
}