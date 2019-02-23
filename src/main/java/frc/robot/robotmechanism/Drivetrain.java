package frc.robot.robotmechanism;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Limelight;
import frc.robot.pidloops.EncoderLoop;
import frc.robot.pidloops.IMULoop;
import frc.robot.pidloops.PIDLoop;
import frc.robot.configuration.ConfigRobot;

public class Drivetrain {
    private TalonSRX mtr_L_Drive_1;
    private TalonSRX mtr_R_Drive_1;

    private double m_L_Power = 0.0;
    private double m_R_Power = 0.0;

    private Encoder enc_Drive;
    private EncoderLoop pid_Drive;

    private AHRS imu_DriveAngle;
    private IMULoop pid_Angle;

    private Limelight lml_Vision;
    private PIDLoop pid_Limelight;

    public Drivetrain(TalonSRX mtr_L_Drive_1, TalonSRX mtr_R_Drive_1, Encoder enc_Drive, AHRS imu_DriveAngle, Limelight lml_Vision) {
        this.mtr_L_Drive_1 = mtr_L_Drive_1;
        this.mtr_R_Drive_1 = mtr_R_Drive_1;
        this.enc_Drive = enc_Drive;
        pid_Drive = new EncoderLoop(0.1, 0.1, 0.1, enc_Drive);
        this.enc_Drive.setDistancePerPulse(ConfigRobot.countsPerInch);
        this.imu_DriveAngle = imu_DriveAngle;
        pid_Angle = new IMULoop(0.1, 0.1, 0.1, imu_DriveAngle);
        this.lml_Vision = lml_Vision;
        pid_Limelight = new PIDLoop(0.1, 0.1, 0.1, lml_Vision);
    }

    //Angle
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
	public double getAngle() { return imu_DriveAngle.getAngle();}
    public void disableAnglePID() { pid_Angle.disable(); }
    public double getP() { return pid_Angle.getP(); }

    //Distance
    public void ConfigureDistancePID(double P, double I, double D) { pid_Drive.setPID(P, I, D); }
	public void ConfigDistanceOutputRange(double min, double max) { pid_Drive.setOutputRange(min, max); }
	public void GoToDistance(double distance) {
		//pid_Angle.setPID(0.024, 0.0, 0.0245);
		pid_Drive.setSetpoint(distance);
		pid_Drive.setAbsoluteTolerance(0.2);
        pid_Drive.enable();
    }

	public boolean isDistancePIDEnabled() { return pid_Drive.isEnabled(); }
	public boolean isAtDistance() { return pid_Drive.onTarget(); }
	public double getDistance() { return enc_Drive.get();}
    public void disableDistancePID() { pid_Drive.disable(); }
    public double getDistanceP() { return pid_Drive.getP(); }
    public void resetDistance() { pid_Drive.resetSource(); }

    //Limelight
    public void ConfigureLimelightPID(double P, double I, double D) { pid_Limelight.setPID(P, I, D); }
	public void ConfigLimeligtOutputRange(double min, double max) { pid_Limelight.setOutputRange(min, max); }
	public void FollowTarget() {
		//pid_Angle.setPID(0.024, 0.0, 0.0245);
		pid_Angle.setSetpoint(0);
		pid_Angle.setAbsoluteTolerance(0.5);
        pid_Angle.enable();
    }

	public boolean isLimelightPIDEnabled() { return pid_Limelight.isEnabled(); }
	public boolean isAtTarget() { return pid_Limelight.onTarget(); }
	public double getOffset() { return lml_Vision.getX();}
    public void disableLimelightPID() { pid_Limelight.disable(); }
    public double getLimelightP() { return pid_Limelight.getP(); }

    //Basic functions
    public void setDrive(double lPower, double rPower) {
        m_L_Power = lPower;
        m_R_Power = rPower;
    }

    public void update() {
        if(isAnglePIDEnabled()){
            if(isAtAngle()) pid_Angle.disable();
            setDrive(-pid_Angle.get(), pid_Angle.get());
        }
        if (isDistancePIDEnabled()) {
            if (isAtDistance()) pid_Drive.disable();
            setDrive(-pid_Drive.get(), -pid_Drive.get());
        }
        if (isLimelightPIDEnabled()) {
            if (isAtTarget()) pid_Limelight.disable();
            setDrive(-pid_Limelight.get(), pid_Limelight.get());
        }
        mtr_L_Drive_1.set(ControlMode.PercentOutput, m_L_Power);
        mtr_R_Drive_1.set(ControlMode.PercentOutput, m_R_Power);
    }
}