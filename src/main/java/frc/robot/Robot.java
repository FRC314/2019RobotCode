package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.robotmechanism.CargoManagement;
import frc.robot.robotmechanism.Drivetrain;
import frc.robot.robotmechanism.HatchMech;
import frc.robot.controlperiod.Teleoperated;

public class Robot extends TimedRobot {

  //Drivetrain
    TalonSRX mtr_L_Drive_1 = new TalonSRX(0);
    VictorSPX mtr_L_Drive_2 = new VictorSPX(1);
    VictorSPX mtr_L_Drive_3 = new VictorSPX(2);
    TalonSRX mtr_R_Drive_1 = new TalonSRX(3);
    VictorSPX mtr_R_Drive_2 = new VictorSPX(4);
    VictorSPX mtr_R_Drive_3 = new VictorSPX(5);
    //2 Encoders
    Encoder enc_Drive = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    Encoder enc_L_Drive = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
    AHRS imu_Drive = new AHRS(SerialPort.Port.kMXP);
    Limelight lml_Vision = new Limelight();
    Drivetrain rbt_Drivetrain = new Drivetrain(mtr_L_Drive_1, mtr_R_Drive_1, enc_Drive, imu_Drive, lml_Vision);

  //Mechanism Base

    //Hatch Mech
      VictorSPX mtr_Hatch = new VictorSPX(6);
      Encoder enc_HatchAngle = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
      DigitalInput lim_Hatch = new DigitalInput(6);

      HatchMech rbt_HatchMech = new HatchMech(mtr_Hatch, enc_HatchAngle, lim_Hatch);
    //Cargo Intake
      VictorSPX mtr_Intake = new VictorSPX(7);
      //Cargo Intake Position
      VictorSPX mtr_IntakePosition = new VictorSPX(8);
      Potentiometer pot_IntakeAngle = new AnalogPotentiometer(1, 270, 0);
      DigitalInput pho_Uptake = new DigitalInput(7);
      
    CargoManagement rbt_CargoManagement = new CargoManagement(mtr_IntakePosition, mtr_Intake, pho_Uptake);
  
  //Controls
    XboxController ctl_Driver = new XboxController(0);
    XboxController ctl_Operator = new XboxController(1);
  //ControlPeriods
    //Teleoperated
      Teleoperated prd_Teleoperated = new Teleoperated(ctl_Driver, ctl_Operator, rbt_Drivetrain, rbt_HatchMech, rbt_CargoManagement);

      Dashboard.NumberEntry dsh_DistanceP = new Dashboard.NumberEntry("Distance P");
      Dashboard.NumberEntry dsh_DistanceI = new Dashboard.NumberEntry("Distance I");
      Dashboard.NumberEntry dsh_DistanceD = new Dashboard.NumberEntry("Distance D");
  public void robotInit() {
    //Drivetrain
      //Left
        mtr_L_Drive_1.setInverted(true);
        mtr_L_Drive_2.setInverted(true);
        mtr_L_Drive_3.setInverted(true);

        mtr_L_Drive_2.follow(mtr_L_Drive_1);
        mtr_L_Drive_3.follow(mtr_L_Drive_1);
      
      //Right
        mtr_R_Drive_1.setInverted(false);
        mtr_R_Drive_2.setInverted(false);
        mtr_R_Drive_3.setInverted(false);

        mtr_R_Drive_2.follow(mtr_R_Drive_1);
        mtr_R_Drive_3.follow(mtr_R_Drive_1);

        rbt_Drivetrain.ConfigDistanceOutputRange(-0.5, 0.5);
    dsh_DistanceP.set(0.1);//0.1
    dsh_DistanceI.set(0.01);//0.01
    dsh_DistanceD.set(0.06);//0.06

  }

  @Override
  public void robotPeriodic() {
    rbt_Drivetrain.ConfigureDistancePID(dsh_DistanceP.get(), dsh_DistanceI.get(), dsh_DistanceD.get());
  }

  @Override
  public void autonomousInit() {
    
  }
  @Override
  public void autonomousPeriodic() {
    rbt_Drivetrain.update();
  }

  @Override
  public void teleopPeriodic() {
    prd_Teleoperated.update();
    rbt_Drivetrain.update();
  }

  @Override
  public void testPeriodic() {
  }
}
