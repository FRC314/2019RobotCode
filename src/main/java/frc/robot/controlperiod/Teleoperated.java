package frc.robot.controlperiod;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.robotmechanism.CargoManagement;
import frc.robot.robotmechanism.Drivetrain;
import frc.robot.robotmechanism.HatchMech;

public class Teleoperated {
    private XboxController ctl_Driver;
    private XboxController ctl_Operator;
    private Drivetrain rbt_Drivetrain;
    private HatchMech rbt_HatchMech;
    private CargoManagement rbt_CargoManagement;

    public Teleoperated(XboxController ctl_Driver, XboxController ctl_Operator, Drivetrain rbt_Drivetrain, HatchMech rbt_HatchMech, CargoManagement rbt_CargoManagement) {
        this.ctl_Driver = ctl_Driver;
        this.ctl_Operator = ctl_Operator;
        this.rbt_Drivetrain = rbt_Drivetrain;
        this.rbt_HatchMech = rbt_HatchMech;
        this.rbt_CargoManagement = rbt_CargoManagement;
    }

    public void arcadeDrive(double t, double s) {
        rbt_Drivetrain.setDrive(t - s, t + s);
    }

    public void update() {
        rbt_Drivetrain.disableAnglePID();
        //Drivetrain
        if (ctl_Driver.getYButton()) {
            rbt_Drivetrain.resetDistance();
        }
        if (ctl_Driver.getAButton()) {
            rbt_Drivetrain.GoToDistance(10);
        }
        else {
            rbt_Drivetrain.disableDistancePID();
            arcadeDrive(ctl_Driver.getY(Hand.kLeft), ctl_Driver.getX(Hand.kRight));
        }
        //Hatchmech
        if (ctl_Operator.getAButton()) {
            rbt_HatchMech.open();
        }
        else if (ctl_Operator.getBButton()){
            rbt_HatchMech.close();
        }
        else if (ctl_Operator.getStartButton() && ctl_Operator.getBackButton()) {
            rbt_HatchMech.moveToZeroAngle();
        }
        else {
            rbt_HatchMech.setHatchPower(0.0);
        }
        //Cargo management
        if (ctl_Operator.getTriggerAxis(Hand.kRight) > 0.1 && rbt_CargoManagement.isObjectDetected() == false) {
            rbt_CargoManagement.setIntakePower(1.0);
        }
        else if (ctl_Operator.getBumper(Hand.kRight) && rbt_CargoManagement.isObjectDetected()) {
            rbt_CargoManagement.setIntakePower(1.0);
        }
        else {
            rbt_CargoManagement.setIntakePower(0.0);
        }

        if (ctl_Operator.getXButton()) {
            rbt_CargoManagement.setRotationPower(1.0);
        }
        else if (ctl_Operator.getYButton()) {
            rbt_CargoManagement.setRotationPower(-1.0);
        }
        else {
            rbt_CargoManagement.setRotationPower(0.0);
        }
    }
}