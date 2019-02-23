package frc.robot.configuration;

import java.lang.Math;

public class ConfigRobot {
    public static final double m_Pi = Math.PI;
    public static final double drivetrainWheelDiameter = 6.125;
    public static final double drivetrainWheelCircumference = m_Pi * drivetrainWheelDiameter;
    public static final double countsPerInch = drivetrainWheelCircumference / 1024;
}