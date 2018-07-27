package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.drivetrain.Drivetrain;

public class Robot extends TimedRobot {
  private Drivetrain drivetrain;
  private Joystick joy;

  public void robotInit() {
    WPI_TalonSRX leftDrive = new WPI_TalonSRX(0);
    WPI_TalonSRX rightDrive = new WPI_TalonSRX(2);

    WPI_TalonSRX leftFollower = new WPI_TalonSRX(1);
    leftFollower.set(ControlMode.Follower, leftDrive.getDeviceID());

    WPI_TalonSRX rightFollower = new WPI_TalonSRX(3);
    rightFollower.set(ControlMode.Follower, rightDrive.getDeviceID());

    drivetrain = new Drivetrain(leftDrive, rightDrive);
    joy = new Joystick(0);
    setPeriod(0.02);
  }

  public void teleopPeriodic() {
    drivetrain.arcadeDrive(-joy.getY(), joy.getX());
  }
}
