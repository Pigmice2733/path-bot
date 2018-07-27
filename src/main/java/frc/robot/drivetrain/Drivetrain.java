package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain {
    private DifferentialDrive drive;

    public Drivetrain(SpeedController leftDrive, SpeedController rightDrive) {
        drive = new DifferentialDrive(leftDrive, rightDrive);
    }

    public void arcadeDrive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }
}
