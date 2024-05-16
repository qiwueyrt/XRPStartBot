package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

/** A command that will turn the robot to the specified angle. */
public class UltrasonicPID extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */

  public UltrasonicPID(double HoldDistanceMillimeters, Drivetrain drive) {
    super(
        new PIDController(DriveConstants.kUltrasonicP, DriveConstants.kUltrasonicI, DriveConstants.kUltrasonicD),
        // Close loop on heading
        drive::getUltrasonicSensor,
        // Set reference to target
        HoldDistanceMillimeters,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(output, 0),
        // Require the drive
        drive);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return false;
  }
}
