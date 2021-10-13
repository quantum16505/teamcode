package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

@Autonomous(name="EncoderAuto")
public class EncoderAuto extends LinearOpMode {
    HardwareProfile robot = new HardwareProfile();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 384.5;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        encoderDrive(DRIVE_SPEED, 12.6, 12.6, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.RearLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.RearRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.RearLeftDrive.setTargetPosition(newLeftTarget);
            robot.RearRightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.RearLeftDrive.setPower(Math.abs(speed));
            robot.RearRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.RearLeftDrive.isBusy() && robot.RearRightDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.RearLeftDrive.getCurrentPosition(),
                        robot.RearRightDrive.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            robot.RearLeftDrive.setPower(0);
            robot.RearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
