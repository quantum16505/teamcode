package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

@TeleOp(name = "Competition_TeleOp", group = "")
public class Competition_TeleOp extends LinearOpMode {

    HardwareProfile robot = new HardwareProfile();   // Use a Pushbot's hardware

    double left;
    double right;
    double drive;
    double turn;
    double max;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
//        robot.RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                drive = -gamepad1.left_stick_y;
                turn  =  gamepad1.right_stick_x;

                // Combine drive and turn for blended motion.
                left  = drive + turn;
                right = drive - turn;

                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0)
                {
                    left /= max;
                    right /= max;
                }

                // Output the safe vales to the motor drives
                robot.RearLeftDrive.setPower(left);
                robot.RearRightDrive.setPower(right);
                robot.FrontLeftDrive.setPower(left);
                robot.FrontRightDrive.setPower(right);
                telemetry.addData("Path0",  "Starting at %7d :%7d",
                        robot.RearLeftDrive.getCurrentPosition(),
                        robot.RearRightDrive.getCurrentPosition());
                telemetry.update();

            }
        }
    }
}