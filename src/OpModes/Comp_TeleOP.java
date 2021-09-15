package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Competition_TeleOp", group = "")
public class Competition_TeleOp extends LinearOpMode {

    public DcMotor frontrightdrive;
    public DcMotor rearrightdrive;
    public DcMotor frontleftdrive;
    public DcMotor rearleftdrive;
    public DcMotor Arm;
    public DcMotor leftlaunch;
    public DcMotor rightlaunch;
    public DcMotor Conveyor;
    public Servo ArmClamp;
    public Servo PinBallPaddle;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        frontrightdrive = hardwareMap.dcMotor.get("frontrightdrive");
        rearrightdrive = hardwareMap.dcMotor.get("rearrightdrive");
        frontleftdrive = hardwareMap.dcMotor.get("frontleftdrive");
        rearleftdrive = hardwareMap.dcMotor.get("rearleftdrive");
        leftlaunch = hardwareMap.dcMotor.get("leftlaunch");
        rightlaunch = hardwareMap.dcMotor.get("rightlaunch");
        Arm = hardwareMap.dcMotor.get("Arm");
        Conveyor = hardwareMap.dcMotor.get("Conveyor");
        ArmClamp = hardwareMap.get(Servo.class,"ArmClamp");
        PinBallPaddle = hardwareMap.get(Servo.class,"PinBallPaddle");

        double ServoPosition;
        double ServoSpeed;
        double PaddlePosition;
        double RightLaunchSpeed;
        double LeftLaunchSpeed;
        double ConveyorSpeed;
        double SpeedMod;

        /// Set motor direction
        frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftlaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        rightlaunch.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        Conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set Motor Speed
        SpeedMod = 0.75;
        LeftLaunchSpeed = 0.0;
        RightLaunchSpeed = 0.0;
        ConveyorSpeed = 0.0;
        // Set servos starting position
        ServoPosition = 0.9;
        ArmClamp.setPosition(ServoPosition);
        PaddlePosition = 0.62;
        PinBallPaddle.setPosition(PaddlePosition);
        ServoSpeed = 0.5;

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                /// Launch speeds for high tower goal
                /// New higher charged battery lower speed
                if (gamepad1.right_bumper) {
                    LeftLaunchSpeed = 0.40;
                    RightLaunchSpeed = 0.20;
                    /// Old lower chargeed battery
                } if (gamepad1.left_bumper) {
                    LeftLaunchSpeed = 0.42;
                    RightLaunchSpeed = 0.20;
                    /// Launch speeds for power shots and mid goal
                    /// New higher charged battery lower speed
                } if (gamepad1.a) {
                    LeftLaunchSpeed = 0.38;
                    RightLaunchSpeed = 0.20;
                    /// Old lower chargeed battery
                } if (gamepad1.y) {
                    LeftLaunchSpeed = 0.35;
                    RightLaunchSpeed = 0.20;
                    /// low goal speed
                } if (gamepad1.b) {
                    LeftLaunchSpeed = 0.20;
                    RightLaunchSpeed = 0.20;
                    /// Stop launch motors
                } if (gamepad1.x) {
                    LeftLaunchSpeed = 0.00;
                    RightLaunchSpeed = 0.00;
                    /// Trigger Control - Feed Rings into Launch Motors
                } if (gamepad1.left_trigger > 0.1) {
                    PaddlePosition = 0.62;
                } if (gamepad1.right_trigger > 0.1) {
                    PaddlePosition = 0.96;
                    /// Intake conveyor speed selections
                }if (gamepad2.b) {
                    ConveyorSpeed = 0.65;
                }if (gamepad2.x) {
                    ConveyorSpeed = 0.00;
                }if (gamepad2.y) {
                    ConveyorSpeed = 0.80;
                }if (gamepad2.a) {
                    ConveyorSpeed = 0.95;
                    /// Intake conveyor speed direction
                } if (gamepad2.left_trigger > 0.1) {
                    Conveyor.setDirection(DcMotorSimple.Direction.FORWARD);
                } if (gamepad2.right_trigger > 0.1) {
                    Conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
                }

                /// control motors and servos
                double vert = gamepad1.right_stick_x;
                double hori = gamepad1.left_stick_x;
                double pivot = 1 * gamepad1.left_stick_y;
                double vert2 = gamepad2.right_stick_x;
                double hori2 = gamepad2.left_stick_x;
                double pivot2 = 1 * gamepad2.left_stick_y;
                frontleftdrive.setPower(SpeedMod * (-pivot + vert + hori));
                rearleftdrive.setPower(SpeedMod * (-pivot + (vert - hori)));
                frontrightdrive.setPower(SpeedMod * (pivot + (vert - hori)));
                rearrightdrive.setPower(SpeedMod * (pivot + vert + hori));
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                Conveyor.setPower(ConveyorSpeed);
                PinBallPaddle.setPosition(PaddlePosition);
                Arm.setPower(0.60 * (pivot2 + vert2 + hori2));
                ArmClamp.setPosition(ServoPosition);
                // Use gamepad left_bumper to open and right_bumper to close servo
                if (gamepad2.left_bumper) {
                    ServoPosition = 1;
                }
                if (gamepad2.right_bumper) {
                    ServoPosition = 0.3;
                }
                /// Telemetry
                telemetry.addData("vert:", vert);
                telemetry.addData("hori:", hori);
                telemetry.addData("pivot:", pivot);
                telemetry.addData("Servo", ServoPosition);
                telemetry.update();
            }
        }
    }
}