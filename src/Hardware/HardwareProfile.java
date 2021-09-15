package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Map;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareProfile
{
    /* Public OpMode members. */
    public DcMotor frontrightdrive;
    public DcMotor rearrightdrive;
    public DcMotor frontleftdrive;
    public DcMotor rearleftdrive;
    public DcMotor rightlaunch;
    public DcMotor leftlaunch;
    public DcMotor Conveyor;
    //public Servo PinBallPaddle;
    //public DcMotor Arm;


    /*public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontrightdrive = hwMap.dcMotor.get("frontrightdrive");
        rearrightdrive = hwMap.dcMotor.get("rearrightdrive");
        frontleftdrive = hwMap.dcMotor.get("frontleftdrive");
        rearleftdrive = hwMap.dcMotor.get("rearleftdrive");
        leftlaunch = hwMap.dcMotor.get("leftlaunch");
        rightlaunch = hwMap.dcMotor.get("rightlaunch");
        Conveyor = hwMap.dcMotor.get("Conveyor");
        //PinBallPaddle = hwMap.Servo.get("PinBallPaddle");
        //Arm = hwMap.dcMotor.get("Arm");
        frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftlaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        rightlaunch.setDirection(DcMotorSimple.Direction.FORWARD);
        Conveyor.setDirection(DcMotorSimple.Direction.FORWARD);
        //Arm.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        frontleftdrive.setPower(0);
        frontrightdrive.setPower(0);
        rearleftdrive.setPower(0);
        rearrightdrive.setPower(0);
        leftlaunch.setPower(0);
        rightlaunch.setPower(0);
        Conveyor.setPower(0);
        //Arm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        /*leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/


    }
}

