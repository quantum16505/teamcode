package org.firstinspires.ftc.teamcode.Hardware;

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
    public DcMotor RearRightDrive;
    public DcMotor RearLeftDrive;
    public DcMotor FrontLeftDrive;
    public DcMotor FrontRightDrive;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        RearRightDrive = hwMap.get(DcMotor.class,"rearrightdrive");
        RearRightDrive.setDirection(DcMotor.Direction.FORWARD);
        RearRightDrive.setPower(0);

        RearLeftDrive = hwMap.get(DcMotor.class,"rearleftdrive");
        RearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RearLeftDrive.setPower(0);

        FrontLeftDrive = hwMap.get(DcMotor.class,"frontleftdrive");
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftDrive.setPower(0);

        FrontRightDrive = hwMap.get(DcMotor.class,"frontrightdrive");
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setPower(0);



    }
}

