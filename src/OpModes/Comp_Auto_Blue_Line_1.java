package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Set;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous
/// based on Autonomous LCQ
public class Competition_Auto_Blue_Line_1 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public DcMotor frontrightdrive;
    public DcMotor rearrightdrive;
    public DcMotor frontleftdrive;
    public DcMotor rearleftdrive;
    public DcMotor leftlaunch;
    public DcMotor rightlaunch;
    public DcMotor Arm;
    public Servo ArmClamp;
    public Servo PinBallPaddle;
    //public Servo IntakeUp;
    //public Servo IntakeDown;

    // create an object to measure the amount of time the program has run
    ElapsedTime runTime = new ElapsedTime();


    private static final String VUFORIA_KEY =
            "ASyCP+z/////AAABmcllA55Hdk8FmZ6iBqhQ6PETm6z/lZMm8MFsoYydCT9Rv4nyjbtbvAwMeYvbZ7sS6cLkJco8tlgCM+41BzqB8ZNS6O4iQgKaImsWX3f6gr4JTR1ruxyewMLEg3o1IOzqfL9yATJfQBRlQNcvjSVo8z+svQb3rgXpWlWZP/UBk2pVqNmEB3w37FSFsTZ+y2Gn4cTgeRM9PB0kQIRvUUcERUXklNbl0NIu8RUg54LeazrdB7m8yLfjA+pJrmeadFHNEetyl02EMvgiE4F0az0B+WtGFweiKa0SMNw03ZCzmJAz00olVUypC9xJAXTm5Pcx0+7JjCADTVWL8uWhziK2TTkIW65k39ST/1H+t7S2ts8j";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        telemetry.addData("Initializing the robot...", "please wait");
        telemetry.update();

        frontrightdrive = hardwareMap.dcMotor.get("frontrightdrive");
        rearrightdrive = hardwareMap.dcMotor.get("rearrightdrive");
        frontleftdrive = hardwareMap.dcMotor.get("frontleftdrive");
        rearleftdrive = hardwareMap.dcMotor.get("rearleftdrive");
        leftlaunch = hardwareMap.dcMotor.get("leftlaunch");
        rightlaunch = hardwareMap.dcMotor.get("rightlaunch");
        Arm = hardwareMap.dcMotor.get("Arm");
        ArmClamp = hardwareMap.get(Servo.class,"ArmClamp");
        PinBallPaddle = hardwareMap.get(Servo.class,"PinBallPaddle");
        //IntakeUp = hardwareMap.get(Servo.class,"IntakeUp");
        //IntakeDown = hardwareMap.get(Servo.class,"IntakeDown");
        double ServoPosition;
        double ServoSpeed;
        /// ALL SETTINGS HAVE BEEN TESTED
        // For High Goals in Auto
        // battery at 12.65V left 0.49 and right 0.20
        // battery below 12.5V left 0.50 and right 0.20
        // battery at 12.3V left 0.52 and right 2.0
        ///  OLD SETTINGS FOR ORIGINAL AUTO PRE-LCQ going for Power shots
        ///  OLD SETTINGS FOR ORIGINAL AUTO PRE-LCQ goung for Power shots
        // low low battery 0.46 and 0.20 - 12.8 to 12.2 volts
        // low battery 0.44 and 0.20 - 13.2 to 12.8 volts
        // -- NEED TO TEST - mid battery 0.42 and 0.20 - 13.5 or 13.2 volts
        // new battery 0.38 and 0.20 - 13.7 or 13.5 volts
        double LeftLaunchSpeed = 0.50;
        double RightLaunchSpeed = 0.20;
        //double IntakeUpPosition;
        //double IntakeDownPosition;

        int position;
        int counter;

// set drive motors for drive forward to get in poition to see rings - 760 target position
        frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
// Set direction of launch motors
        leftlaunch.setDirection(DcMotorSimple.Direction.REVERSE);
        rightlaunch.setDirection(DcMotorSimple.Direction.FORWARD);
        // Set Motor Speed
        double SpeedMod = 0.75;
        double SpeedModFast = 0.95;
        double PaddleResetPos = 0.60;
        double PaddleShootPos = 0.96;
        // Set servo to close position, paddle to ready position and intake to down position
        ArmClamp.setPosition(0.9);
        PinBallPaddle.setPosition(PaddleResetPos);
        //IntakeUpPosition = -0.70;
        //IntakeUp.setPosition(IntakeUpPosition);
        //IntakeDownPosition = 0.17;
        //IntakeDown.setPosition(IntakeDownPosition);
        ServoSpeed = 0.5;
        position = 1;
        counter = 0;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        } else {
            telemetry.addData("WARNING: ", "TFOD Not Initialized properly!!!");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Ready to go!! Good luck!");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            ArmClamp.setPosition(0.9);
            PinBallPaddle.setPosition(PaddleResetPos);
            Arm.setPower(0.00);
            //IntakeUpPosition = 0.98;
            //IntakeDownPosition = 0.49;
            //IntakeUp.setPosition(IntakeUpPosition);
            //IntakeDown.setPosition(IntakeDownPosition);

// motors are set to for drive forward to get in poition to see rings - 760 target position
            frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            distance(760, 760, 760, 760);

            frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

            while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
            }

// Set motor direction and slight pivot right to see rings - 467 target position
            frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
            frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
            rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
            frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            distance(467, 467, 467, 467);

            frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

            while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
            }

            // set motor power for no movement and view rings
            drive(0, 0, 0, 0);

            // create variable to track robot program runtime
            double startTime = runTime.time();

            // while loop checks rings for target zone
            while (tfod != null && (runTime.time() - startTime) <= 1.5) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if ((updatedRecognitions != null)) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.addData("  position",position );
                        if (recognition.getLabel() == "Quad") position = 3;
                        else if (recognition.getLabel() == "Single") position = 2;
                    }   // end of for(Recogn...)
                    telemetry.update();
                }   // end of if ((updated...)
            }       // end of while(tfod...)

// *****************************************************************************
// ***************************** Target Zone B *********************************
// *****************************************************************************

            if (position == 2){
                // Pivot left toward Target Zones at an angle to be out of way of other robot
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);

                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                /// was 437 in LCQ - Changed to higher value for champions ship to over piivot and drive to target zone A at angle
                distance(750, 750, 750, 750);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                sleep(250);


                // set motors and drive forward to toward target zone A before turn back straight
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(1950, 1950, 1950, 1950);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                /// Stop robot drivetrain
                drive(0, 0, 0, 0);
                sleep(150);

                // Set motor direction and slight pivot right to drive straight down wall towards target Zone

                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(330, 330, 330, 330);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }

                drive(0, 0, 0, 0);
                sleep(150);

                // set motors and drive forward to target zone

                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(2800, 2800, 2800, 2800);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }

                // Stop motor power
                drive(0, 0, 0, 0);
                sleep(150);

                // Pivot right to place in target zone

                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);

                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(750, 750, 750, 750);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){

                }

                // stop drive motors
                drive(0, 0, 0, 0);

                // Set Arm Motor Direction, Target Position, and Power and use are to deliver wobble goal
                Arm.setDirection(DcMotorSimple.Direction.REVERSE);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(505);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.85);
                while (Arm.isBusy()){
                }
                sleep(250);
                ArmClamp.setPosition(0.3);
                sleep(500);

                // Set Arm Motor Direction, Target Position, and Power and use are to raise arm slightly to get ready for second wobble goal
                Arm.setDirection(DcMotorSimple.Direction.FORWARD);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(250);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.85);
                while (Arm.isBusy()){
                }

                // Pivot back left to line up for back up behind line to shoot

                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);

                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(735, 735, 735, 735);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){

                }

                /// Stop robot drivetrain
                drive(0, 0, 0, 0);
                sleep(150);

                // back up behind line to shoot

                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(2000, 2000, 2000, 2000);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }

                /// Stop robot drivetrain
                drive(0, 0, 0, 0);
                sleep(150);
                // Pivot right slightly to shoot rings

                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);

                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(350, 350, 350, 350);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){

                }

                // stop drive motors
                drive(0, 0, 0, 0);

                // Set launch motors speed and pin up launch motors
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(2500);
                // Shoot 1st ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(2250);

                // Slight pivot right to line up for power shot 2
                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(10, 10, 10, 10);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){

                }
                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Shoot 2nd ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(1550);

                // Slight pivot right to line up for power shot 3
                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(10, 10, 10, 10);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){

                }
                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Shoot 3rd ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(0.0);
                rightlaunch.setPower(0.0);
                PinBallPaddle.setPosition(PaddleResetPos);

                sleep(6500);

                // set motors and drive forward to navigate line
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(250, 250, 250, 250);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Set Arm Motor Direction, Target Position, and Power and use are to drop arm over line for navigatgion
                Arm.setDirection(DcMotorSimple.Direction.REVERSE);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(250);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.50);
                while (Arm.isBusy()){
                }
                Arm.setPower(0.0);


// *****************************************************************************
// ***************************** Target Zone C *********************************
// *****************************************************************************
            }   else if (position == 3){

                // Pivot left toward Target Zones at an angle to be out of way of other robot
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);

                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                /// was 437 in LCQ - Changed to higher value for champions ship to over piivot and drive to target zone A at angle
                distance(750, 750, 750, 750);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                sleep(250);


                // set motors and drive forward to toward target zone A before turn back straight
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(1950, 1950, 1950, 1950);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                /// Stop robot drivetrain
                drive(0, 0, 0, 0);
                sleep(150);

// Set motor direction and slight pivot right to drive straight down wall towards target Zone

                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(337, 337, 337, 337);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }

                drive(0, 0, 0, 0);
                sleep(150);

                // set motors and drive forward to target zone

                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(3850, 3850, 3850, 3850);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                // stop drive motors
                drive(0, 0, 0, 0);
                sleep(150);
                // Set motor direction and slight pivot left to place wobble goal in target Zone

                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(357, 357, 357, 357);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }

                // stop drive motors
                drive(0, 0, 0, 0);
                sleep(150);

                // Set Arm Motor Direction, Target Position, and Power and use are to deliver wobble goal
                Arm.setDirection(DcMotorSimple.Direction.REVERSE);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(505);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.85);
                while (Arm.isBusy()){
                }
                ArmClamp.setPosition(0.3);
                sleep(500);

                // Set Arm Motor Direction, Target Position, and Power and use are to raise arm slightly to get ready for second wobble goal
                Arm.setDirection(DcMotorSimple.Direction.FORWARD);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(250);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.85);
                while (Arm.isBusy()){
                }

                // Set motor direction and slight pivot right to line up for back up behind line to shoot

                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(350, 350, 350, 350);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }

                // stop drive motors
                drive(0, 0, 0, 0);
                sleep(150);

                // set motors and drive backward to shoot
                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(3200, 3200, 3200, 3200);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Set direction & pivot right to shoot
                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(325, 325, 325, 325);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){

                }

                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Set direction of launch motors
                leftlaunch.setDirection(DcMotorSimple.Direction.REVERSE);
                rightlaunch.setDirection(DcMotorSimple.Direction.FORWARD);

                // Set launch motors speed and pin up launch motors
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(2500);
                // Shoot 1st ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(1750);

                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Shoot 2nd ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(1750);


                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Shoot 3rd ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(750);
                // Shoot extra ring and reset pinballpaddlein case ring stuck
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(0.0);;
                rightlaunch.setPower(0.0);
                PinBallPaddle.setPosition(PaddleResetPos);

                // wait for other team to complete Autonomous before park
                sleep(5500);

                // Set direction & pivot left to navigate
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(325, 325, 325, 325);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }


                // Set Arm Motor Direction, Target Position, and Power and use navigate line
                Arm.setDirection(DcMotorSimple.Direction.REVERSE);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(250);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.55);
                while (Arm.isBusy()){
                }
                Arm.setPower(0.0);

                /// Stop robot drivetrain
                drive(0, 0, 0, 0);


            } else {
// *****************************************************************************
// ***************************** Target Zone A *********************************
// *****************************************************************************


                // Pivot left toward Target Zones
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearleftdrive.setDirection(DcMotorSimple.Direction.FORWARD);

                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                /// was 437 in LCQ - Changed to higher value for champions ship to over piivot and drive to target zone A at angle
                distance(740, 740, 740, 740);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                sleep(250);

                // set motors and drive forward to deliver swobble goal
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(2000, 2000, 2000, 2000);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedMod, SpeedMod, SpeedMod, SpeedMod);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Set Arm Motor Direction, Target Position, and Power and use are to deliver wobble goal
                Arm.setDirection(DcMotorSimple.Direction.REVERSE);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(505);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.85);
                while (Arm.isBusy()){
                }
                ArmClamp.setPosition(0.3);
                sleep(500);

                // Set Arm Motor Direction, Target Position, and Power and pull arm back
                Arm.setDirection(DcMotorSimple.Direction.FORWARD);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(250);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.75);
                while (Arm.isBusy()){
                }

                /// Wait for other team to complete autonomous or at least be out of the way
                sleep(10000);

                // Set direction & pivot right to shoot
                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(600, 600, 600, 600);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){

                }

                /// Stop robot drivetrain
                drive(0, 0, 0, 0);
                // Set direction of launch motors
                leftlaunch.setDirection(DcMotorSimple.Direction.REVERSE);
                rightlaunch.setDirection(DcMotorSimple.Direction.FORWARD);

                // Set launch motors speed and pin up launch motors
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(2500);
                // Shoot 1st ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(1500);

                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Shoot 2nd ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(1250);


                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Shoot 3rd ring and reset pinballpaddle
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(750);
                // Shoot extra ring and reset pinballpaddlein case ring stuck
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                PinBallPaddle.setPosition(PaddleShootPos);
                leftlaunch.setPower(LeftLaunchSpeed);
                rightlaunch.setPower(RightLaunchSpeed);
                sleep(750);
                leftlaunch.setPower(0.0);;
                rightlaunch.setPower(0.0);
                PinBallPaddle.setPosition(PaddleResetPos);
                sleep(750);

                // Set direction & pivot right to navigate
                frontrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearrightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(350, 350, 350, 350);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }

                // set motors and drive forward to navigate line
                frontrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                rearrightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
                frontleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                rearleftdrive.setDirection(DcMotorSimple.Direction.REVERSE);
                frontleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearrightdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rearleftdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                distance(1450, 1450, 1450, 1450);

                frontleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearrightdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rearleftdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive(SpeedModFast, SpeedModFast, SpeedModFast, SpeedModFast);

                while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && rearrightdrive.isBusy() && rearleftdrive.isBusy()){
                }
                /// Stop robot drivetrain
                drive(0, 0, 0, 0);

                // Set Arm Motor Direction, Target Position, and Power and use are to drop arm over line for navigatgion
                Arm.setDirection(DcMotorSimple.Direction.REVERSE);
                Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Arm.setTargetPosition(250);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(0.50);
                while (Arm.isBusy()){
                }
                Arm.setPower(0.0);
            }
        }       // end of if(opModeIsActive)

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    /// Set up drive method
    private void drive(double FL, double FR, double RL, double RR){
        frontleftdrive.setPower(FL);
        frontrightdrive.setPower(FR);
        rearleftdrive.setPower(RL);
        rearrightdrive.setPower(RR);
    }   // end of drive method    /// Set up drive method
    private void distance(int FLE, int FRE, int RLE, int RRE){
        frontleftdrive.setTargetPosition(FLE);
        frontrightdrive.setTargetPosition(FRE);
        rearleftdrive.setTargetPosition(RLE);
        rearrightdrive.setTargetPosition(RRE);
    }   // end of drive method

}
