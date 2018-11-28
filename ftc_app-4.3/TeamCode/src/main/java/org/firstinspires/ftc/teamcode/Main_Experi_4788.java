
package org.firstinspires.ftc.teamcode.dogecv;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Size;

import com.disnodeteam.dogecv.detectors.roverrukus.SilverDetector;






@TeleOp(name="Main_4788_Experimental", group="DogeCV")
//@Dissabled
public class Main_Experi_4788 extends OpMode
{
    // Detector objects
    private GoldAlignDetector detector;
    private SilverDetector detector2;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor hexDrive1 = null;
    private DcMotor hexDrive2 = null;


    @Override
    public void init() {
        //==================================================================================================
        //Gold Detector Start
        //==================================================================================================
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Alignment");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!
        //==================================================================================================
        //Gold Detector End
        //==================================================================================================






        //==================================================================================================
        //Silver Detector Start
        //==================================================================================================
        /*
        telemetry.addData("Status", "DogeCV 2018.0 - Gold SilverDetector Example");

        // Setup detector2
        detector2 = new SilverDetector(); // Create detector2
        detector2.setAdjustedSize(new Size(480, 270)); // Set detector2 size
        detector2.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector2 with app context and camera
        detector2.useDefaults(); // Set default detector2 settings
        // Optional tuning

        detector2.downscale = 0.4; // How much to downscale the input frames

        detector2.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector2.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector2.maxAreaScorer.weight = 0.005;

        detector2.ratioScorer.weight = 5;
        detector2.ratioScorer.perfectRatio = 1.0;
        detector2.enable(); // Start detector2
        */
        //==================================================================================================
        //Silver Detector End
        //==================================================================================================

    }

    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        hexDrive1 = hardwareMap.get(DcMotor.class, "hex_motor1");
        hexDrive2 = hardwareMap.get(DcMotor.class, "hex_motor2");

        

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        hexDrive1.setDirection(DcMotor.Direction.FORWARD);
        hexDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hexDrive2.setDirection(DcMotor.Direction.FORWARD);
        hexDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        


        // Wait for the game to start (driver presses PLAY)


    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
        telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
        //telemetry.addData("IsAligned" , detector2.getAligned()); // Is the bot aligned with the gold mineral?
        //telemetry.addData("X Pos" , detector2.getXPosition()); // Gold X position.

        double leftPower;
        double rightPower;
        double visionLeftPower = 0.3;
        double visionRightPower = 0.3;



        //====================================
        //Gold Vision Tracking Start
        //====================================

        while(gamepad1.right_bumper){

        
            if(detector.getAligned()){
                leftDrive.setPower(visionLeftPower);
                rightDrive.setPower(visionRightPower);
            }
            if(detector.getXPosition() > 0){
                leftDrive.setPower(-visionLeftPower);
                rightDrive.setPower(visionRightPower);
            }
            if(detector.getXPosition() < 0){
                leftDrive.setPower(visionLeftPower);
                rightDrive.setPower(-visionRightPower);
            }

        }

        //====================================
        //Gold Vision Tracking End
        //====================================





        //====================================
        //Silver Vision Tracking Start
        //====================================
        while(gamepad1.left_bumper){
            //if(detector2.getAligned()){

            //}


        }

        //====================================
        //Silver Vision Tracking End
        //====================================




        // --------------------------------------------------------------------------------------------------------
        // Extra Motor Section
        // --------------------------------------------------------------------------------------------------------
            // Assigns 0% power to hex_motors/ arm power, x and x for low power, a and b for high power.
            float armPower = 0;
            if (gamepad1.x) armPower += 0.3; // assigns %30 power when press x
            if (gamepad1.y) armPower -= 0.3;

            if (gamepad1.a) armPower += 1.0; // assigns %100 power when press a
            if (gamepad1.b) armPower -= 1.0;

            hexDrive1.setPower(armPower); // sets motor power to gamepad button.
            hexDrive2.setPower(-armPower);





        // --------------------------------------------------------------------------------------------------------
        // Main Drive Section
        // --------------------------------------------------------------------------------------------------------
        double drive = -gamepad1.left_stick_y;
        double turn  = -gamepad1.right_stick_x;

        leftPower    = Range.clip(drive + turn, 0.2, 1.0) ;
        rightPower    = Range.clip(drive + turn, 0.2, 1.0) ;


        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);




        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Disable the detector
        detector.disable();
    }

}
