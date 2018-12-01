

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Size;

import com.disnodeteam.dogecv.detectors.roverrukus.SilverDetector;



import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;




@TeleOp(name="Drive_4788", group="Linear Opmode")

//@Disabled
public class Drive_4788 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor hexDrive1 = null;
    private DcMotor hexDrive2 = null;

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    // Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    WebcamName webcamName;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    // DogeCV detector
    GoldAlignDetector detector;

    @Override
    public void runOpMode() {
          // Default webcam name
          webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

          // Set up parameters for Vuforia
          int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
          VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
  
          // Vuforia licence key
          parameters.vuforiaLicenseKey = "AXaCj/3/////AAABmUzcrisvx0PutYdHAGy1V7FtTHCds9DE5p07vCMF4yHeSckVBZjgIUxogVRXLrTYGhGfWnBjK8qdR6M0L8+Hmhgka3pVPvIzc4xXqkFfDRJstgVlh93pdoQMcUTAUeLTQAoVp3PYMI7YiXGJxZ7kAEg1mZn4mbbXNBLZj/99ejGvXIAAC+VSDL3OjvIdbB4zw1hFn2zssPm8p2HPkbjYEFrlh/ZNhyiV2EQwU+DDZYvVWAVWoY4wh4JMniPSDcwM7U9UE6tgpurqphrFxogLi8j6vKwn3tZwbKUYzq9GjB7gueLUfLZwced7s09e6Ijkip/JWQcqr9jduWA/oI7qiyRkWxGN1Xmc4ahK+U3qtvC6";
          parameters.fillCameraMonitorViewParent = true;
  
          // Set camera name for Vuforia config
          parameters.cameraName = webcamName;
  
          // Create Dogeforia object
          vuforia = new Dogeforia(parameters);
          vuforia.enableConvertFrameToBitmap();
  
          //Setup trackables
          VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
          VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
          blueRover.setName("Blue-Rover");
          VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
          redFootprint.setName("Red-Footprint");
          VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
          frontCraters.setName("Front-Craters");
          VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
          backSpace.setName("Back-Space");
  
          // For convenience, gather together all the trackable objects in one easily-iterable collection */
          allTrackables.addAll(targetsRoverRuckus);
  
          OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                  .translation(0, mmFTCFieldWidth, mmTargetHeight)
                  .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
          blueRover.setLocation(blueRoverLocationOnField);
  
          OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                  .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                  .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
          redFootprint.setLocation(redFootprintLocationOnField);
  
          OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                  .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                  .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
          frontCraters.setLocation(frontCratersLocationOnField);
  
          OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                  .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                  .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
          backSpace.setLocation(backSpaceLocationOnField);
  
  
          final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
          final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
          final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
  
          OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                  .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                  .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                          CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
  
          for (VuforiaTrackable trackable : allTrackables)
          {
              ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
          }
  
          // Activate the targets
          targetsRoverRuckus.activate();
  
          // Initialize the detector
          detector = new GoldAlignDetector();
          detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
          detector.useDefaults();
          detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
          //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
          detector.downscale = 0.4;
  
          // Set the detector
          vuforia.setDogeCVDetector(detector);
          vuforia.enableDogeCV();
          vuforia.showDebug();
          vuforia.start();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        /*
        hexDrive1 = hardwareMap.get(DcMotor.class, "hex_motor1");
        hexDrive2 = hardwareMap.get(DcMotor.class, "hex_motor2");
        */

        

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
        hexDrive1.setDirection(DcMotor.Direction.FORWARD);
        hexDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hexDrive2.setDirection(DcMotor.Direction.FORWARD);
        hexDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */

        


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }


        //====================================
        //Gold Vision Tracking Start
        //====================================
        double goal = 290;
        double kP = 0.01;
        double output;
        double error;
        double visionRightPowerStraight = 1.0;
        double visionLeftPowerStraight = 1.0;
       
        while(gamepad1.a){



            if (targetVisible = true) { // If object is directly in front of it
                leftDrive.setPower(-visionLeftPowerStraight);
                rightDrive.setPower(-visionRightPowerStraight);

            }

            if (detector.getXPosition() > 290) { // If object is on the right 
                error = goal - detector.getXPosition();

                output = kP * error;
    
                leftDrive.setPower(output);
            }

            if (detector.getXPosition() < 290) { // If object is on the left
                error = goal - detector.getXPosition();

                output = kP * error;
    
                leftDrive.setPower(-output);
                
            }
           





        //double visionLeftPowerClose = 0.2;
        //double visionRightPowerClose = 0.2;
        //double visionRightPowerFar = 0.4;
        //double visionLeftPowerFar = 0.4;
        //double visionRightPowerStraight = 0.6;
        //double visionLeftPowerStraight = 0.6;
            /*

            if (targetVisible = true) {
                leftDrive.setPower(-visionLeftPowerStraight);
                rightDrive.setPower(-visionRightPowerStraight);

            }
            if (detector.getXPosition() < 350) {
                leftDrive.setPower(-visionLeftPowerStraight);
                rightDrive.setPower(-visionRightPowerStraight);
            }
            if (detector.getXPosition() > 230) {
                leftDrive.setPower(-visionLeftPowerStraight);
                rightDrive.setPower(-visionRightPowerStraight);
            }


            //Far
            if(detector.getXPosition() > 480){
                leftDrive.setPower(-visionLeftPowerFar);
                rightDrive.setPower(visionRightPowerFar);
            }
            
            
            if(detector.getXPosition() < 100){
                leftDrive.setPower(visionLeftPowerFar);
                rightDrive.setPower(-visionRightPowerFar);
            }
            //close
            if(detector.getXPosition() > 350){
                leftDrive.setPower(-visionLeftPowerClose);   //230 350 = center
                rightDrive.setPower(visionRightPowerClose);
            }
            
            
            if(detector.getXPosition() < 230){
                leftDrive.setPower(visionLeftPowerClose);
                rightDrive.setPower(-visionRightPowerClose);
            }
            */



            /*else{
                leftDrive.setPower(-visionLeftPower);
                rightDrive.setPower(-visionRightPower);
            }*/
            //====================================
            //Gold Vision Tracking Start
            //====================================
            
            

        }




            // --------------------------------------------------------------------------------------------------------
            // Drive Motor Section
            // --------------------------------------------------------------------------------------------------------

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
          


                // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight..3.\10
            double drive = -gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;

            leftPower    = Range.clip(drive + turn, 0.2, 1.0) ;
            rightPower   = Range.clip(drive + turn, 0.2, 1.0) ;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            //leftPower  = gamepad1.left_stick_y ;
            //rightPower = gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(-leftPower);
            rightDrive.setPower(-rightPower);






            /*
            // --------------------------------------------------------------------------------------------------------
            // Extra Motor Section
            // --------------------------------------------------------------------------------------------------------
            // Assigns 0% power to hex_motors/ arm power, x and x for low power, a and b for high power.
            float armPower = 0;
            if (gamepad1.x) armPower += 0.3; // assigns %30 power when press x
            if (gamepad1.y) armPower -= 0.3;

            //if (gamepad1.a) armPower += 1.0; // assigns %100 power when press a
            //if (gamepad1.b) armPower -= 1.0;

            hexDrive1.setPower(armPower); // sets motor power to gamepad button.
            hexDrive2.setPower(-armPower);
            */





            // ---------------------------------------------------------------------------------
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
