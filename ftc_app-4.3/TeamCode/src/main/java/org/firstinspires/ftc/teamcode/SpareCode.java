
// These include the libraries ==================================
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
// End of libraries ---------------------------------------------






// Telop Enabled ==================================================================================================================
// ================================================================================================================================
//=================================================================================================================================






@TeleOp(name="Spare Code", group="Linear Opmode")
//@Disabled
public class SpareCode extends LinearOpMode {

    // Declare OpMode members ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor hexDrive1 = null;
    private DcMotor hexDrive2 = null;
    // Finished OpMode ``````````````````````````````````````````````````````````










    // Main Code ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @Override
    public void runOpMode() {
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
        // Sets the motor to a constant state of on in both directions. To give the ability to break
        hexDrive1.setDirection(DcMotor.Direction.FORWARD);
        hexDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hexDrive2.setDirection(DcMotor.Direction.FORWARD);
        hexDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
         // run until the end of the match (driver presses STOP)





         // Quite Self Explanitary, While the Op mode is active the below code runs.
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight..3.\10
            double drive = -gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            //leftPower  = gamepad1.left_stick_y ;
            //rightPower = gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            float armPower = 0;
            if (gamepad1.a) armPower += 0.5;
            if (gamepad1.b) armPower -= 0.5;

            float SecondaryMotor = 0;
            if (gamepad1.x) SecondaryMotor += 0.5;
            if (gamepad1.y) SecondaryMotor -= 0.5;

            hexDrive1.setPower(armPower);
            hexDrive2.setPower(SecondaryMotor);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

// Teleop Enabled Finish ------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------
