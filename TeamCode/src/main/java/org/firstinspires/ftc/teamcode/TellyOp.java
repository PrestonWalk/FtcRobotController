package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="the real actual telly")
//@Disabled
public class TellyOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final int LAUNCH_TARGET_VEL = 900;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        DcMotorEx launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo blocker = hardwareMap.get(CRServo.class, "blocker");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        //DcMotor leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        //DcMotor rightLift = hardwareMap.get(DcMotor.class, "right_lift");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        //leftLift.setDirection(DcMotor.Direction.REVERSE);
        //rightLift.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean flywheelRunning = false;
        boolean liftRunning = false;
        double liftStartedAt = 0;
        int shotProgress = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial; double lateral; double yaw;
            if(Math.abs(gamepad2.left_stick_x) + Math.abs(gamepad2.left_stick_y) + Math.abs(gamepad2.right_stick_x) > 0.2) {
                axial   = -gamepad2.left_stick_y  * 0.3;  // Note: pushing stick forward gives negative value
                lateral =  gamepad2.left_stick_x  * 0.3;
                yaw     =  gamepad2.right_stick_x * 0.3;
            } else {
                axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                lateral =  gamepad1.left_stick_x;
                yaw     =  gamepad1.right_stick_x;
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (gamepad1.left_bumper) max *= 3.0; // in case we need more precise movement
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Intake, overridden while launching
            if(shotProgress == 0) {
                if(gamepad1.x || gamepad1.square) intake.setPower(-0.8);
                else if(gamepad1.a || gamepad1.cross) intake.setPower(0.8);
                else intake.setPower(0.0);
            }
            // Launcher
            if(gamepad1.b || gamepad1.circle) flywheelRunning = true;
            else if(gamepad1.y || gamepad1.triangle) flywheelRunning = false;
            if(flywheelRunning) launcher.setPower(Math.min(1.0, Math.max(0.0, 1.0 - 0.01 * (launcher.getVelocity() - LAUNCH_TARGET_VEL))));
            else if(gamepad1.x || gamepad1.square) launcher.setPower(-1.0);
            else launcher.setPower(0.0);
            // Shoot!
            if(gamepad1.right_trigger > 0.1) {
                if(shotProgress == 0) {
                    // Step 1: Spin up the launcher
                    shotProgress = 1;
                    flywheelRunning = true; // Start the launcher if it isn't already
                } else if(shotProgress == 1 && launcher.getVelocity() > LAUNCH_TARGET_VEL) {
                    // Step 2: Spin the intake and move the blocker
                    shotProgress = 2;
                    intake.setPower(1.0);
                    blocker.setPower(1.0);
                }
            } else {
                blocker.setPower(0.0);
                shotProgress = 0;
            }
            // Lift
            if(gamepad2.a && !liftRunning) {
                //leftLift.setPower(1.0);
                //rightLift.setPower(1.0);
                liftStartedAt = runtime.milliseconds();
                liftRunning = true;
            } else if(gamepad2.b && !liftRunning) {
                //leftLift.setPower(-1.0);
                //rightLift.setPower(-1.0);
                liftStartedAt = runtime.milliseconds();
                liftRunning = true;
            }
            if(liftRunning && runtime.milliseconds() - liftStartedAt > 1000) { // stop the lift after a while
                //leftLift.setPower(0.0);
                //rightLift.setPower(0.0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}