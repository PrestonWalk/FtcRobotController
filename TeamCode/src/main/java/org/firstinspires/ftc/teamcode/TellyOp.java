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
@Disabled
public class TellyOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final int LAUNCH_TARGET_VEL = 900;
    private DcMotorEx rightLift;
    private DcMotorEx leftLift;
    int liftRunningStep = 0;

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
        Servo blocker2 = hardwareMap.get(Servo.class, "the_real_blocker");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean flywheelRunning = false;
        int shotProgress = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial;
            double lateral;
            double yaw;
            if (Math.abs(gamepad2.left_stick_x) + Math.abs(gamepad2.left_stick_y) + Math.abs(gamepad2.right_stick_x) > 0.2) {
                axial = -gamepad2.left_stick_y * 0.25;  // Note: pushing stick forward gives negative value
                lateral = gamepad2.left_stick_x * 0.25;
                yaw = gamepad2.right_stick_x * 0.25;
            } else {
                axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                lateral = gamepad1.left_stick_x;
                yaw = gamepad1.right_stick_x;
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (gamepad1.left_bumper) max *= 3.0; // in case we need more precise movement
            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // Intake, overridden while launching
            if (shotProgress == 0) {
                if (gamepad1.x || gamepad1.square) {
                    intake.setPower(-0.8);
                    blocker.setPower(-1.0);
                } else if (gamepad1.a || gamepad1.cross) {
                    intake.setPower(0.8);
                    blocker.setPower(0.0);
                } else {
                    intake.setPower(0.0);
                    blocker.setPower(0.0);
                }
            }
            // Launcher
            if (gamepad1.b || gamepad1.circle) flywheelRunning = true;
            else if (gamepad1.y || gamepad1.triangle) flywheelRunning = false;
            if (flywheelRunning)
                launcher.setPower(Math.min(1.0, Math.max(0.0, 1.0 - 0.01 * (launcher.getVelocity() - LAUNCH_TARGET_VEL))));
            else if (gamepad1.x || gamepad1.square) launcher.setPower(-1.0);
            else launcher.setPower(0.0);
            // Shoot!
            if (gamepad1.right_trigger > 0.1) {
                if (shotProgress == 0) {
                    // Step 1: Spin up the launcher
                    shotProgress = 1;
                    flywheelRunning = true; // Start the launcher if it isn't already
                } else if (shotProgress == 1 && launcher.getVelocity() > LAUNCH_TARGET_VEL) {
                    // Step 2: Spin the intake and move the blocker
                    shotProgress = 2;
                    intake.setPower(1.0);
                    blocker.setPower(1.0);
                    blocker2.setPosition(0.0);
                }
            } else {
                blocker.setPower(0.0);
                blocker2.setPosition(0.8);
                shotProgress = 0;
            }
            // blocker test thing
            if (gamepad2.left_bumper) {
                blocker2.setPosition(-1.0);
            } else if (gamepad2.right_bumper) {
                blocker2.setPosition(1.0);
            } else if (gamepad2.right_trigger > 0.1) {
                blocker2.setPosition(2 * gamepad2.right_trigger - 1);
            } else if (gamepad2.dpad_left) {
                blocker2.setPosition(0.0);
            }
            // Lift
            if (liftRunningStep == 0 && (gamepad2.x || gamepad2.square)) {
                boolean leftMaxed = leftLift.getCurrentPosition() >= 5850;
                boolean rightMaxed = rightLift.getCurrentPosition() >= 5850;
                if (leftLift.getCurrentPosition() > rightLift.getCurrentPosition()) {
                    leftLift.setPower(leftMaxed ? 0 : ((leftLift.getCurrentPosition() - rightLift.getCurrentPosition() > 300) ? 0.2 : 0.5));
                    rightLift.setPower(rightMaxed ? 0 : 0.7);
                } else {
                    leftLift.setPower(leftMaxed ? 0 : 0.7);
                    rightLift.setPower(rightMaxed ? 0 : ((rightLift.getCurrentPosition() - leftLift.getCurrentPosition() > 300) ? 0.2 : 0.5));
                }
            } else if (liftRunningStep == 0 && (gamepad2.y || gamepad2.triangle)) {
                leftLift.setPower(-0.5);
                rightLift.setPower(-0.5);
            } else if (liftRunningStep == 0 && (gamepad2.a || gamepad2.cross) && (gamepad2.left_trigger > 0.7)) { // needs to be more pressed!
                //doTheLift(0.5, 1);
            } else if (gamepad2.dpad_up) {
                liftRunningStep = 1;
            } else if (liftRunningStep == 1) {
                leftLift.setPower(leftLift.getCurrentPosition() >= 5800 ? 0 : 0.15);
                rightLift.setPower(rightLift.getCurrentPosition() >= 5800 ? 0 : 0.15);
            } else {
                leftLift.setPower(0.0);
                rightLift.setPower(0.0);
                if (gamepad2.b || gamepad2.circle) {
                    rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Run time", runtime);
            telemetry.addData("R. lift pos", rightLift.getCurrentPosition());
            telemetry.addData("L. lift pos", leftLift.getCurrentPosition());
            telemetry.update();
        }
    }
    // yeah it doesn't work I guess
    /*public void doTheLift(double speed, double distance) {
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            int rTarget = rightLift.getCurrentPosition() + (int) (distance * 5850);
            int lTarget = leftLift.getCurrentPosition() + (int) (distance * 5850);
            rightLift.setTargetPosition(rTarget);
            leftLift.setTargetPosition(lTarget);

            // Turn On RUN_TO_POSITION
            rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < 10) && (rightLift.isBusy() || leftLift.isBusy())) {
                telemetry.addData("Lifting", "(ignoring all other commands)");
                if(rightLift.getCurrentPosition() - leftLift.getCurrentPosition() > 100) {
                    rightLift.setPower(0);
                    leftLift.setPower(Math.abs(speed));
                } else if(leftLift.getCurrentPosition() - rightLift.getCurrentPosition() > 100) {
                    leftLift.setPower(0);
                    rightLift.setPower(Math.abs(speed));
                } else {
                    leftLift.setPower(Math.abs(speed));
                    rightLift.setPower(Math.abs(speed));
                }
                telemetry.update();
            }

            // Stop all motion
            rightLift.setPower(0);
            leftLift.setPower(0);

            liftRunningStep = 1; // From now on, keep running the lift motors.
        }
    }*/
}