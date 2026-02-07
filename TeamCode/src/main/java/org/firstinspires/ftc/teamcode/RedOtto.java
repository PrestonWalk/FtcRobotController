package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="the real actual RED CLOSE otto")
//@Disabled
public class RedOtto extends LinearOpMode {

    // When making changes to the Otto, copy everything into the blue file, then change this.
    private final boolean allianceIsRed = true;
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive;
    private DcMotorEx backLeftDrive;
    private DcMotorEx frontRightDrive;
    private DcMotorEx backRightDrive;
    private DcMotorEx launcher;
    static final double COUNTS_PER_MOTOR_REV = 537.6; // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        CRServo blocker = hardwareMap.get(CRServo.class, "blocker");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int autoProgress = 0;
        boolean flywheelRunning = true;
        double startedAt = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(flywheelRunning) {
                launcher.setPower(Math.min(1.0, Math.max(0.0, 1.0 - 0.01 * (launcher.getVelocity() - 900))));
                telemetry.addData("Launcher's Speed", launcher.getVelocity());
                telemetry.addData("Launcher's Power", launcher.getPower());
            }
            else launcher.setPower(-0.2);
            // Shoot!
            if(autoProgress == 0) {
                encoderDrive(0.5, -20, 0, 0, 3, flywheelRunning);
                autoProgress = 2;
            } else if(autoProgress == 2 && launcher.getVelocity() > 900) {
                // Step 1: Spin the intake and move the blocker
                autoProgress = 3;
                intake.setPower(1.0);
                blocker.setPower(1.0);
                startedAt = runtime.milliseconds();
            } else if(autoProgress == 3 && (runtime.milliseconds() - startedAt) > 4000) {
                // Step 2: Stop after a while
                intake.setPower(0.0);
                blocker.setPower(0.0);
                flywheelRunning = false;
                startedAt = runtime.milliseconds();
                encoderDrive(0.5, -15, 0, 0, 3, flywheelRunning);
                encoderDrive(0.5, 0,0, (allianceIsRed ? 1 : -1) * 10, 3, flywheelRunning);
                encoderDrive(0.5, 0, (allianceIsRed ? 1 : -1) * 15, 0, 3, flywheelRunning);
                blocker.setPower(-0.4);
                intake.setPower(0.7);
                encoderDrive(0.5, 30, 0, 0, 3, flywheelRunning);
                intake.setPower(0.0);
                blocker.setPower(0.0);
                flywheelRunning = true;
                encoderDrive(0.5, -30, 0, 0, 3, flywheelRunning);
                encoderDrive(0.5, 0, (allianceIsRed ? -1 : 1) * 15, 0, 3, flywheelRunning);
                encoderDrive(0.5, 0, 0, (allianceIsRed ? -1 : 1) * 10, 3, flywheelRunning);
                encoderDrive(0.5, 18, 0, 0, 3, flywheelRunning);
                autoProgress = 4;
            } else if(autoProgress == 4 && launcher.getVelocity() > 900) {
                autoProgress = 5;
                intake.setPower(1.0);
                blocker.setPower(1.0);
                startedAt = runtime.milliseconds();
            } else if(autoProgress == 5 && (runtime.milliseconds() - startedAt) > 4000) {
                autoProgress = 99999;
                intake.setPower(0.0);
                blocker.setPower(0.0);
                flywheelRunning = false;
                encoderDrive(0.5, -12, (allianceIsRed ? -1 : 1) * 12, 0, 3, flywheelRunning);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }
    }
    public void encoderDrive(double speed, double axial, double lateral, double yaw, double timeoutS, boolean fwRunning) {
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = frontLeftDrive.getCurrentPosition() + (int) ((axial + lateral + yaw) * COUNTS_PER_INCH);
            newBLTarget = backLeftDrive.getCurrentPosition() + (int) ((axial - lateral + yaw) * COUNTS_PER_INCH);
            newFRTarget = frontRightDrive.getCurrentPosition() + (int) ((axial - lateral - yaw) * COUNTS_PER_INCH);
            newBRTarget = backRightDrive.getCurrentPosition() + (int) ((axial + lateral - yaw) * COUNTS_PER_INCH);
            //double frontLeftPower  = axial + lateral + yaw;
            //            double frontRightPower = axial - lateral - yaw;
            //            double backLeftPower   = axial - lateral + yaw;
            //            double backRightPower  = axial + lateral - yaw;
            frontLeftDrive.setTargetPosition(newFLTarget);
            backLeftDrive.setTargetPosition(newBLTarget);
            frontRightDrive.setTargetPosition(newFRTarget);
            backRightDrive.setTargetPosition(newBRTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backRightDrive.isBusy() && backLeftDrive.isBusy())) {
                telemetry.addData("Driving", "axial=" + axial + ", lateral=" + lateral + ", yaw=" + yaw);
                if(fwRunning) {
                    launcher.setPower(Math.min(1.0, Math.max(0.0, 1.0 - 0.01 * (launcher.getVelocity() - 900))));
                    telemetry.addData("Launcher's Speed", launcher.getVelocity());
                    telemetry.addData("Launcher's Power", launcher.getPower());
                }
                else launcher.setPower(-0.2);
                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}