package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
    @TeleOp(name="TellyOp 0.0.1")
    @Disabled
    public class TellyOp extends LinearOpMode {

        // Declare OpMode members for each of the 4 motors.
        private final ElapsedTime runtime = new ElapsedTime();

        @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
            DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
            DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
            DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
            DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");

            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            launcher.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the game to start (driver presses START)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            double launcherPower = 0.0;
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

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

                // Send calculated power to wheels
                frontLeftDrive.setPower(frontLeftPower);
                frontRightDrive.setPower(frontRightPower);
                backLeftDrive.setPower(backLeftPower);
                backRightDrive.setPower(backRightPower);

                // Toggle the launcher
                if (gamepad1.a && launcherPower < 0.5) {
                    launcherPower += 0.05;
                } else if(gamepad1.b && launcherPower >= 0.05) {
                    launcherPower -= 0.05;

                }
                launcher.setPower(launcherPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
                telemetry.addData("Back left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
                telemetry.update();
            }
        }
    }