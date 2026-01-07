package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Turret encoder range test:
 * - Move turret with LEFT STICK X
 * - Shows current encoder ticks + recorded min/max ticks
 * - Press A to "set zero here" (reset encoder to 0)
 * - Press X to clear recorded min/max (starts from current)
 * - Press B to stop motor instantly
 */
@TeleOp(name = "TEST - Turret Encoder Range", group = "TEST")
public class TurretEncoderRangeTest extends LinearOpMode {

	@Override
	public void runOpMode() {
		RobotHardware robot = new RobotHardware(hardwareMap);

		DcMotorEx turret = robot.turret.getMotor();

		// Keep power control manual, but read encoder.
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		int minTicks = turret.getCurrentPosition();
		int maxTicks = minTicks;

		telemetry.addLine("Turret Encoder Range Test ready");
		telemetry.addLine("LEFT STICK X = move turret");
		telemetry.addLine("A = reset encoder to 0 at current position");
		telemetry.addLine("X = reset min/max tracking (to current)");
		telemetry.addLine("B = stop motor");
		telemetry.update();

		waitForStart();

		boolean prevA = false, prevX = false;

		while (opModeIsActive()) {
			// Manual move
			double stick = gamepad1.left_stick_x; // -1..+1
			double power = stick * 0.35;          // scale for safety (adjust if too slow)

			if (gamepad1.b) power = 0;

			turret.setPower(power);

			// Read encoder
			int ticks = turret.getCurrentPosition();

			// Track min/max observed
			if (ticks < minTicks) minTicks = ticks;
			if (ticks > maxTicks) maxTicks = ticks;

			// Edge-detected buttons
			boolean a = gamepad1.a;
			boolean x = gamepad1.x;

			// A: reset encoder to 0 HERE
			if (a && !prevA) {
				turret.setPower(0);
				turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

				ticks = turret.getCurrentPosition(); // should be 0
				minTicks = ticks;
				maxTicks = ticks;
			}

			// X: reset min/max tracking only (keep encoder as-is)
			if (x && !prevX) {
				minTicks = ticks;
				maxTicks = ticks;
			}

			prevA = a;
			prevX = x;

			telemetry.addData("Turret ticks", ticks);
			telemetry.addData("Min ticks (seen)", minTicks);
			telemetry.addData("Max ticks (seen)", maxTicks);
			telemetry.addData("Range (ticks)", (maxTicks - minTicks));
			telemetry.addData("Power", power);
			telemetry.update();
		}

		turret.setPower(0);
	}
}
