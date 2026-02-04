// ===============================
// TeleOpOuttakeRPM.java (MODIFICAT: nimic la nume/controale, doar mesajele de tuning + limitări mai safe)
// ===============================
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardwareTEST;
import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;

@TeleOp(name = "TeleOp - Outtake RPM (separat)", group = "TeleOp")
public class TeleOpOuttakeRPM extends OpMode {

	private Gamepad input1, input2;
	private Gamepad driveInput, armInput;

	private RobotHardwareTEST robot;

	// ===== RPM =====
	private double targetRpm = 4500;
	private final double rpmStep = 100;

	private final double presetLow = 4200;   // X
	private final double presetHigh = 5200;  // B

	// ===== toggle edges =====
	private boolean outtakeToggleOn = false;
	private boolean lastY = false;
	private boolean lastUp = false;
	private boolean lastDown = false;
	private boolean lastX = false;
	private boolean lastB = false;
	private boolean lastA = false;

	@Override
	public void init() {
		input1 = gamepad1;
		input2 = gamepad2;

		jOnInitialize();

		robot = new RobotHardwareTEST(hardwareMap);
		robot.init();

		OuttakeSystem.TICKS_PER_REV = 28;

		telemetry.addLine("TeleOpOuttakeRPM ready");
		telemetry.addLine("G2: RT hold / Y toggle, X=4200, B=5200, Dpad +/- , A=STOP");
		telemetry.addLine("Anti-overshoot: ramp + PIDF mai bland");
		telemetry.update();
	}

	// preferința ta
	public void jOnInitialize() {
		driveInput = input1;
		armInput = input2;
	}

	@Override
	public void loop() {
		// ---------------- DRIVE (încearcă RR-style) ----------------
		double y = -driveInput.left_stick_y;
		double x = -driveInput.left_stick_x;
		double rx = -driveInput.right_stick_x;

		if (Math.abs(x) < 0.05) x = 0;
		if (Math.abs(y) < 0.05) y = 0;
		if (Math.abs(rx) < 0.05) rx = 0;

		try {
			robot.drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(y, x), rx));
		} catch (Exception ignored) {}

		// ---------------- OUTTAKE CONTROL ----------------
		// Y toggle
		boolean yBtn = armInput.y;
		if (yBtn && !lastY) outtakeToggleOn = !outtakeToggleOn;
		lastY = yBtn;

		// Hold RT
		boolean holdSpin = armInput.right_trigger > 0.2;

		// A = stop imediat + oprește toggle
		boolean aBtn = armInput.a;
		if (aBtn && !lastA) {
			outtakeToggleOn = false;
			robot.outtake1.stop();
			robot.outtake2.stop();
		}
		lastA = aBtn;

		// Dpad +/- rpm
		boolean up = armInput.dpad_up;
		if (up && !lastUp) targetRpm += rpmStep;
		lastUp = up;

		boolean down = armInput.dpad_down;
		if (down && !lastDown) targetRpm -= rpmStep;
		lastDown = down;

		// Presets
		boolean xBtn = armInput.x;
		if (xBtn && !lastX) targetRpm = presetLow;
		lastX = xBtn;

		boolean bBtn = armInput.b;
		if (bBtn && !lastB) targetRpm = presetHigh;
		lastB = bBtn;

		// clamp (safe)
		if (targetRpm < 0) targetRpm = 0;
		if (targetRpm > 6500) targetRpm = 6500;

		boolean shouldSpin = (holdSpin || outtakeToggleOn) && targetRpm > 0 && !aBtn;

		if (shouldSpin) {
			robot.outtake1.setRpm(targetRpm);
			robot.outtake2.setRpm(targetRpm);
		} else if (!aBtn) {
			robot.outtake1.stop();
			robot.outtake2.stop();
		}

		// ---------------- TELEMETRY ----------------
		double rpm1 = robot.outtake1.getRpm();
		double rpm2 = robot.outtake2.getRpm();
		double avg = (rpm1 + rpm2) / 2.0;

		telemetry.addData("Outtake", shouldSpin ? (holdSpin ? "HOLD" : "TOGGLE") : "OFF");
		telemetry.addData("TargetRPM", (int) targetRpm);
		telemetry.addData("OT1 RPM", (int) rpm1);
		telemetry.addData("OT2 RPM", (int) rpm2);
		telemetry.addData("AVG RPM", (int) avg);

		telemetry.addData("kV", OuttakeSystem.kV);
		telemetry.addData("kP", OuttakeSystem.kP);
		telemetry.addData("kI", OuttakeSystem.kI);
		telemetry.addData("kD", OuttakeSystem.kD);
		telemetry.addData("Slew", OuttakeSystem.RPM_SLEW);


		telemetry.addData("Slew", "%.0f rpm/s", OuttakeSystem.RPM_SLEW);

		telemetry.addLine("Daca inca trece peste target: scade F (9 -> 8.5 -> 8) sau scade Slew (2500 -> 2000).");
		telemetry.addLine("Daca e prea lent: creste Slew (2500 -> 3500). Daca nu ajunge: creste F putin.");

		telemetry.update();
	}

	@Override
	public void stop() {
		if (robot != null) {
			robot.outtake1.stop();
			robot.outtake2.stop();
			try {
				robot.drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
			} catch (Exception ignored) {}
		}
	}
}
