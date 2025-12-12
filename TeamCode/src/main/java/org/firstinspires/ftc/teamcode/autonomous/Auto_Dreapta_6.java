package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.waypointsRED;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "Autonom_Dreapta", group = "Auto")
public class Auto_Dreapta_6 extends BaseOpMode {
	private RobotHardware robot;

	// === TUNE THESE ===
	private static final int AUTON_TURRET_TICKS = 20;      // tune this
	private static final double AUTON_SHOOTER_POS = 0.16;  // tune this
	private static final double TURRET_HOLD_POWER = 0.12;  // tune 0.05–0.20

	@Override
	protected void OnInitialize() {
		robot = new RobotHardware(hardwareMap);
		robot.init();

		// Zero turret encoder at known starting angle
		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		// Important: resist motion when power=0
		turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		// Optional: start holding current position immediately (prevents any bump drift)
		turretHoldCurrent(TURRET_HOLD_POWER);
	}

	@Override
	protected void jOnInitialize() {}

	/** Actively holds turret at its current encoder position (prevents drift). */
	private void turretHoldCurrent(double holdPower) {
		if (robot == null || robot.turret == null) return;
		DcMotorEx turret = robot.turret.getMotor();
		int pos = turret.getCurrentPosition();
		turret.setTargetPosition(pos);
		turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		turret.setPower(holdPower);
	}

	/** Lets turret be driven open-loop (used during limelight aiming). */
	private void turretOpenLoopBrake() {
		DcMotorEx turret = robot.turret.getMotor();
		turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	private static class AimTurretWithLimelightAction implements Action {
		private final Auto_Dreapta_6 op;      // to access turretHoldCurrent()
		private final RobotHardware robot;

		private final double kP;
		private final double minPower;
		private final double maxPower;
		private final double lockThresholdDeg;
		private final long timeoutMs;
		private final double directionSign;
		private final double holdPower;

		private boolean initialized = false;
		private long startTimeMs = 0;

		AimTurretWithLimelightAction(
				Auto_Dreapta_6 op,
				RobotHardware robot,
				double kP,
				double minPower,
				double maxPower,
				double lockThresholdDeg,
				long timeoutMs,
				double directionSign,
				double holdPower
		) {
			this.op = op;
			this.robot = robot;
			this.kP = kP;
			this.minPower = minPower;
			this.maxPower = maxPower;
			this.lockThresholdDeg = lockThresholdDeg;
			this.timeoutMs = timeoutMs;
			this.directionSign = directionSign;
			this.holdPower = holdPower;
		}

		@Override
		public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
			if (!initialized) {
				initialized = true;
				startTimeMs = System.currentTimeMillis();

				// Open-loop for smooth aiming, but with BRAKE so it doesn't freewheel
				op.turretOpenLoopBrake();
				robot.turret.getMotor().setPower(0);
			}

			long now = System.currentTimeMillis();
			long elapsed = now - startTimeMs;

			DcMotorEx turretMotor = robot.turret.getMotor();

			// Timeout -> hold and finish
			if (elapsed >= timeoutMs) {
				turretMotor.setPower(0);
				op.turretHoldCurrent(holdPower);
				packet.put("LL Aim", "TIMEOUT->HOLD");
				return false;
			}

			if (robot.limelight == null) {
				turretMotor.setPower(0);
				op.turretHoldCurrent(holdPower);
				packet.put("LL Aim", "NO LIMELIGHT->HOLD");
				return false;
			}

			LLResult result = robot.limelight.getLatestResult();
			if (result == null || !result.isValid()) {
				// No target -> stop turning, but keep waiting until timeout (do NOT finish yet)
				turretMotor.setPower(0);
				packet.put("LL Aim", "NO TARGET");
				return true;
			}

			double tx = result.getTx(); // deg
			double absTx = Math.abs(tx);

			packet.put("LL tx", tx);
			packet.put("LL absTx", absTx);
			packet.put("LL timeLeftMs", (timeoutMs - elapsed));

			// Locked on -> hold and finish
			if (absTx <= lockThresholdDeg) {
				turretMotor.setPower(0);
				op.turretHoldCurrent(holdPower);
				packet.put("LL Aim", "ALIGNED->HOLD");
				return false;
			}

			// P-control
			double power = directionSign * (kP * tx);

			// enforce minimum power so it actually moves
			if (power > 0) power = Math.max(power, minPower);
			else           power = Math.min(power, -minPower);

			// clamp max power
			if (power >  maxPower) power =  maxPower;
			if (power < -maxPower) power = -maxPower;

			turretMotor.setPower(power);
			packet.put("Turret power", power);

			return true;
		}
	}

	@Override
	protected void OnRun() {

		robot.drivetrain.updatePoseEstimate();

		/// === PATH ACTIONS ===

		// START -> SHOOT_BACK
		Action goToShootBack = robot.drivetrain.actionBuilder(waypointsRED.START)
				.splineTo(
						new Vector2d(waypointsRED.SHOOT_BACK.position.x, waypointsRED.SHOOT_BACK.position.y),
						waypointsRED.SHOOT_BACK.heading.toDouble()
				)
				.build();

		// SHOOT_BACK -> PICKUP1
		Action goToPickup = robot.drivetrain.actionBuilder(waypointsRED.SHOOT_BACK)
				.splineTo(
						new Vector2d(waypointsRED.PICKUP1.position.x, waypointsRED.PICKUP1.position.y),
						waypointsRED.PICKUP1.heading.toDouble()
				)
				.build();

		Action goBackPickup = robot.drivetrain.actionBuilder(waypointsRED.FPICKUP1)
				.lineToY(waypointsRED.PICKUP1.position.y)
				.build();

		// Pickup movement
		Action intakeLine = robot.drivetrain.actionBuilder(waypointsRED.PICKUP1)
				.lineToY(waypointsRED.FPICKUP1.position.y)
				.build();

		// FPICKUP1 -> SHOOT_BACK
		Action goToShootBack2 = robot.drivetrain.actionBuilder(waypointsRED.FPICKUP1)
				.splineTo(
						new Vector2d(waypointsRED.SHOOT_BACK.position.x, waypointsRED.SHOOT_BACK.position.y),
						waypointsRED.SHOOT_BACK.heading.toDouble()
				)
				.build();

		// ending location
		Action finishLine = robot.drivetrain.actionBuilder((waypointsRED.SHOOT_BACK))
				.splineTo(
						new Vector2d(waypointsRED.FINISH.position.x, waypointsRED.FINISH.position.y),
						waypointsRED.FINISH.heading.toDouble()
				)
				.build();

		/// === SHOOTER AND INTAKE ACTIONS ===

		// LL aim action (tune)
		Action aimTurretLL = new AimTurretWithLimelightAction(
				this,
				robot,
				0.045,   // kP
				0.12,    // minPower
				0.40,    // maxPower
				0.45,     // lockThreshold degrees
				2500,    // timeout ms
				+1.0,    // directionSign
				TURRET_HOLD_POWER
		);

		// start shooter and open stopper
		Action shooter_on = packet -> {
			robot.outtake1.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.outtake2.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);

			// extra safety: ensure turret is holding before the vibration starts
			turretHoldCurrent(TURRET_HOLD_POWER);

			return false;
		};

		//start shooter and close stopper
		Action shooter_off = packet -> {
			robot.outtake1.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.outtake2.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
			return false;
		};

		// feed artifacts to shooter
		Action shootArtifact = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.transfer.setPower(-1);
			return false;
		};

		// stop feeding artifacts to shooter
		Action stopShooting = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.transfer.setPower(0);
			return false;
		};

		// turret preset angle
		Action moveTurretToAutonAngle = packet -> {
			DcMotorEx turretMotor = robot.turret.getMotor();
			turretMotor.setTargetPosition(AUTON_TURRET_TICKS);
			turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			turretMotor.setPower(0.3);
			return false;
		};

		// shooter platform servo angle
		Action setAutonShooterAngle = packet -> {
			robot.turretTumbler.setPosition(AUTON_SHOOTER_POS);
			return false;
		};

		// floor intake (for pickup path)
		Action startIntake = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			return false;
		};

		Action stopIntake = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			return false;
		};

		// === FULL AUTON SEQUENCE ===

		Actions.runBlocking(
				RunSequentially(
						goToShootBack,
						WaitFor(0.3),

						// Aim + tilt
						aimTurretLL,
						setAutonShooterAngle,
						WaitFor(0.3),

						shooter_on,
						WaitFor(1.0),

						// BALL 1
						shootArtifact,
						WaitFor(0.27),
						stopShooting,
						WaitFor(1.2),

						// BALL 2
						aimTurretLL,
						shootArtifact,
						WaitFor(0.30),
						stopShooting,
						WaitFor(1.2),

						// BALL 3
						aimTurretLL,
						setAutonShooterAngle,
						shootArtifact,
						WaitFor(0.55),
						stopShooting,
						WaitFor(0.2),

						stopShooting,
						shooter_off,

						// pickup
						goToPickup,
						WaitFor(0.2),

						startIntake,
						WaitFor(0.1),

						intakeLine,
						WaitFor(0.6),

						stopIntake,
						WaitFor(0.4),

						// return (you had goToPickup again; leaving as-is)
						goToPickup,
						WaitFor(0.3),
						goToShootBack2,
						WaitFor(0.8),

						// Aim + tilt
						aimTurretLL,
						setAutonShooterAngle,
						WaitFor(0.3),

						shooter_on,
						WaitFor(1.0),

						// BALL 1
						shootArtifact,
						WaitFor(0.27),
						stopShooting,
						WaitFor(1.3),

						// BALL 2
						aimTurretLL,
						setAutonShooterAngle,
						shootArtifact,
						WaitFor(0.33),
						stopShooting,
						WaitFor(1.2),

						// BALL 3
						aimTurretLL,
						setAutonShooterAngle,
						shootArtifact,
						WaitFor(0.5),
						stopShooting,
						WaitFor(0.4),

						stopShooting,
						shooter_off,

						finishLine
				)
		);
	}
}
