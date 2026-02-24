package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.RunInParallel;
import static org.firstinspires.ftc.teamcode.Utilities.RunSequentially;
import static org.firstinspires.ftc.teamcode.Utilities.WaitFor;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.BaseOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.WAYPOINTS_BLUE_CLOSE_EXP;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.WAYPOINTS_RED_FAR;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.WAYPOINTS_TEST;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name = "TEST TRAIECTORII", group = "Auto")
public class Auto_TEST extends BaseOpMode {

	private RobotHardware robot;

	private static final int AUTON_TURRET_TICKS = 20;
	private static final double AUTON_SHOOTER_POS = 0.44;
	private static final double TURRET_HOLD_POWER = 0.1;
	private static final double SHOOT_SAFE_IN = 8.0;
	private static final double SHOOT_SAFE_IN_2 = 15.0;
	private static final double SHOOT_SAFE_IN_START = 2.5;

	private boolean shooterEnabled = false;
	private double shooterRpmCmd = 4500;
	private boolean autonDone = false;

	// ===== Shooter debug timing =====
	private long shooterOnMs = -1;
	private int shootBurstIndex = 0;

	@Override
	protected void OnInitialize() {
		robot = new RobotHardware(hardwareMap);
		robot.init();
		robot.limelight.pipelineSwitch(1);

		OuttakeSystem.TICKS_PER_REV = 28;

		DcMotorEx turretMotor = robot.turret.getMotor();
		turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		turretHoldCurrent(TURRET_HOLD_POWER);
	}

	@Override protected void jOnInitialize() {}

	private void turretHoldCurrent(double holdPower) {
		DcMotorEx turret = robot.turret.getMotor();
		turret.setTargetPosition(turret.getCurrentPosition());
		turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		turret.setPower(holdPower);
	}

	private void turretOpenLoopBrake() {
		DcMotorEx turret = robot.turret.getMotor();
		turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	private Action newAimTurretLL() {
		return new AimTurretWithLimelightAction(this, robot, 0.035, 0.07, 0.5, 0.30, 750, 1.0, TURRET_HOLD_POWER);
	}
	private Action newAimTurretLLNEW() {
		return new AimTurretWithLimelightAction(this, robot, 0.035, 0.07, 0.5, 0.30, 750, 1.0, TURRET_HOLD_POWER);
	}
	private Action newAimTurretLLSecond() { return newAimTurretLL(); }
	private Action newAimTurretLLFinal() { return newAimTurretLL(); }

	/** Wait până când avgRPM e aproape de target (ca în TeleOp), cu timeout. */
	private Action waitUntilShooterRpm(double targetRpm, double tolRpm, long timeoutMs) {
		return new Action() {
			boolean init = false;
			long start;

			@Override
			public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
				if (!init) {
					init = true;
					start = System.currentTimeMillis();
				}
				if (System.currentTimeMillis() - start >= timeoutMs) return false;

				double r1 = robot.outtake1.getRpm();
				double r2 = robot.outtake2.getRpm();
				double avg = (r1 + r2) / 2.0;

				// debug în packet, ca să vezi că așteaptă fix după rpm
				packet.put("WAIT target", targetRpm);
				packet.put("WAIT avg", avg);
				packet.put("WAIT err", targetRpm - avg);
				packet.put("WAIT tol", tolRpm);

				return avg < targetRpm - tolRpm;
			}
		};
	}

	private static class AimTurretWithLimelightAction implements Action {
		private final Auto_TEST op;
		private final RobotHardware robot;
		private final double kP, minPower, maxPower, lockDeg, dir, hold;
		private final long timeout;
		private boolean init = false;
		private long start;

		AimTurretWithLimelightAction(Auto_TEST op, RobotHardware robot,
									 double kP, double minPower, double maxPower,
									 double lockDeg, long timeout, double dir, double hold) {
			this.op = op; this.robot = robot; this.kP = kP;
			this.minPower = minPower; this.maxPower = maxPower;
			this.lockDeg = lockDeg; this.timeout = timeout;
			this.dir = dir; this.hold = hold;
		}

		@Override
		public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
			if (!init) {
				init = true;
				start = System.currentTimeMillis();
				op.turretOpenLoopBrake();
				robot.turret.getMotor().setPower(0);
			}

			packet.put("TurretAim active", true);
			packet.put("TurretAim timeMs", System.currentTimeMillis() - start);

			if (System.currentTimeMillis() - start >= timeout) {
				op.turretHoldCurrent(hold);
				packet.put("TurretAim done", "TIMEOUT");
				return false;
			}

			LLResult r = robot.limelight.getLatestResult();
			boolean valid = (r != null && r.isValid());
			packet.put("LL valid", valid);

			if (!valid) {
				robot.turret.getMotor().setPower(0);
				packet.put("TurretAim done", "NO_LL");
				return true;
			}

			double tx = r.getTx();
			packet.put("LL tx", tx);

			if (Math.abs(tx) <= lockDeg) {
				op.turretHoldCurrent(hold);
				packet.put("TurretAim done", "LOCK");
				return false;
			}

			double p = dir * kP * tx;
			p = Math.max(Math.min(p, maxPower), -maxPower);
			if (p > 0) p = Math.max(p, minPower);
			else p = Math.min(p, -minPower);

			packet.put("TurretPwr", p);

			robot.turret.getMotor().setPower(p);
			return true;
		}
	}

	@Override
	protected void OnRun() {

		robot.drivetrain.updatePoseEstimate();



		// ===== Always-running shooter controller + TELEMETRY =====
		Action shooterController = packet -> {
			if (autonDone) return false;

			// update pose info
			packet.put("PoseX", robot.drivetrain.pose.position.x);
			packet.put("PoseY", robot.drivetrain.pose.position.y);
			packet.put("PoseHdeg", Math.toDegrees(robot.drivetrain.pose.heading.log()));

			if (shooterEnabled) {
				robot.outtake1.setRpm(shooterRpmCmd);
				robot.outtake2.setRpm(shooterRpmCmd);
			}

			double r1 = robot.outtake1.getRpm();
			double r2 = robot.outtake2.getRpm();
			double avg = (r1 + r2) / 2.0;

			packet.put("Shooter Enabled", shooterEnabled);
			packet.put("Shooter TargetRPM", shooterRpmCmd);
			packet.put("OT1 RPM", r1);
			packet.put("OT2 RPM", r2);
			packet.put("AVG RPM", avg);
			packet.put("RPM Error", shooterRpmCmd - avg);
			packet.put("Shooter READY", avg >= shooterRpmCmd - 150);

			// power (ultimul power setat)
			packet.put("PWR OT1", robot.outtake1.raw().getPower());
			packet.put("PWR OT2", robot.outtake2.raw().getPower());

			// cât timp a trecut de la shooter_on
			if (shooterOnMs > 0) packet.put("Shooter ON ms", System.currentTimeMillis() - shooterOnMs);

			// limelight quick status
			LLResult ll = robot.limelight.getLatestResult();
			boolean llValid = (ll != null && ll.isValid());
			packet.put("LL valid", llValid);
			if (llValid) packet.put("LL tx", ll.getTx());

			// burst index
			packet.put("Burst idx", shootBurstIndex);

			return true;
		};

		Action shooter_on = packet -> {
			shooterRpmCmd = 4700;
			shooterEnabled = true;
			shooterOnMs = System.currentTimeMillis();

			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);
			turretHoldCurrent(TURRET_HOLD_POWER);

			packet.put("EVENT", "shooter_on");
			return false;
		};

		Action shooter_off = packet -> {
			shooterEnabled = false;
			robot.outtake1.stop();
			robot.outtake2.stop();
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);

			packet.put("EVENT", "shooter_off");
			return false;
		};

		Action shootArtifact = packet -> {
			shootBurstIndex++;
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD);
			robot.transfer.setPower(-1);
			packet.put("EVENT", "shootArtifact");
			return false;
		};

		Action stopShooting = packet -> {
			robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP);
			robot.transfer.setPower(0);
			packet.put("EVENT", "stopShooting");
			return false;
		};

		Action turretToTicks = robot.turret.goToTicksAction(
				-212, // target
				0.6,                // power (0..1)
				8,                  // toleranta ticks
				1200,               // timeout ms
				TURRET_HOLD_POWER   // hold power dupa ce ajunge
		);

		Action turretHomeReset = robot.turret.goToZeroAndResetAction(
				0.6,                // power
				8,                  // toleranta
				1500,               // timeout ms
				TURRET_HOLD_POWER   // hold power
		);

		//traiectorii

		Action StartToOne =
				robot.drivetrain.actionBuilder(WAYPOINTS_TEST.start)
						.strafeTo(new Vector2d(
								WAYPOINTS_TEST.one.position.x,
								WAYPOINTS_TEST.one.position.y
						))
						.build();

		Action OneToTwo =
				robot.drivetrain.actionBuilder(WAYPOINTS_TEST.one)
						.strafeTo(new Vector2d(
								WAYPOINTS_TEST.two.position.x,
								WAYPOINTS_TEST.two.position.y
						))
						.build();

		Action TwoToThree =
				robot.drivetrain.actionBuilder(WAYPOINTS_TEST.two)
						.strafeTo(new Vector2d(
								WAYPOINTS_TEST.three.position.x,
								WAYPOINTS_TEST.three.position.y
						))
						.build();

		Action ThreeToFour =
				robot.drivetrain.actionBuilder(WAYPOINTS_TEST.three)
						.strafeTo(new Vector2d(
								WAYPOINTS_TEST.four.position.x,
								WAYPOINTS_TEST.four.position.y
						))
						.build();

		Action FourToFive =
				robot.drivetrain.actionBuilder(WAYPOINTS_TEST.four)
						.strafeTo(new Vector2d(
								WAYPOINTS_TEST.five.position.x,
								WAYPOINTS_TEST.five.position.y
						))
						.build();

		Action FiveToOne =
				robot.drivetrain.actionBuilder(WAYPOINTS_TEST.five)
						.strafeTo(new Vector2d(
								WAYPOINTS_TEST.one.position.x,
								WAYPOINTS_TEST.one.position.y
						))
						.build();

		Action OneToStart =
				robot.drivetrain.actionBuilder(WAYPOINTS_TEST.one)
						.strafeTo(new Vector2d(
								WAYPOINTS_TEST.start.position.x,
								WAYPOINTS_TEST.start.position.y
						))
						.build();

		Action startIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.FORWARD); packet.put("EVENT", "startIntake"); return false; };
		Action stopIntake = packet -> { robot.intake.setIntakeDirection(IntakeSystem.IntakeDirection.STOP); packet.put("EVENT", "stopIntake"); return false; };

		Action setAutonShooterAngle = packet -> { robot.turretTumbler.setPosition(AUTON_SHOOTER_POS); packet.put("EVENT", "setShooterAngle"); return false; };

		Actions.runBlocking(
						RunSequentially(

								OneToTwo,
								WaitFor(0.5),
								TwoToThree,
								WaitFor(0.5),
								ThreeToFour,
								WaitFor(0.5),
								FourToFive,
								WaitFor(0.5),
								FiveToOne,
								WaitFor(0.5),
								OneToStart,
								WaitFor(0.5)


						)
		);

		autonDone = true;
	}
}