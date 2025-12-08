package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name="TEST AUTO", group="Auto")
public class autonomXXX extends LinearOpMode {

	private RobotHardware robot;

	private static final double TURRET_LL_DEADZONE = 2.0;  // cât de centrat vrei (în grade)

	@Override
	public void runOpMode() throws InterruptedException {

		// Inițializezi tot robotul dintr-un singur loc
		robot = new RobotHardware(hardwareMap);
		robot.init();

		telemetry.addLine("Gata de start");
		telemetry.update();

		waitForStart();

		if (!opModeIsActive()) return;

		// (opțional) mic delay ca să pornească bine Limelight-ul
		sleep(200);

		// 1) auto-aim pe turret cu Limelight
		autoAimTurret(2000); // încearcă max 2s să se alinieze

		// 2) poți seta unghiul de shooter dacă vrei
		//    (acum doar îl pun pe BUSY după mers, cum aveai și tu)
		// robot.turretTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);

		// 3) secvența de shoot (3 mingi)
		shootSecventa();

		// 4) mers înainte 1.5 secunde
		mergeInainteFaraEncodere(0.3, 1500);

		// 5) la final, pune tumbler-ul unde vrei tu (cum aveai în cod)
		robot.turretTumbler.setDestination(TumblerSystem.TumblerDestination.BUSY);
	}

	// === AUTO AIM TURRRET ÎN AUTONOM ===
	private void autoAimTurret(long timeoutMs) {
		long start = System.currentTimeMillis();

		while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {

			if (robot.limelight == null) {
				telemetry.addLine("Limelight NULL in auto");
				telemetry.update();
				break;
			}

			LLResult result = robot.limelight.getLatestResult();

			if (result == null || !result.isValid()) {
				// nu avem target -> nu mișca tureta
				robot.turret.setIntakeDirection(IntakeDirection.STOP);
				telemetry.addLine("NO TARGET");
				telemetry.update();
				sleep(10);
				continue;
			}

			double tx = result.getTx();
			double absTx = Math.abs(tx);

			telemetry.addData("LL tx", tx);
			telemetry.update();

			// destul de centrat -> stop și ieșim
			if (absTx < TURRET_LL_DEADZONE) {
				robot.turret.setIntakeDirection(IntakeDirection.STOP);
				return;
			}

			boolean fast = absTx > 10; // dacă e foarte departe, mișcă mai repede

			if (tx > 0) {
				// targetul e în dreapta imaginii
				robot.turret.setIntakeDirection(
						fast ? IntakeDirection.FORWARD : IntakeDirection.SLOW_FORWARD
				);
			} else {
				// targetul e în stânga imaginii
				robot.turret.setIntakeDirection(
						fast ? IntakeDirection.REVERSE : IntakeDirection.SLOW_REVERSE
				);
			}

			sleep(10); // lasă tureta să se miște un pic
		}

		// la final, oprește oricum tureta
		robot.turret.setIntakeDirection(IntakeDirection.STOP);
	}

	// === SHOOT SECVENȚA FOLOSIND SUBSYSTEM-URILE (3 mingi) ===
	private void shootSecventa() throws InterruptedException {
		// pornești shooter-ele
		robot.outtake1.setIntakeDirection(IntakeDirection.FORWARD);
		robot.outtake2.setIntakeDirection(IntakeDirection.FORWARD);

		// timp de spin-up
		sleep(1500);

		// feed-ezi 3 mingi, cu transfer + intake + stopper
		for (int i = 0; i < 3 && opModeIsActive(); i++) {
			// deschizi stopper-ul spre shooter
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.TRANSFER);

			// pornești intake + transfer
			robot.transfer.start();
			// atenție: dacă dă invers, schimbă FORWARD/REVERSE aici
			robot.intake.setIntakeDirection(IntakeDirection.FORWARD);

			sleep(600); // timp să ajungă mingea

			// oprești feed-ul
			robot.transfer.stop();
			robot.intake.setIntakeDirection(IntakeDirection.STOP);
			robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);

			sleep(500); // mică pauză între mingi
		}

		// oprești tot la final
		robot.outtake1.setIntakeDirection(IntakeDirection.STOP);
		robot.outtake2.setIntakeDirection(IntakeDirection.STOP);
		robot.transfer.stop();
		robot.intake.setIntakeDirection(IntakeDirection.STOP);
		robot.intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
	}

	// === MERS ÎNAINTE TIMP X, CU DRIVETRAIN-UL DIN MecanumDrive ===
	private void mergeInainteFaraEncodere(double viteza, long milisecunde) {
		long endTime = System.currentTimeMillis() + milisecunde;

		while (opModeIsActive() && System.currentTimeMillis() < endTime) {
			// atenție la semn: dacă merge înapoi, schimbă -viteza în +viteza
			robot.drivetrain.setDrivePowers(
					new PoseVelocity2d(
							new Vector2d(+viteza, 0), // Y = înainte/înapoi
							0.0                        // fără rotație
					)
			);
		}

		// oprești robotul
		robot.drivetrain.setDrivePowers(
				new PoseVelocity2d(
						new Vector2d(0, 0),
						0.0
				)
		);
	}
}
