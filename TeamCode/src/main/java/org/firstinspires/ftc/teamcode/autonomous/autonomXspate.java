package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem.IntakeDirection;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

@Autonomous(name="Shoot + Mers Inainte 2 Sec (spate)", group="Auto")
public class autonomXspate extends LinearOpMode
{


	DcMotor rightFront, leftFront, rightBack, leftBack;


	DcMotor outtake1, intake, outtake2, turret;
	Servo intakeStopper;
	CRServo transfer;


	RobotHardware robot;
	Limelight3A limelight;

	@Override
	public void runOpMode() throws InterruptedException
	{

		// === INIT RobotHardware (CA ÎN TELEOP) ===
		robot = new RobotHardware(hardwareMap);
		robot.init();

		// Folosim exact aceeași Limelight ca în teleop
		limelight = robot.limelight;


		rightFront = hardwareMap.get(DcMotor.class, "rightFront");
		leftFront = hardwareMap.get(DcMotor.class, "leftFront");
		rightBack = hardwareMap.get(DcMotor.class, "rightBack");
		leftBack = hardwareMap.get(DcMotor.class, "leftBack");

		outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
		outtake2 = hardwareMap.get(DcMotor.class, "outtake2");
		transfer = hardwareMap.get(CRServo.class, "transfer");
		intake = hardwareMap.get(DcMotor.class, "intake");
		turret = hardwareMap.get(DcMotor.class, "turret");
		intakeStopper = hardwareMap.get(Servo.class, "intakeStopper");


		outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		leftFront.setDirection(DcMotor.Direction.REVERSE);
		leftBack.setDirection(DcMotor.Direction.REVERSE);

		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


		telemetry.setMsTransmissionInterval(11);

		telemetry.addLine("Gata de start");
		telemetry.update();
		waitForStart();

		if (opModeIsActive())
		{

			mersinainteShoot(0.3, 2500);

			autoAimTurretLimelight(0.4, 2500);

			shootSecventa(3000);

			IntoarceDreapta(0.3, -0.3, 2000);

			//mersinainte(0.3, 2000);


			robot.turretTumbler.setDestination(TumblerSystem.TumblerDestination.HOVER);
		}
	}


	public void autoAimTurretLimelight(double maxPower, long timeoutMs)
	{
		if (limelight == null)
		{
			telemetry.addLine("Limelight NU e mapat!");
			telemetry.update();
			return;
		}

		double kP = 0.03;           // gain pentru tx
		double minPower = 0.12;     // putere minimă ca să miște tureta
		double lockThreshold = 1.2; // considerăm ALIGNED

		long startTime = System.currentTimeMillis();

		turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		while (opModeIsActive()
				&& (System.currentTimeMillis() - startTime) < timeoutMs
				&& !isStopRequested())
		{

			LLResult result = limelight.getLatestResult();

			if (result == null)
			{
				turret.setPower(0);
				telemetry.addLine("LL: result == null");
				telemetry.update();
				continue;
			}

			if (!result.isValid())
			{
				turret.setPower(0);
				telemetry.addLine("LL: no valid target");
				telemetry.update();
				continue;
			}

			double tx = result.getTx();
			double absTx = Math.abs(tx);

			telemetry.addData("LL tx", tx);
			telemetry.addData("LL absTx", absTx);

			// LOCK: dacă suntem suficient de aproape de centru
			if (absTx < lockThreshold)
			{
				turret.setPower(0);
				telemetry.addLine("Turret ALIGNED");
				telemetry.update();
				break;
			}

			double power = kP * tx;


			if (power > 0) power = Math.max(power, minPower);
			else power = Math.min(power, -minPower);

			// clamp la maxPower
			if (power > maxPower) power = maxPower;
			if (power < -maxPower) power = -maxPower;

			turret.setPower(power);

			telemetry.addData("Turret power", power);
			telemetry.update();
		}

		turret.setPower(0);
	}


	public void mersinainteShoot(double viteza, long milisecunde)
	{
		rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		rightFront.setPower(viteza);
		rightBack.setPower(viteza);
		leftBack.setPower(viteza);
		leftFront.setPower(viteza);

		sleep(milisecunde);

		rightFront.setPower(0);
		rightBack.setPower(0);
		leftBack.setPower(0);
		leftFront.setPower(0);

	}

	public void IntoarceDreapta(double viteza1, double viteza2, long milisecunde)
	{
		leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		// Stânga înainte, dreapta înapoi (față de robot)
		leftFront.setPower(viteza1);
		leftBack.setPower(viteza2);

		rightFront.setPower(-viteza1);
		rightBack.setPower(-viteza2);

		sleep(milisecunde);

		leftFront.setPower(0);
		leftBack.setPower(0);
		rightFront.setPower(0);
		rightBack.setPower(0);
	}


	public void shootSecventa(long milisecunde) throws InterruptedException
	{

		intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		intake.setDirection(DcMotor.Direction.FORWARD);

		// ridici stopperul
		intakeStopper.setPosition(0.6);

		// pornesti outtake-urile
		outtake1.setPower(0.8);
		outtake2.setPower(0.8);


		sleep(1600);

		for (int i = 0; i < 3; i++)
		{


			transfer.setPower(-0.6);
			sleep(500);


			intake.setPower(-0.9);
			sleep(300);

			//transfer.setPower(0);
			//sleep(450);
		}

		// opresti tot
		transfer.setPower(0.0);
		outtake1.setPower(0.0);
		outtake2.setPower(0.0);
		intake.setPower(0.0);
	}

	public void mersinainte(double viteza, long milisecunde)
	{
		rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		rightFront.setPower(viteza);
		rightBack.setPower(viteza);
		leftBack.setPower(viteza);
		leftFront.setPower(viteza);

		sleep(milisecunde);

		rightFront.setPower(0);
		rightBack.setPower(0);
		leftBack.setPower(0);
		leftFront.setPower(0);

	}
}