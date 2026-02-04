// ===============================
// RobotHardware.java (modificat)
// ===============================
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.OuttakeSystem;
import org.firstinspires.ftc.teamcode.systems.TransferSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

public final class RobotHardwareTEST {
	public final PinpointDrive drivetrain;
	public TumblerSystem intakeStopper, turretTumbler;
	public DistanceSystem distanceSystem;

	public IntakeSystem intake, turret;
	public OuttakeSystem outtake1, outtake2;

	public TransferSystem transfer;
	public Limelight3A limelight;

	public RobotHardwareTEST(HardwareMap hardwareMap) {
		drivetrain = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

		intake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		turret = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "turret"));

		// IMPORTANT:
		// Dacă outtake2 se învârte invers față de outtake1 (mecanic oglindă), pune REVERSE la unul.
		// Începe cu ambele FORWARD. Dacă unul e invers -> schimbă doar pe acela.
		outtake1 = new OuttakeSystem(hardwareMap.get(DcMotorEx.class, "outtake1"), DcMotorSimple.Direction.REVERSE);
		outtake2 = new OuttakeSystem(hardwareMap.get(DcMotorEx.class, "outtake2"), DcMotorSimple.Direction.REVERSE);

		transfer = new TransferSystem(hardwareMap.get(CRServo.class, "transfer"));

		intakeStopper = new TumblerSystem(hardwareMap.get(Servo.class, "intakeStopper"),
				0.61f, 0.5f, 0.15f, 0.3f, 0.38f);
		turretTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "turretTumbler"),
				0.65f, 0.32f, 0.25f, 1.0f, 0.32f);

		limelight = hardwareMap.get(Limelight3A.class, "limelight");
	}

	public void init() {
		intakeStopper.init();
		intakeStopper.setDestination(TumblerSystem.TumblerDestination.IDLE);
		turretTumbler.init();

		turret.init();
		outtake1.init();
		outtake2.init();
		intake.init();
		transfer.init();

		limelight.start();
		limelight.setPollRateHz(50);
	}
}
