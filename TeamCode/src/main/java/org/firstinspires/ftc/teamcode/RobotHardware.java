package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.systems.DistanceSystem;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.TransferSystem;
import org.firstinspires.ftc.teamcode.systems.TumblerSystem;

public final class RobotHardware {
	public final PinpointDrive drivetrain;
	public TumblerSystem intakeStopper, turretTumbler;
	public DistanceSystem distanceSystem;
	public IntakeSystem intake, outtake1, outtake2, turret;
	public TransferSystem transfer;
	public Limelight3A limelight;

	public RobotHardware(HardwareMap hardwareMap) {
		drivetrain = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

		intake = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "intake"));
		outtake1 = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "outtake1"));
		outtake2 = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "outtake2"));
		turret = new IntakeSystem(hardwareMap.get(DcMotorEx.class, "turret"));
		transfer = new TransferSystem(hardwareMap.get(CRServo.class, "transfer"));

		intakeStopper = new TumblerSystem(hardwareMap.get(Servo.class, "intakeStopper"), 0.61f, 0.5f, 0.15f, 0.3f, 0.38f);
		turretTumbler = new TumblerSystem(hardwareMap.get(Servo.class, "turretTumbler"), 0.65f, 0.32f, 0.25f, 1.0f, 0.32f);

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
