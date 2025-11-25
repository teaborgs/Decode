package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Mers Inainte 2 Sec", group="Test")

public class autonomX extends LinearOpMode
{
	DcMotor rightFront, leftFront, rightBack, leftBack;
	@Override public void runOpMode()
			throws InterruptedException
	{
		rightFront = hardwareMap.get(DcMotor.class, "rightFront");
		leftFront = hardwareMap.get(DcMotor.class, "leftFront");
		rightBack = hardwareMap.get(DcMotor.class, "rightBack");
		leftBack = hardwareMap.get(DcMotor.class, "leftBack");
		 leftFront.setDirection(DcMotor.Direction.REVERSE);
		 leftBack.setDirection(DcMotor.Direction.REVERSE);
		 telemetry.addLine("Gata de start");
		 telemetry.update(); waitForStart();
		 if (opModeIsActive())
		 { mergeInainteFaraEncodere(0.5, 1500);}
	}
	public void mergeInainteFaraEncodere(double viteza, long milisecunde)
	{
		rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		rightFront.setPower(viteza); rightBack.setPower(viteza);
		leftBack.setPower(viteza); leftFront.setPower(viteza);
		sleep(milisecunde);
		rightFront.setPower(0);
		rightBack.setPower(0);
		leftBack.setPower(0);
		leftFront.setPower(0);}
}