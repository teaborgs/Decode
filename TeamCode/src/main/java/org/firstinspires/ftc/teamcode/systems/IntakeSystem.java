package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSystem extends AbstractSystem
{
	private final DcMotorEx intakeMotor;

	public enum IntakeDirection
	{
		FORWARD,
		FORWARDCLOSE,
		SLOW_FORWARD,
		SLOW_REVERSE,
		REVERSE,
		STOP
	}

	public IntakeSystem(DcMotorEx intakeMotor)
	{
		this.intakeMotor = intakeMotor;
		this.intakeMotor.setDirection(DcMotor.Direction.REVERSE);
		this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		stop(); // Ensure it's stopped on init
	}

	@Override
	public void init()
	{
		stop();
	}

	private void stop()
	{
		intakeMotor.setPower(0f);
	}

	public void setIntakeDirection(IntakeDirection direction)
	{
		switch (direction)
		{
			case FORWARDCLOSE:
				intakeMotor.setPower(0.8f);
				break;
			case FORWARD:
				intakeMotor.setPower(1f);
				break;
			case REVERSE:
				intakeMotor.setPower(-1f);
				break;
			case SLOW_FORWARD:
				intakeMotor.setPower(0.25f);
				break;
			case SLOW_REVERSE:
				intakeMotor.setPower(-0.25f);
				break;
			case STOP:
			default:
				intakeMotor.setPower(0f);
				break;
		}
	}
	public Action setDirectionAction(IntakeDirection direction) {
		return new InstantAction(() -> setIntakeDirection(direction));
	}
	public DcMotorEx getMotor() {
		return intakeMotor;
	}
}




