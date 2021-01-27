package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "TestOpOnly")
public class TestOp extends LinearOpMode {


    public DcMotor fl, fr, bl, br;
    public DcMotor shooter1, shooter2;
    public Servo claw, kicker;

    @Override
    public void runOpMode()  {



        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        kicker = hardwareMap.get(Servo.class, "kicker");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerDrivetrain(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            drive();

            if (gamepad1.b) {
                shooterPower(1);
            }else {
                shooterPower(0);
            }

        }
    }

    public void drive() {

        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        if(gamepad1.left_stick_button) {
            fl.setPower(v1 * .7);
            fr.setPower(v2 * .7);
            bl.setPower(v3 * .7);
            br.setPower(v4 * .7);
        }else {
            fl.setPower(v1 / 2);
            fr.setPower(v2 / 2);
            bl.setPower(v3 / 2);
            br.setPower(v4 / 2);
        }
    }


    public void shooterPower(double pwr){
        shooter1.setPower(pwr);
        shooter2.setPower(pwr);
    }

    public void setZeroPowerDrivetrain (DcMotor.ZeroPowerBehavior behavior){
        fl.setZeroPowerBehavior(behavior);
        fr.setZeroPowerBehavior(behavior);
        bl.setZeroPowerBehavior(behavior);
        br.setZeroPowerBehavior(behavior);
    }
}
