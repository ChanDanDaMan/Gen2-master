package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class UpdateRobotPositionTele extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    Encoder leftEncoder, rightEncoder, middleEncoder;
    BNO055IMU imu;

    static final double TICKS_PER_REV = 537.6;
    static final double WHEEL_DIAMETER = 100/25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    OdometryGlobalCoordinateSystem positionUpdate;

    @Override
    public void runOpMode() {



        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fl"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "br"));
        middleEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// get snailed

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        positionUpdate = new OdometryGlobalCoordinateSystem(leftEncoder, rightEncoder, middleEncoder, TICKS_PER_INCH, 1000);
        Thread position = new Thread(positionUpdate);
        position.start();

        while (opModeIsActive()) {
            drive();
            telemetry.addData("angle (degree)", positionUpdate.returnOrientation());
            telemetry.addData("x position", positionUpdate.returnXCoordinate()/ TICKS_PER_INCH);
            telemetry.addData("y position", positionUpdate.returnYCoordinate()/ TICKS_PER_INCH);
            telemetry.update();
        }
        positionUpdate.stop();
    }
    public void drive() {

        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        fl.setPower(-v1);
        fr.setPower(-v2);
        bl.setPower(-v3);
        br.setPower(-v4);

    }
}
