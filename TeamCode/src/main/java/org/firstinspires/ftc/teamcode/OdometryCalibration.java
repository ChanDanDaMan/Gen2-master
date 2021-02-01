package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp
public class OdometryCalibration extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    Encoder leftEncoder, rightEncoder, middleEncoder;

    BNO055IMU imu;

    ElapsedTime timer = new ElapsedTime();

    //sets power of motor
    static final double calibrationSpeed = .5;

    //going to change after known type of drivetrain motor in detail *ask Jack or Justin*

    static final double TICKS_PER_REV = 537.6;

    static final double WHEEL_DIAMETER = 100/25.4;

    //Next, is the gear ratio needs to also be changed once drivetrain motors' info is found

    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    File sidewheelSeparationFile = AppUtil.getInstance().getSettingsFile("sidewheelsSeperationFile");
    File middleTickOffsetFile  = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");

    @Override
    public void runOpMode(){
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

        telemetry.addData("Status:", "Ready!");
        telemetry.update();

        waitForStart();

        while (getZAngle() < 90 && opModeIsActive()){
            fr.setPower(-calibrationSpeed);
            fl.setPower(calibrationSpeed);
            bl.setPower(calibrationSpeed);
            br.setPower(-calibrationSpeed);
            if(getZAngle() < 60){
                fr.setPower(-calibrationSpeed);
                fl.setPower(calibrationSpeed);
                bl.setPower(calibrationSpeed);
                br.setPower(-calibrationSpeed);
            }else {
                fr.setPower(-calibrationSpeed / 2);
                fl.setPower(calibrationSpeed / 2);
                bl.setPower(calibrationSpeed / 2);
                br.setPower(-calibrationSpeed / 2);
            }
        }
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        timer.reset();
        while(timer.seconds() < 1 && opModeIsActive()){
            telemetry.addLine("waiting....");
        }

        double angle = getZAngle();
        double encoderDifference = Math.abs(Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition()) - Math.abs(middleEncoder.getCurrentPosition()));
        double sideEncoderTicksOffset = encoderDifference / angle;
        double sideWheelSeparation = (180 * sideEncoderTicksOffset ) / (TICKS_PER_INCH * Math.PI);
        double middleTickOffset = middleEncoder.getCurrentPosition() / Math.toRadians(imu.getAngularOrientation().thirdAngle);

        ReadWriteFile.writeFile(sidewheelSeparationFile, String.valueOf(sideWheelSeparation));
        ReadWriteFile.writeFile(middleTickOffsetFile, String.valueOf(middleTickOffset));
    }

    private double getZAngle(){
        return (imu.getAngularOrientation().firstAngle);
    }
}
