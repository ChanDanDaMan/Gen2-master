package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "AutoBot2")
public class UpdateRobotPositionAuto extends LinearRobot {



    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();

        waitForStart();
        while (opModeIsActive()){
            fl.setPower(.5);
        }

    }
}
