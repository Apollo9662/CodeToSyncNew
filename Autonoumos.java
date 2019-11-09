package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "Autonoumos ")
public class Autonoumos extends functions{
    @Override
    public void runOpMode()  {

        robot.init(hardwareMap);
        telemetry.addData("state ==> ","finished");
        telemetry.update();
        waitForStart();
        robot.setCollectMotorsPower(-1);
        while (opModeIsActive()){
            gyroDrive(0.7,23,0);
            sleep(300);
            gyroDrive(-0.7,8,0);
            sleep(300);
            gyroTurn(0.3,-90);
            gyroDrive(0.7,45,-90);
            robot.setCollectMotorsPower(1);
            sleep(300);
            gyroDrive(-0.7,10,-90);
            break;


        }
    }
}
