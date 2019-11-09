package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Autonomous(name = "Pure Pursuit")
public class Auto extends LinearOpMode {
    RobotMovement robotMovement = new RobotMovement();
    ArrayList<CurvePoint> pathCurrent = new ArrayList();
    ArrayList<CurvePoint> path = new ArrayList();
    public void runOpMode(){
        robotMovement.robot.init(hardwareMap);
        path.add(new CurvePoint(1000,500,1.0,0.5,50,Math.toRadians(30),1.0));
        path.add(new CurvePoint(1000,0,1.0,0.5,50,Math.toRadians(30),1.0));
        path.add(new CurvePoint(200,400,1.0,0.5,50,Math.toRadians(30),1.0));

        telemetry.addData("finish","wating to start");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("opMose","is active");
            telemetry.addData("bot point", new Point(robotMovement.botX(), robotMovement.botY()));
            telemetry.addData("pathCurrent.size",pathCurrent.size());

            telemetry.addData("error",robotMovement.error);
            telemetry.addData("steer",robotMovement.steer);
            telemetry.addData("angele", robotMovement.disaierd_angle);
            telemetry.addData("pathCurrent.size()",pathCurrent.size());
            if(pathCurrent.size() > 0) {
                pathCurrent = robotMovement.followCurve(pathCurrent,1);
                telemetry.addData("followMe:", "X => " + robotMovement.followMe.x);

            }

            else{
                robotMovement.drive(0, 0);
                if(path.size() > 0){
                    pathCurrent.add(path.get(0));
                    path.remove(path.get(0));
                }
            }
           // telemetry.addData("followMe:", "X => " + robotMovement.followMe.x + " Y => " + robotMovement.followMe.y);

            telemetry.update();

        }
    }
}
