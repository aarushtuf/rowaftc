package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "motor1")
public class RobotConfig extends OpMode {

    DcMotor dcMotor;
    /**
     *
     */
    @Override
    public void init() {
        //dcMotor = hardwareMap.dcMotor.get("motor1");
        dcMotor = hardwareMap.get(DcMotor.class,"motor1");
        
    }

    /**
     *
     */
    @Override
    public void loop() {
        dcMotor.setPower(1);
    }
}
