package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "KILLME")
public class Iwanttodie extends OpMode{
    AadilHardwareFullBot robot = new AadilHardwareFullBot();

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
    }
    @Override
    public void loop() {
        telemetry.addData("Armleft", robot.armleft.getCurrentPosition());
        telemetry.addData("armright", robot.armright.getCurrentPosition());
        telemetry.update();
        robot.left.setPower(1);
        robot.right.setPower(1);
    }
}
