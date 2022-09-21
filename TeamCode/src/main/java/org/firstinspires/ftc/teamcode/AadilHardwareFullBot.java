/*       ________                   ________           ________
        /        \                 /        \         |  _____ \
       /   ____   \               /   ____   \        | |     | \
      /   /    \   \             /   /    \   \       | |      \ \
     /   /      \   \           /   /      \   \      | |       \ \
    /   /________\   \         /   /________\   \     | |        | |
   /   /__________\   \       /   /__________\   \    | |       / /
  /   /            \   \     /   /            \   \   | |      / /
 /   /              \   \   /   /              \   \  | |_____| /
/   /                \   \ /   /                \   \ |________/
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AadilHardwareFullBot {

    public DcMotorEx front_left;
    public DcMotorEx front_right;
    public DcMotorEx back_left;
    public DcMotorEx back_right;

    HardwareMap hwMap;

    public AadilHardwareFullBot(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap=ahwMap;

        // Define and Initialize Motors
        front_left  = hwMap.get(DcMotorEx.class, "front_left");
        front_right = hwMap.get(DcMotorEx.class, "front_right");
        back_left   = hwMap.get(DcMotorEx.class, "back_left");
        back_right  = hwMap.get(DcMotorEx.class, "back_right");

        front_left.setDirection(DcMotorEx.Direction.REVERSE);
        front_right.setDirection(DcMotorEx.Direction.FORWARD);
        back_left.setDirection(DcMotorEx.Direction.REVERSE);
        back_right .setDirection(DcMotorEx.Direction.FORWARD);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);


        front_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}

