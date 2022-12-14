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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AadilHardwareFullBot {

    public DcMotorEx right;
    public DcMotorEx left;
    public DcMotorEx armright;
    public DcMotorEx armleft;
    public Servo servo;
    //public DcMotorEx f_right;
   // public DcMotorEx f_left; //Rajiv added
   // public DcMotorEx b_right;
   // public DcMotorEx b_left;

    HardwareMap hwMap;

    public AadilHardwareFullBot(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap=ahwMap;

        // Define Each Individual Wheel Motor
        // Define and Initialize Motors
   /*     f_right  = hwMap.get(DcMotorEx.class, "f_right");
        f_left = hwMap.get(DcMotorEx.class, "f_left");
        b_right  = hwMap.get(DcMotorEx.class, "b_right");
        b_left = hwMap.get(DcMotorEx.class, "b_left");*/
         armleft = hwMap.get(DcMotorEx.class, "armleft");
         armright = hwMap.get(DcMotorEx.class, "armright");
         servo = hwMap.get(Servo.class, "servo");

     /*   f_left.setDirection(DcMotorEx.Direction.REVERSE);
        b_left.setDirection(DcMotorEx.Direction.REVERSE);
        f_right.setDirection(DcMotorEx.Direction.FORWARD);
        b_right.setDirection(DcMotorEx.Direction.FORWARD);*/
        // Define and Initialize Motors
        left  = hwMap.get(DcMotorEx.class, "left");
        right = hwMap.get(DcMotorEx.class, "right");
        armleft = hwMap.get(DcMotorEx.class, "armleft");
        armright = hwMap.get(DcMotorEx.class, "armright");
        servo = hwMap.get(Servo.class, "servo");

        left.setDirection(DcMotorEx.Direction.FORWARD);
        right.setDirection(DcMotorEx.Direction.REVERSE);
        armleft.setDirection(DcMotorEx.Direction.FORWARD);
        armright.setDirection(DcMotorEx.Direction.REVERSE);


        left.setPower(0);
        right.setPower(0);


        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
     //   armright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     //   armleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   armleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   armright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}

