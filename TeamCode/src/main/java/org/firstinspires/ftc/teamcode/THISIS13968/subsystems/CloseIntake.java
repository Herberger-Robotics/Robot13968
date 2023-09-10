package org.firstinspires.ftc.teamcode.THISIS13968.subsystems;///package org.firstinspires.ftc.teamcode.THISIS10111.subsystems;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.THISIS10111.subsystems.ArmObj;
//
//import java.util.concurrent.TimeUnit;
//
//public class CloseIntake extends CommandBase {
//
//    private ElapsedTime timer = new ElapsedTime();
//    ArmObj arm;
//
//    public CloseIntake(ArmObj armObj) {
//        this.arm = armObj;
//    }
//
//    @Override
//    public void initialize() {
//        arm.closeIntake();
//        timer.reset();
//    }
////
//    @Override
//    public boolean isFinished() {
//        return timer.time(TimeUnit.MILLISECONDS) >= 500;
//    }
//
//}
