package org.firstinspires.ftc.teamcode.command;
import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
public abstract class CommandOpMode extends LinearOpMode {
    protected Scheduler scheduler;
    ElapsedTime clock = new ElapsedTime();
    double last = 0;
    public abstract void initOpMode();
    public void waitOpMode() {}
    public void startOpMode() {}
    @Override
    public void runOpMode() {
        scheduler = new Scheduler();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(AUTO);
        }
        initOpMode();
        while (!isStarted() && !isStopRequested()) {
            waitOpMode();
            scheduler.run(false);
        }
        startOpMode();
        while (opModeIsActive()) {
            scheduler.run(true);
            telemetry.addData("Loop speed", 1 / (-last + (last = clock.seconds())));
            telemetry.addData("Commands", scheduler.getCommands());
            telemetry.update();
        }
    }
    public void register(Subsystem... subsystems) {
        scheduler.register(subsystems);
    }
}
