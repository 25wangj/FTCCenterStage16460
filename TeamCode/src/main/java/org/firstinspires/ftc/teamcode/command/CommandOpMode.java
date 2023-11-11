package org.firstinspires.ftc.teamcode.command;
import static com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode.AUTO;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
public abstract class CommandOpMode extends LinearOpMode {
    protected Scheduler scheduler;
    private boolean done = false;
    private ElapsedTime clock = new ElapsedTime();
    private double last = 0;
    public abstract void initOpMode();
    public void waitOpMode() {}
    public void startOpMode() {}
    public void endOpMode() {}
    @Override
    public void runOpMode() {
        scheduler = new Scheduler();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initOpMode();
        while (!isStarted() && !isStopRequested()) {
            waitOpMode();
            scheduler.run(false);
            telemetry.update();
        }
        startOpMode();
        while (opModeIsActive() && !done) {
            scheduler.run(true);
            telemetry.addData("Loop speed", 1 / (-last + (last = clock.seconds())));
            telemetry.update();
        }
        endOpMode();
    }
    public void register(Subsystem... subsystems) {
        scheduler.register(subsystems);
    }
    public void end() {
        done = true;
    }
}
