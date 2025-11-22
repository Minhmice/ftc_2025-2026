package org.firstinspires.ftc.teamcode.script;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Main;
import org.firstinspires.ftc.teamcode.motor.MotorController;
import org.firstinspires.ftc.teamcode.script.task.Task;


public class TaskExecuting {
    public Main opMode;
    public TaskExecuting(Main opMode) {
         this.opMode = opMode;
    }
    TaskFetching tasks = new TaskFetching();

    public void execute() {
        Task task = tasks.tasks.peek();
        if(task == null) return;
        if(task.is_done()) {
            task.run();
        } else {
            tasks.tasks.poll();
        }
    }

}
