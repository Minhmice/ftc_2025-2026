package org.firstinspires.ftc.teamcode.script;


import org.firstinspires.ftc.teamcode.motor.MotorController;
import org.firstinspires.ftc.teamcode.script.task.Task;


public class TaskExecuting {
    MotorController motors;
    public TaskExecuting(MotorController motors) {
         this.motors = motors;
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
