package org.firstinspires.ftc.teamcode.script;

import org.firstinspires.ftc.teamcode.script.task.Task;
import org.firstinspires.ftc.teamcode.script.task.backward;
import org.firstinspires.ftc.teamcode.script.task.forward;

import java.util.ArrayDeque;
import java.util.Queue;

public class TaskFetching {

    public Queue<Task> tasks;
    public TaskFetching() {
        tasks = new ArrayDeque<>();
        String[] instructions = SCRIPT.script.split("\n");
        for(String instruction : instructions) {
            String command = instruction.substring(0, 3);
            switch (command) {
                /* EXAMPLE
                case "fwd": {
                    tasks.add(new forward());
                    break;
                }
                case "bwd": {
                    tasks.add(new backward());
                    break;
                }
                */
            }
        }
    }


}
