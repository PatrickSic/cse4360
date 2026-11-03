sd card files

/proj1
    proj1.cfg -> robot parameters: wheel diameter, gains, ports, signs, etc
    robot.h/c -> gyro and encorder motions
    params.h/.c -> runtime parameter overides passed from proj1.cfg and/or course.txt
    pid.h/.c -> PID controller used by robot.c
    planner.h/.c -> manhattan distance grid path planner
    planner_draw.h/.c -> draws a path of the grid
    course.txt -> course for the robot to take: start, goal, obstacles. Currently it is the provided course from the professor
    
    /tests
        diag -> diagnostics menu (gyro drift/sign, 2m straight, 90 degree turns, motor test)
        trim -> measures motor left and right pull
        move1s -> moves the robot foward for one second

    /courses
        example -> provided example from the project pdf
        diagonal -> a path where the robot must travel at a straight diaganoal
        snake -> a path where the robot has to take constant curves 
  
    /host
        robot_host.c & ev3_stub.h are for simulating the inputs of the robot digitally

the brick screen shows:
    cfg:file course:file - when both files were read
    cfg:default / course:course:h - when not
    cfg:ERR - compilation error

editing
  course.txt  start x y / goal x y / obstacle x y  (metres, hand-out frame, origin bottom-left, X along the 16-tile side).  
              Delete or rename the file to use the compiled-in example course.

Building
    I built this on a linux mint machine, the code compiles fine on both of them, however the kernel for the ev3 brick is outdated so I had to use ziglang to cross-compile that.
    
    sudo apt update
    sudo apt install build-essential git python3 python3-pip libudev-dev pkg-config
        (the makefile looks for ../EV3-API, ...so that's the reason why proj1/ and ev3-api/ have to stay together in the root)
    pip install --user "ziglang==0.14.1"
    python3 -m ziglang version (You should see 0.14.1 output)
        if pip3 install --user throws an externally managed environment,
            pip3 install --user --break-system-packes "ziglang==0.14.1"

Using (standalone)
        Running main.c without being connected to an ev3 brick will just run a simulation of the course.h without any physicality.

    cd ~/pr1/proj1
    make host && ./proj1_host
    ./proj1_host courses/snake.txt    

ascii map legend:
 *   #  obstacle   .  blocked (too close)   *  path   o  waypoint
 *   S  start      G  goal                  show_dist: also the distance map */


    
