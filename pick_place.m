global Px100;
Px100.DEVICENAME = 'COM4';
init_robot();

main_task();

function main_task()
    [pick_pose_base, drop_pose_base] = get_pick_and_place_position();

    above = trvec2tform([0.15, 0.10, 0.10]);
    pick_pose_base = trvec2tform([pick_pose_base(1,4), -pick_pose_base(2,4), 0.05]);    
    go_up = trvec2tform([0.15, 0.10, 0.10]);
    hover = trvec2tform([0.15, -0.10, 0.10]);
    drop_pose_base = trvec2tform([drop_pose_base(1,4), -drop_pose_base(2,4), 0.05]);
    home_hover = trvec2tform([0.15, -0.10, 0.19]);

    % Step 1: Execute pick
    execute_above(above);
    execute_pick(pick_pose_base);
    execute_go_up(go_up);
    execute_hover(hover);

    % Step 2: Execute place
    execute_place(drop_pose_base);
    execute_home_hover(home_hover);

    % Step 3: Return to home position
    return_home();
end

function execute_above(pos)
    current_joint_positions = get_joint_pos();
    joint_config = inverse_kinematics(pos);

    % Plan and execute trajectory
    waypoints = plan_trajectory(current_joint_positions, joint_config);
    execute_trajectory(waypoints);

    % Close gripper
    pause(0.5);
end

function execute_pick(pick_pose)
    current_joint_positions = get_joint_pos();
    pick_joint_config = inverse_kinematics(pick_pose);

    % Plan and execute trajectory
    waypoints = plan_trajectory(current_joint_positions, pick_joint_config);
    execute_trajectory(waypoints);

    % Close gripper
    closeGripper(1);
    pause(0.5);
end

function execute_go_up(pos)
    current_joint_positions = get_joint_pos();
    joint_config = inverse_kinematics(pos);

    % Plan and execute trajectory
    waypoints = plan_trajectory(current_joint_positions, joint_config);
    execute_trajectory(waypoints);

    % Close gripper
    pause(0.5);
end


function execute_hover(pos)
    current_joint_positions = get_joint_pos();
    joint_config = inverse_kinematics(pos);

    % Plan and execute trajectory
    waypoints = plan_trajectory(current_joint_positions, joint_config);
    execute_trajectory(waypoints);

    % Close gripper
    pause(0.5);
end

function execute_place(drop_pose)
    current_joint_positions = get_joint_pos();
    drop_joint_config = inverse_kinematics(drop_pose);

    % Plan and execute trajectory
    waypoints = plan_trajectory(current_joint_positions, drop_joint_config);
    execute_trajectory(waypoints);
    pause(0.5);

    % Open gripper
    closeGripper(0);
    pause(0.5);
end


function execute_home_hover(pos)
    current_joint_positions = get_joint_pos();
    joint_config = inverse_kinematics(pos);

    % Plan and execute trajectory
    waypoints = plan_trajectory(current_joint_positions, joint_config);
    execute_trajectory(waypoints);

    pause(0.5);
end

function return_home()
    home_config = [0, 0, 0, 0];
    current_joint_positions = get_joint_pos();

    % Plan and execute trajectory
    waypoints = plan_trajectory(current_joint_positions, home_config);
    execute_trajectory(waypoints);
end

function execute_trajectory(waypoints)
    for i = 1:size(waypoints, 1)
        set_joint_pos(waypoints(i, :));
        while ~is_at_position(waypoints(i, :))
            pause(0.01);
        end
    end
end

function joint_angles = inverse_kinematics(cartesian_position)
    global Px100;
    persistent ik;
    if isempty(ik)
        ik = inverseKinematics('RigidBodyTree', Px100.robot_model);
    end

    weights = [0.5, 0.5, 0, 1, 1, 1];
    initial_guess = Px100.robot_model.homeConfiguration;

    [configSol, ~] = ik('px100/ee_gripper_link', cartesian_position, weights, initial_guess);
    joint_angles = [configSol(1).JointPosition, configSol(2).JointPosition, configSol(3).JointPosition, configSol(4).JointPosition];
end

function waypoints = plan_trajectory(current_config, target_config)
    delta = target_config - current_config;
    num_steps = 30;
    waypoints = zeros(num_steps, length(current_config));

    for step = 1:num_steps
        waypoints(step, :) = current_config + (step / num_steps) * delta;
    end
end

function x = is_at_position(target_joint_config)
    current_joint_positions = get_joint_pos();
    x = norm(current_joint_positions - target_joint_config) < 0.01;
end
