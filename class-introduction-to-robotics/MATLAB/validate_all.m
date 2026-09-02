function validate_all()
    scripts = {
        'ch2_spatial_transformations', ...
        'ch3_forward_kinematics_dh', ...
        'ch4_inverse_kinematics', ...
        'ch5_jacobian_and_velocity', ...
        'ch6_manipulator_dynamics', ...
        'ch7_trajectory_generation', ...
        'ch8_mechanism_design_sizing', ...
        'ch9_linear_joint_control', ...
        'ch10_computed_torque_control', ...
        'ch11_force_and_impedance_control'
    };

    all_ok = true;
    for k = 1:length(scripts)
        s_name = scripts{k};
        fprintf('\n=========================================\n');
        fprintf('>>> Testing script [%d/%d]: %s.m\n', k, length(scripts), s_name);
        fprintf('=========================================\n');
        try
            run_single(s_name);
            fprintf('\n>>> [PASS] %s.m executed successfully!\n', s_name);
        catch ME
            fprintf('\n>>> [FAIL] %s.m failed with error:\n%s\n', s_name, ME.message);
            all_ok = false;
        end
    end

    if all_ok
        fprintf('\n\n**************************************************\n');
        fprintf('  ALL 10 ROBOTICS MATLAB SCRIPTS PASSED (100%%)!\n');
        fprintf('**************************************************\n');
    else
        error('Some scripts failed execution!');
    end
end

function run_single(s_name)
    feval(s_name);
end
