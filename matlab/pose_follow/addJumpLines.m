function addJumpLines(ax, run, jointIndex, style)
%ADDJUMPLINES Mark IK target jumps affecting one joint.
for eventIndex = 1:numel(run.ik_target_jumps.times_sec)
    triggered = string(run.ik_target_jumps.triggered_joints{eventIndex});
    if any(triggered == run.joint_names(jointIndex))
        xline(ax, run.ik_target_jumps.times_sec(eventIndex), ':', ...
            'Color', style.jump_color_rgb, 'LineWidth', 1.1, ...
            'HandleVisibility', 'off');
    end
end
end
