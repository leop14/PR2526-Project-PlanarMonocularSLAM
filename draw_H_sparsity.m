
function draw_H_sparsity(H, num_poses, num_landmarks, label, save_name)
    % Sparsity pattern of the BA system matrix H, with the pose/landmark
    % boundary marked. Nothing is drawn if H is empty.
    if isempty(H)
        return;
    end

    n_pose_vars = num_poses * 3;
    n_lm_vars   = num_landmarks * 3;

    figure('Name', save_name, 'NumberTitle', 'off');
    spy(H);
    hold on;
    % Dashed lines separating the pose block from the landmark block
    n = size(H, 1);
    plot([n_pose_vars, n_pose_vars] + 0.5, [0, n], 'r--', 'LineWidth', 1.5);
    plot([0, n], [n_pose_vars, n_pose_vars] + 0.5, 'r--', 'LineWidth', 1.5);
    title({sprintf('H sparsity (%s, iter 1)', label), ...
           sprintf('%d pose + %d landmark vars, nnz = %d', n_pose_vars, n_lm_vars, nnz(H))});
    xlabel('Variable index'); ylabel('Variable index');

    if ~exist('figures', 'dir')
        mkdir('figures');
    end
    out_path = sprintf('figures/%s.png', save_name);
    try
        print(gcf, out_path, '-dpng', '-r150');
        fprintf('Figure saved to: %s\n', out_path);
    catch err
        fprintf('Warning: could not save H sparsity figure (%s)\n', err.message);
    end
    drawnow;
end
