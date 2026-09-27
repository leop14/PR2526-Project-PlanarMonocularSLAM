
function draw_error_vs_obs(errors_list, num_obs_list, labels, colors, outlier_threshold, save_name)
    % Error of each landmark after BA (log scale) vs the number of frames in
    % which it was observed. One subplot per method.
    % errors_list and num_obs_list are cell arrays with one vector per method,
    % colors has one RGB row per method.
    if nargin < 6
        save_name = 'landmark_error_vs_obs';
    end

    num_methods = numel(errors_list);
    max_obs = max(cellfun(@max, num_obs_list));
    all_err = [errors_list{:}];
    y_lim = [max(min(all_err), 1e-4), max(all_err)] .* [0.5, 2];

    figure('Name', save_name, 'NumberTitle', 'off');
    for m = 1:num_methods
        subplot(num_methods, 1, m);
        hold on; grid on;
        plot(num_obs_list{m}, errors_list{m}, 'o', 'Color', colors(m, :), 'MarkerSize', 3);
        plot([0, max_obs + 1], [outlier_threshold, outlier_threshold], 'k--');
        set(gca, 'yscale', 'log');
        xlim([0, max_obs + 1]);
        ylim(y_lim);
        ylabel('Error (m)');
        title(sprintf('%s  (%d / %d landmarks > %.1f m)', labels{m}, ...
                      sum(errors_list{m} > outlier_threshold), numel(errors_list{m}), ...
                      outlier_threshold));
    end
    xlabel('Number of observations');

    if ~exist('figures', 'dir')
        mkdir('figures');
    end
    out_path = sprintf('figures/%s.png', save_name);
    try
        print(gcf, out_path, '-dpng', '-r150');
        fprintf('Figure saved to: %s\n', out_path);
    catch err
        fprintf('Warning: could not save figure (%s)\n', err.message);
    end
    drawnow;
end
