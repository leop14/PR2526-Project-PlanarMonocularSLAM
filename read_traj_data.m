

% Loading the data file
function traj_data = read_traj_data(filename)
    % Open the file
    traj_file = fopen(filename, 'r');

    if traj_file == -1
        error('Cannot open file: %s', filename);
    end
    
    % Reading the data matrix.
    % The file has 7 columns. We can use fscanf to read the file all at once
    % It fills the matrix column-by-column, so we create a [7 x N] matrix.
    raw_data = fscanf(traj_file, '%f', [7, Inf])';
    % Transpose to get a [N x 7] matrix where N is the number of poses
    
    fclose(traj_file);

    
    % Extract columns based on README specifications
    % Column 1: Pose ID
    traj_data.id = raw_data(:, 1);
    
    % Columns 2-4: Odometry (x, y, theta)
    traj_data.odom = raw_data(:, 2:4);
    
    % Columns 5-7: Ground Truth (x, y, theta)
    traj_data.gt = raw_data(:, 5:7);

    fprintf('\nLast pose - as verification:\n');
    N = length(traj_data.id);
    fprintf('%5d | %8.4f %8.4f %8.4f | %8.4f %8.4f %8.4f\n', ...
        traj_data.id(N), ...
        traj_data.odom(N, 1), traj_data.odom(N, 2), traj_data.odom(N, 3), ...
        traj_data.gt(N, 1), traj_data.gt(N, 2), traj_data.gt(N, 3)
    );

    
end





