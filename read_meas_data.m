
function read_meas = read_meas_data(meas_dir)

    % Getting the list of measurements files
    pattern = fullfile(meas_dir, "meas-*.dat");
    file_list = dir(pattern);

    if isempty(file_list)
        error("No meas file found in %s", meas_dir);
    end

    % Sorting the files
    filenames = {file_list.name};   # Get the "cell array" of file names
    file_nums = cellfun(@(x) str2double(regexp(x, '\d+', "match", "once")),...
                        filenames);
    # @(x) defines a lambda function. In this case it is a function to
    # extract the sequence number from the file name
    # "cellfun" applies the function to every element in filenames
    [~, sort_idx] = sort(file_nums);
    ord_file_list = file_list(sort_idx);


    num_files = length(file_list);
    fprintf("Loading %d measurement frames", num_files);

    measurements = cell(num_files, 1);

    % Iterating on the ordered files
    for i = 1:num_files
        fname = fullfile(meas_dir, ord_file_list(i).name);
        meas_file = fopen(fname, "r");

        if meas_file == -1
            warning("Could not open %s", fname);
            continue;
        end

        % Line 1: "seq: <id>"
        line = fgetl(meas_file);
        seq_num = sscanf(line, 'seq: %d'); 
        
        % Line 2: "gt_pose: <x> <y> <theta>"
        line = fgetl(meas_file);
        gt_pose = sscanf(line, 'gt_pose: %f %f %f');
        
        % Line 3: "odom_pose: <x> <y> <theta>"
        line = fgetl(meas_file);
        odom_pose = sscanf(line, 'odom_pose: %f %f %f');

        % Parsing Landmarks' Measurements
        % Format: "point <local_id> <actual_id> <u> <v>"    
        raw_data = textscan(meas_file, '%*s %d %d %f %f');
        % We can give "textscan" the pattern to read in every line,
        % It returns a Cell Array 
        %   raw_data(0): "point " word
        %   raw_data(1): local_id (discarded later)
        %   raw_data(2): actual_id
        %   raw_data(3,4): pixel coords (p_img)
        fclose(meas_file);
        

        % Store in Structure 
        frame = struct();
        frame.seq_num = seq_num;
        frame.gt_pose = gt_pose;     
        frame.odom_pose = odom_pose;

        % raw_data is a cell array: {local_id, actual_id, u, v}
        if ~isempty(raw_data{2})
            frame.point_ids = double(raw_data{2});    % The Actual IDs
            frame.observations = [raw_data{3}, raw_data{4}]; % The pixels coords
        else
            frame.point_ids = [];
            frame.observations = [];
        end
        
        % Save in the returned variable "measurements"
        measurements{i} = frame;

    end
    
    read_meas = measurements;
    fprintf('\nMeasurements Loaded\n');

    %measurements
    disp(measurements);
end