function measurements_db = new_read_meas_data(meas_dir)    % Initialize a structure to hold feature tracks
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


    % Initialize the Database (Map)
    % Keys: Landmark ID (int)
    % Values: Struct containing list of observations
    measurements_db = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    
    num_files = length(file_list);
    fprintf('Building Measurements DB from %d files\n', num_files);
    
    % Iterate through every measurement file
    for i = 1:num_files
        fname = fullfile(meas_dir, file_list(i).name);
        meas_file = fopen(fname, 'r');
        
        if meas_file == -1
            warning('Could not open %s', fname);
            continue;
        end
        
        # Scan seq_num
        line = fgetl(meas_file);
        seq_num = sscanf(line, 'seq: %d');
        
        # Scan gt_pose
        line = fgetl(meas_file);
        gt_pose = sscanf(line, 'gt_pose: %f %f %f'); 
        
        # Scan odom_pose
        line = fgetl(meas_file);
        odom_pose = sscanf(line, 'odom_pose: %f %f %f'); 
        
        % Parse Observations 
        % We can give "textscan" the pattern to read in every line,
        % Format matches : "point <local_id> <actual_id> <u> <v>"
        % We discard "point" and "local_id".
        % We keep "actual_id" (Column 2) and "u, v" (Columns 3, 4)
        % It returns a Cell Array 
        raw_data = textscan(meas_file, 'point %d %d %f %f');
    
        fclose(meas_file);  # We've scanned all the file

        actual_ids = raw_data{2};
        us = raw_data{3};
        vs = raw_data{4};
        
        if isempty(actual_ids)
            continue;
        end
        

        % Update the Map 
        for k = 1:length(actual_ids)
            id = int32(actual_ids(k));
            
            % Create the observation struct for this specific frame
            new_obs.seq_num = seq_num;
            new_obs.gt_pose = gt_pose;     % For later evaluation
            new_obs.odom_pose = odom_pose; 
            new_obs.uv = [us(k), vs(k)];
            
            if isKey(measurements_db, id)
                % If landmark_id already exists: Append new entry
                entry = measurements_db(id);
                entry.count = entry.count + 1;
                entry.observations{end+1} = new_obs; 
                measurements_db(id) = entry;
            else
                % Else: Initialize new array
                entry.count = 1;
                entry.observations = {new_obs}; % New Cell array of structs
                measurements_db(id) = entry;
            end
        end
        
    end
    
    fprintf('Done. Database contains %d unique point IDs.\n', length(measurements_db));
end