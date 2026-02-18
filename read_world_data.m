
function world_gt = read_world(filepath)
    
    world_file = fopen(filepath, 'r');
    if world_file == -1
        error('Cannot open %s', filepath); 
    end
    
    % Format: LANDMARK_ID (1) POSITION (2:4) 
    data = fscanf(world_file, '%f', [4, Inf])';

    fclose(world_file);
    
    % Storing in a map for lookup by ID
    world_gt = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    for i = 1:size(data, 1)
        id = data(i, 1);
        pos = data(i, 2:4)'; % [x; y; z]
        world_gt(id) = pos;
    end
end