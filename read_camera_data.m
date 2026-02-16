

% Loading the data file
function cam_data = read_camera_data(filename)
    % Opening the file
    cam_file = fopen(filename, "r");

    % Skipping the header line "camera matrix:"
    fgetl(cam_file);

    % Reading the Camera Matrix K
    % It is a 3x3 matrix
    K = fscanf(cam_file, '%f', [3, 3])';
    % Result is transposed since "fscanf" reads column-by-column

    % fscanf leaves the file pointer at the end of the values.
    fgetl(cam_file); % We need to consume the leftover newline character.
    % And also to skip the header line "cam_transform"
    fgetl(cam_file);    

    % Reading Camera Transform w.r.t. Robot frame (Extrinsics parameters)
    T = fscanf(cam_file, "%f", [4, 4])';

    fgetl(cam_file);

    % Reading "z_near"
    line = fgetl(cam_file);
    z_near = sscanf(line, "z_near: %f");

    % Reading "z_far"
    line = fgetl(cam_file);
    z_far = sscanf(line, "z_far: %f");

    % Reading "width"
    line = fgetl(cam_file);
    width = sscanf(line, "width: %d");

    % Reading "height"
    line = fgetl(cam_file);
    height = sscanf(line, "height: %d");

    % Close the file
    fclose(cam_file);

    % Storing data
    cam_data.K = K;
    cam_data.T = T;
    cam_data.z_near = z_near;
    cam_data.z_far = z_far;
    cam_data.height = height;
    cam_data.width = width;

    % Printing gathered data
    printf("Camera Loaded\n");
    printf("Dimensions: %dx%d\n", width, height);
    disp("K:"); disp(K);
    disp("Camera pose w.r.t robot:"); disp(T);

end



