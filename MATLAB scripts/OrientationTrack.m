

clear

% Set the COM port number and baud rate
comPort = 'COM5';  % Modify this with the appropriate COM port
baudRate = 115200;  % Modify this with the appropriate baud rate


qs = quaternion([0,0,0],'eulerd','ZYX','frame');
ps = [0 ,0 ,0];
setView = HelperOrientationViewer("Title",{"IMU Sensor Fusing"},"ReferenceFrame", "NED" );
% patch = poseplot(qs , ps , "ENU");


try
    % Create a serial object
    port = serialport(comPort, baudRate);

    % Start communication with the board
    write(port,'#' ,"char");

    % Read and print the data
    while true
        % Read data from the serial port
        check = read(port, 1, "char");

        if check == '$'
            rotations = read(port, 3, "single");
        end

        qs = quaternion(rotations , "eulerd" ,"ZYX","frame");
        setView(qs); 


    end

catch exception
    % Display an error message if an exception occurs
    disp(['Error: ' exception.message]);

    % Close and delete the serial port object
    clear s;
end


