function serialRead(block)
    setup(block);
end

%% Setup: Configure ports, sample time, and register methods.
function setup(block)
    % Dialog parameters: COM port and baud rate.
    block.NumDialogPrms = 1;
    
    % No input ports; one output port for encoder data (4-element vector).
    block.NumInputPorts = 0;
    block.NumOutputPorts = 1;
    
    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 4;
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Sample time.
    block.SampleTimes = [0.01 0];
    
    % Default simulation state compliance.
    block.SimStateCompliance = 'DefaultSimState';
    
    % Register methods.
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);    
    block.RegBlockMethod('Terminate', @Terminate);
end

%% Define a Dwork vector to hold the latest valid encoder reading.
function DoPostPropSetup(block)
    block.NumDworks = 1;
    block.Dwork(1).Name = 'encoderData';
    block.Dwork(1).Dimensions = 4;
    block.Dwork(1).DatatypeID = 0;  % double
    block.Dwork(1).Complexity = 'Real';
    % Do not initialize block.Dwork(1).Data here.
end

%% Start: Open the serial port, wait for the handshake, and send the initial command.
function Start(block)
    global serialPortObj;
    comPort = block.DialogPrm(1).Data;
    baudRate = 115200;
    serialPortObj = serialport(comPort, baudRate);
    
    % Wait (up to 5 seconds) for the microcontroller to send "READY".
    timeout = 5; % seconds
    tStart = tic;
    readyReceived = false;
    while toc(tStart) < timeout
        pause(0.1);
        if serialPortObj.NumBytesAvailable > 0
            line = readline(serialPortObj);
            if contains(line, 'READY', 'IgnoreCase', true)
                readyReceived = true;
                break;
            end
        end
    end
    if ~readyReceived
        warning('serialRead: Handshake failed: No READY message received.');
    else
        disp('serialRead: Handshake successful.');
    end
    
    % Initialize the stored encoder data.
    block.Dwork(1).Data = [0 0 0 0];
    
    % Send an initial command (e.g., "plot 1") to set up the microcontroller.
    writeline(serialPortObj, 'plot 1');
    % Put system in run mode
    writeline(serialPortObj, 'run');
end

%% Outputs: Read a line, parse for encoder data, and output the data.
function Outputs(block)
    global serialPortObj;
    try
        rawData = readline(serialPortObj);
        encoderData = checkAndConvert(rawData);
    catch
        encoderData = [0 0 0 0];
    end
    
    % If the parsed data is not valid (returned all zeros), keep the previous valid reading.
    if all(encoderData == 0)
        encoderData = block.Dwork(1).Data;
    else
        block.Dwork(1).Data = encoderData;
    end
    
    block.OutputPort(1).Data = encoderData;
    % pause for 10ms
    %pause(0.01);
end

%% Terminate: Send shutdown command and close the port.
function Terminate(block)
    global serialPortObj;
    if ~isempty(serialPortObj)
        try
            writeline(serialPortObj, 'plot 0');
            writeline(serialPortObj, 'stop');
        catch
            % Ignore errors during termination.
        end
        delete(serialPortObj);
        clear serialPortObj;
    end
end

%% Helper: Parse a line from the serial port to extract encoder data.
function output = checkAndConvert(str)
    str = strtrim(str);
    % Check if the line starts with a digit or minus sign.
    if isempty(regexp(str, '^[\d\-]', 'once'))
        output = [0 0 0 0];
        return;
    end
    % Split by whitespace.
    parts = strsplit(str);
    if numel(parts) ~= 4
        output = [0 0 0 0];
        return;
    end
    nums = str2double(parts);
    if any(isnan(nums))
        output = [0 0 0 0];
    else
        output = nums;
    end
end
