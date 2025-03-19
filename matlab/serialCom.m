function serialRead(block)
    setup(block);
end

%% Setup function: Configure ports, sample times, and register methods.
function setup(block)
    % Number of dialog parameters: COM port and baud rate.
    block.NumDialogPrms = 2;
    
    % Register number of ports.
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;
    
    % Setup output port properties (encoder data: 4 elements).
    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 4;
    block.OutputPort(1).SamplingMode = 'Sample';
    
    % Setup input port properties (PWM command; assume 2 elements, use first).
    block.InputPort(1).Dimensions = 1;
    block.InputPort(1).DatatypeID = 0; % double
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = false;
    
    % Set block sample time.
    block.SampleTimes = [1 0];
    
    % Set the block simStateCompliance to default (same as a built-in block).
    block.SimStateCompliance = 'DefaultSimState';
    
    % Register methods.
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);    
    block.RegBlockMethod('Terminate', @Terminate);
end

%% PostPropagationSetup: Define a Dwork vector to store the latest valid encoder data.
function DoPostPropSetup(block)
    block.NumDworks = 1;
    block.Dwork(1).Name = 'serialData';
    block.Dwork(1).Dimensions = 4;         % To store a 4-element vector.
    block.Dwork(1).DatatypeID = 0;         % double.
    block.Dwork(1).Complexity = 'Real';
    % Do NOT initialize block.Dwork(1).Data here.
end

%% Start: Open the serial port, wait for handshake, initialize Dwork, and send initial command.
function Start(block)
    global serialPortObj;
    comPort = block.DialogPrm(1).Data;  % COM port parameter.
    baudRate = block.DialogPrm(2).Data;   % Baud rate parameter.
    serialPortObj = serialport(comPort, baudRate);
    
    % Wait for the handshake "READY" message from the microcontroller.
    timeout = 5; % seconds.
    tStart = tic;
    readyReceived = false;
    while toc(tStart) < timeout
        pause(0.1); % Avoid tight looping.
        if serialPortObj.NumBytesAvailable > 0
            line = readline(serialPortObj);
            if contains(line, 'READY', 'IgnoreCase', true)
                readyReceived = true;
                break;
            end
        end
    end
    if ~readyReceived
        warning('Handshake failed: No READY message received from microcontroller.');
    else
        disp('Handshake successful: Microcontroller is READY.');
    end
    
    % Initialize the Dwork vector with a default encoder reading.
    block.Dwork(1).Data = [0 0 0 0];
    
    % Now that the handshake is complete, send the initial command to set up the system.
    writeline(serialPortObj, 'plot 1');

    % Put system in open loop
    writeline(serialPortObj, 'mode 0');
end

%% Outputs: Read and parse incoming serial data, update output, and send PWM/direction commands.
function Outputs(block)
    global serialPortObj;
    if block.CurrentTime >= 0.001
        % Attempt to read a line from the serial port.
        try
            rawData = readline(serialPortObj);
            encoderData = checkAndConvert(rawData);
        catch
            encoderData = [0 0 0 0];
        end
        
        % If the parsed data is not valid (returned default [0 0 0 0]),
        % retain the previous valid reading.
        if all(encoderData == 0)
            encoderData = block.Dwork(1).Data;
        else
            block.Dwork(1).Data = encoderData;
        end
        
        % Set the output port data (encoder values).
        block.OutputPort(1).Data = encoderData;
        
        % Process PWM command using the input port value.
        % The first element is used for PWM value; its sign sets direction.
        pwmVal = block.InputPort(1).Data(1);
        if pwmVal < 0
            dir = 1;  % Negative value → Reverse.
        else
            dir = 0;  % Otherwise → Forward.
        end
        pwmVal = abs(pwmVal);
        
        % Send the PWM and direction commands to the microcontroller.
        writeline(serialPortObj, sprintf('pwm %d', pwmVal));
        writeline(serialPortObj, sprintf('dir %d', dir));
    end
end

%% Terminate: Send a shutdown command and close the serial port.
function Terminate(block)
    global serialPortObj;
    if ~isempty(serialPortObj)
        try
            writeline(serialPortObj, 'plot 0');
        catch
            % Handle errors if necessary.
        end
        delete(serialPortObj);
        clear serialPortObj;
    end
end

%% checkAndConvert: Parse a line from the serial port to extract encoder data.
% This function returns a 4-element vector if the string contains exactly 4
% numeric tokens (the first character must be a digit or a minus sign).
% Otherwise, it returns [0 0 0 0].
function output = checkAndConvert(str)
    % Remove leading/trailing whitespace.
    str = strtrim(str);
    
    % Check if the string starts with a digit or minus sign.
    if isempty(regexp(str, '^[\d\-]', 'once'))
        output = [0 0 0 0];
        return;
    end
    
    % Split the string by whitespace.
    parts = strsplit(str);
    
    % Validate that there are exactly 4 tokens.
    if numel(parts) ~= 4
        output = [0 0 0 0];
        return;
    end
    
    % Convert tokens to numeric values.
    nums = str2double(parts);
    if any(isnan(nums))
        output = [0 0 0 0];
    else
        output = nums;
    end
end
