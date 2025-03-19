function serialRead(block)
    setup(block);
end

function setup(block)
    % Number of dialog parameters: COM port and baud rate
    block.NumDialogPrms = 2;
    
    % Register number of ports
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    % Setup output port properties
    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 4;
    block.OutputPort(1).SamplingMode = 'Sample';

    % Setup input port properties
    block.InputPort(1).Dimensions = 1;
    block.InputPort(1).DatatypeID = 0; % double
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = false;

    % Set block sample time
    block.SampleTimes = [1 0];

    % Set the block simStateCompliance to default (i.e., same as a built-in block)
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);    
    block.RegBlockMethod('Terminate', @Terminate);
end

function DoPostPropSetup(block)
    % Define Dwork vector structure without initializing runtime data.
    block.NumDworks = 1;
    block.Dwork(1).Name = 'serialData';
    block.Dwork(1).Dimensions = 4;         % To store a 4-element vector
    block.Dwork(1).DatatypeID = 0;         % 0 for double
    block.Dwork(1).Complexity = 'Real';
    % Do NOT initialize block.Dwork(1).Data here.
end

function Start(block)
    % Initialize the serial port and Dwork data here (runtime phase).
    global serialPortObj;
    comPort = block.DialogPrm(1).Data;   % COM port parameter
    baudRate = block.DialogPrm(2).Data;    % Baud rate parameter
    serialPortObj = serialport(comPort, baudRate);
    
    % Initialize the Dwork vector with a default value.
    block.Dwork(1).Data = [0 0 0 0];
    
    % Send the command "plot 1" at simulation start to setup the system.
    writeline(serialPortObj, 'plot 1');

    % Put model in open loop
    writeline(serialPortObj, 'mode 0');

    % Start the system
    writeline(serialPortObj, 'run');
end

function Outputs(block)
    global serialPortObj;
    if block.CurrentTime >= 0.001
        % Attempt to read from the serial port
        try
            rawData = readline(serialPortObj);
            data = checkAndConvert(rawData);
        catch
            % In case of error, use a zero vector
            data = [0 0 0 0];
        end

        % If the data received is all zeros, retain the previous valid reading.
        if all(data == 0)
            data = block.Dwork(1).Data;
        else
            % Otherwise, update the stored valid reading.
            block.Dwork(1).Data = data;
        end
        
        % Set the output port data.
        block.OutputPort(1).Data = data;
        
        % Write data to model
        pwmVal = block.InputPort(1).Data(1);
        if pwmVal < 0
            dir = 1;  % Negative value indicates reverse direction.
        else
            dir = 0;  % Otherwise, forward direction.
        end
        % Use absolute value for PWM magnitude.
        pwmVal = abs(pwmVal);
        
        % Send the PWM command.
        writeline(serialPortObj, sprintf('pwm %d', pwmVal));
        % Optionally display the PWM value.
        disp(pwmVal)
        
        % Send the direction command.
        writeline(serialPortObj, sprintf('dir %d', dir));
    end
end

function Terminate(block)
    % At simulation stop, send the command "plot 0" then close the serial port.
    global serialPortObj;
    if ~isempty(serialPortObj)
        try
            writeline(serialPortObj, 'plot 0');
            % stop the model
            writeline(serialPortObj, 'stop');
        catch
            % Handle error if needed (for example, if the port is already closed)
        end
        delete(serialPortObj);
        clear serialPortObj;
    end
end

function output = checkAndConvert(str)
    % Split the input string into parts based on spaces.
    parts = strsplit(str, " ");
    % Convert the parts to double values.
    nums = str2double(parts);
    
    % Check if conversion was successful and result is a 4-element vector.
    if all(~isnan(nums)) && numel(nums)==4
        output = nums;
    else
        % Return a default 4-element zero vector if conversion fails.
        output = [0 0 0 0];
    end
end
