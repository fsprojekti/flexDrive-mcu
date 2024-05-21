function serialRead(block)
    setup(block);
end

function setup(block)

    block.NumDialogPrms = 2;
    
    % Register number of ports
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    % Setup port properties
    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 2;
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
    % Set number of Dwork vectors
    block.NumDworks = 1;
    block.Dwork(1).Name = 'serialData';  % Name it for clarity
    block.Dwork(1).Dimensions = 1;  % Adjust based on needs
    block.Dwork(1).DatatypeID = 0;  % 0 for double
    block.Dwork(1).Complexity = 'Real';
end

function Start(block)
    % Initialize serial port here
    global serialPortObj;  % Declare the global variable
    % Read parameters from dialog
    comPort = block.DialogPrm(1).Data; % COM port parameter
    baudRate = block.DialogPrm(2).Data; % Baud rate parameter
    serialPortObj = serialport(comPort, baudRate);
end

function Outputs(block)
    
    global serialPortObj;
    if block.CurrentTime >= 0.001
        % Read data from the serial port
        try
            data = readline(serialPortObj);
            data = checkAndConvert(data);
            block.OutputPort(1).Data = data;
        catch
            block.OutputPort(1).Data = [0 0]; % In case of read error
        end

        % Write data to seriap port
        dataToSend = block.InputPort(1).Data;
        writeline(serialPortObj, strtrim(sprintf('%.0f ', dataToSend)));
    end
    
end

function Terminate(block)
    % Close serial port
    global serialPortObj;
    if ~isempty(serialPortObj)
        delete(serialPortObj);
        clear serialPortObj;
    end
end

function output = checkAndConvert(str)
    % Split the string into parts based on spaces
    parts = strsplit(str," ");
    % Convert parts to double
    nums = str2double(parts);
    
    % Check if all elements are numbers (not NaN)
    if all(~isnan(nums))
        output = nums; % Return the numbers if all are valid
    else
        output = [0 0]; % Return [0 0] if any element is not a number
    end
end
