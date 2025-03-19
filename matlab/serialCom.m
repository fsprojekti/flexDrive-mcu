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
    block.Dwork(1).Dimensions = 4;
    block.Dwork(1).DatatypeID = 0;
    block.Dwork(1).Complexity = 'Real';
end

function Start(block)
    % Initialize the serial port and Dwork data here (runtime phase).
    global serialPortObj;
    comPort = block.DialogPrm(1).Data;  % COM port parameter
    baudRate = block.DialogPrm(2).Data;   % Baud rate parameter
    serialPortObj = serialport(comPort, baudRate);

    % Wait for handshake "READY" message from the microcontroller.
    timeout = 5; % seconds timeout
    tStart = tic;
    readyReceived = false;
    while toc(tStart) < timeout
        pause(0.1); % Small delay to avoid tight looping
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

    % Initialize the Dwork vector with a default value.
    block.Dwork(1).Data = [0 0 0 0];

    % Now that the handshake is complete, send the command "plot 1" to set up the system.
    writeline(serialPortObj, 'plot 1');
end

function Outputs(block)
    global serialPortObj;
    if block.CurrentTime >= 0.001
        % Use the input port value for the PWM command.
        pwmVal = block.InputPort(1).Data(1);
        if pwmVal < 0
            dir = 1;  % Negative value indicates reverse.
        else
            dir = 0;  % Otherwise, forward.
        end
        pwmVal = abs(pwmVal);

        % Send the PWM and direction commands.
        writeline(serialPortObj, sprintf('pwm %d', pwmVal));
        disp(pwmVal);
        writeline(serialPortObj, sprintf('dir %d', dir));
    end
end

function Terminate(block)
    % At simulation stop, send the command "plot 0" then close the serial port.
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
