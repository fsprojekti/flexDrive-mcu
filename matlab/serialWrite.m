function serialWrite(block)
    setup(block);
end

function setup(block)
    block.NumDialogPrms = 1;  % e.g., COM port parameter
    block.NumInputPorts = 1;
    block.NumOutputPorts = 0;
    
    block.InputPort(1).Dimensions = 1;  % [PWM value, ...] (first element is used)
    block.InputPort(1).DatatypeID = 0;
    block.InputPort(1).Complexity = 'Real';
    block.InputPort(1).DirectFeedthrough = true;
    
    block.SampleTimes = [1 0];  % match the read block's sample time (or adjust as needed)
    block.SimStateCompliance = 'DefaultSimState';
    
    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Terminate', @Terminate);
end

% function Start(block)
%     global serialPortObj;
%     if isempty(serialPortObj) || ~isvalid(serialPortObj)
%         comPort = block.DialogPrm(1).Data;
%         baudRate = 115200;
%         serialPortObj = serialport(comPort, baudRate);
%         % You might also wait for the handshake here if this block runs first.
%     end
% end
function Start(block)
    global serialPortObj;
end

function Outputs(block)
    global serialPortObj;
    cmdData = block.InputPort(1).Data;
    pwmVal = cmdData(1);
    if pwmVal < 0
        dir = 1;
    else
        dir = 0;
    end
    pwmVal = abs(pwmVal);
    
    if ~isempty(serialPortObj) && isvalid(serialPortObj)
        writeline(serialPortObj, sprintf('pwm %d', int32(pwmVal)));
        writeline(serialPortObj, sprintf('dir %d', dir));
    end
end

function Terminate(block)
    % Do not close the port here; let the read block handle it.
end
