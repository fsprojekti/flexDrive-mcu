% % Motor Driver Parameters
% % Supply voltage for the motor driver
% motorDriver.supplyVoltageV = 6; % Supply voltage in Volts
% 
% % Motor Parameters
% motor.noLoadSpeedRPM = 290;     % No-load speed in RPM
% motor.stallCurrentA = 2.1;      % Stall current in Amperes [A]
% motor.resistanceOhms = motorDriver.supplyVoltageV / motor.stallCurrentA; % Motor resistance in Ohms
% 
% % Back EMF (Electromotive Force) constant
% motor.backEMFConstantVPerRad = motorDriver.supplyVoltageV / (motor.noLoadSpeedRPM * 2 * pi / 60); % Back EMF constant in V/rad/s
% 
% % Torque constant (Nm/A), assumed to be equal to back EMF constant in SI units
% motor.torqueConstantNmPerA = motor.backEMFConstantVPerRad; % Torque constant (Nm/A)
% 
% % No-load current (from datasheet)
% motor.noLoadCurrentA = 0.09; % No-load current in Amperes
% 
% % Gearbox Parameters (if applicable)
% gearbox.ratio = 20.4; % Gear ratio
% 
% % Torque-Speed Linear Relationship Parameters (from the motor characteristics)
% motor.speedTorqueSlope = 11;  % Slope of the speed-torque linear relation (RPM per kg-mm)
% motor.speedNoLoad = 290;      % No-load speed in RPM (already included above)
% motor.stallTorqueKgMm = 13;   % Stall torque in kg-mm (from the datasheet)
% 
% % Torque-Current Relationship
% motor.currentTorqueSlope = 0.079; % Slope of current vs. torque (A/kg-mm)
% 
% % Final torque equation variables (constants for further use)
% motor.torqueIntercept = motorDriver.supplyVoltageV - (motor.backEMFConstantVPerRad * motor.noLoadSpeedRPM);
% motor.torqueSlope = -(motor.backEMFConstantVPerRad * motor.speedTorqueSlope) + (motor.resistanceOhms * motor.currentTorqueSlope);
% 
% % Display motor structure
% disp(motor);

% Motor Parameters
V = 6;                    % Supply Voltage (V)
I_stall = 2.1;            % Stall Current (A)
tau_stall_out_kgmm = 13;  % Stall Torque (kg-mm)
N = 20.4;                 % Gear Ratio (motor:output)

% Unit Conversions
kgmm_to_Nm = 0.00980665;  % Conversion factor
tau_stall_out = tau_stall_out_kgmm * kgmm_to_Nm; % Stall Torque (N.m)

% Calculated Parameters
Kt = tau_stall_out / I_stall;          % Torque Constant (N.m/A)
Ke = Kt;                                % Back EMF Constant (V/(rad/s))
R = V / I_stall;                        % Armature Resistance (Ohm)

% Assumed Parameters
J = 0.01;               % Moment of Inertia (kg.m^2)
L = 0.5;                % Armature Inductance (H)
b = 5.14e-6;            % Viscous Friction Coefficient (N.m.s/rad)

% Transfer Function Parameters
s = tf('s');
numerator = N * Kt * (J * s + b);
denominator = (J * L) * s^2 + (J * R + b * L) * s + (b * R + Kt * Ke);
P_torque = numerator / denominator;

% Display Transfer Function
disp('Open-Loop Transfer Function (Voltage to Output Torque):');
disp(P_torque);

