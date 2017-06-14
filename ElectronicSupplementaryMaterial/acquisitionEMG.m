function acquisitionEMG(numTrials, muscle)

session = input('session name: ', 's');
%% ros setup
%rosinit('localhost');
move_motor = rossvcclient('/myo_blink/move');
topic = '/myo_blink/joints/lower/angle';
muscle_sub = rossubscriber(topic);

request = rosmessage(move_motor);
request.Muscle = muscle;
request.Action = 'keep';
force = 40;
request.Setpoint = force;

%% start recording
data = struct('EMG', {}, 'angle', {}, 'force', {});

%% set force on the arm
i = 0;
while i <= numTrials
    
    call(move_motor, request);
    tic

    %% save emg signal, time, muscle force, current angle
    while toc < 3
        [emg_msg,~] = judp('RECEIVE',16573,200);
        emg = jsondecode(char(emg_msg));
        joint_msg = receive(muscle_sub);
        joint_angle = joint_msg.Data;
        data(end+1) = struct('EMG', emg, 'angle', joint_angle, 'force', request.Setpoint);
    end
    
    request.Setpoint = 38;
    call(move_motor, request);
    i = i + 1;
    request.Setpoint = force + 5;
    pause(2);
end

save(strcat(session, '.mat'), 'data');
end
