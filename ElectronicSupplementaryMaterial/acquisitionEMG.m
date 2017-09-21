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

setParam(loadParams());
param = getParam();

%% obtain MVC
MVC = calculateMVC(param.channels(muscle));

%% display current incoming data
f = figure();
scrsz = get(groot,'ScreenSize');
f.Position = [2000 scrsz(4) scrsz(3) scrsz(4)];

emg_array = zeros(100);
data = struct('EMG', {}, 'angle', {}, 'force', {});
i = 0;
j = 1;
while i <= numTrials
    
    call(move_motor, request);
    tic

    %% save emg signal, time, muscle force, current angle
    while toc < 3
        [emg_msg,~] = judp('RECEIVE',16573,400);
        emg = jsondecode(char(emg_msg));
        emg_array(j) = filterEMG(emg(param.channels(muscle)), MVC);
        %hold on;
        plot(1:j, emg_array(1:j), '-');
        %plot(j, emg(1), '-x');
        j = j + 1;
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
