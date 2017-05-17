function calculate_force()
%rosinit('127.0.0.1');
name = 'biceps';
topic = strcat('/myo_blink/muscles/', name, '/sensors');
muscle_sub = rossubscriber(topic);
MP = muscle_param_mus01;

f = figure();
f.Position = [3000 800 800 400];
hold on;
set(f,'KeyPressFcn', @clear);

    while 1
        hold on;
        [l_CE, l_SEE, l_MTC] = read_muscle(muscle_sub, MP);
        %lower_joint_angle = joint_msg.Data;

        [F_MTC, dot_l_CE, F_elements] = mtu_model_matlab(l_CE, l_MTC, 0, 0.1, MP);
        plot(dot_l_CE, F_MTC, '.'); 
    end
end

function [l_CE, l_SEE, l_MTC] = read_muscle(muscle_sub, params)
muscle_msg = receive(muscle_sub);
l_CE =  params.CE.l_CEopt + muscle_msg.ContractileDisplacement * 0.006 * pi / 108544; % tansform from ticks to meters
l_SEE = params.SEE.l_SEE0 + muscle_msg.ElasticDisplacement / 66666.666; % from ticks to meters
l_MTC = l_CE + l_SEE;
end


function clear(fig_obj,eventDat)
clf(fig_obj);
end