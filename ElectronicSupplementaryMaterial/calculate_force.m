function calculate_force()
%rosinit('localhost');
name = 'triceps';
topic = strcat('/myo_blink/muscles/', name, '/sensors');
muscle_sub = rossubscriber(topic);
MP = muscle_param_mus01;

f = figure();
f.Position = [3000 800 1600 900];
hold on;
setGlobalStep(0);
set(f,'KeyPressFcn', @clear);

    while 1
        hold on;
        [l_CE, delta_l_SEE, dot_l_CE_emp, dot_l_SE_emp] = read_muscle(muscle_sub, MP);
        %lower_joint_angle = joint_msg.Data;
        dot_l_MTC =  dot_l_CE_emp + dot_l_SE_emp;
        [F_MTC, dot_l_CE, F_elements] = mtu_model_matlab(l_CE, delta_l_SEE, dot_l_MTC, 1, MP);
        
        %plot(dot_l_CE, F_MTC, '.'); 
        subplot(5,1,1);
        title('muscle model velocity');
        %i_dot_l_CE = interp1(step, dot_l_CE);
        plot(getGlobalStep, dot_l_CE, 'o');%, stepq, i_dot_l_CE, ':.');
        
        hold on;
        
        subplot(5,1,2);
        title('emprical velocity');
        plot(getGlobalStep, dot_l_CE_emp, '*');
        hold on;
        
        subplot(5,1,3);
        title('CE force');
        plot(getGlobalStep,  F_elements(4), 'x');
        hold on;
        
        subplot(5,1,4);
        title('SEE force');
        plot(getGlobalStep, F_elements(1), '*');
        hold on;
        
        subplot(5,1,5);
        title('elastic displacement');
        plot(getGlobalStep, delta_l_SEE, '*');
        hold on;
       
        
        setGlobalStep(getGlobalStep + 1);
        %pause(0.1);
    end
end

function [l_CE, delta_l_SEE, dot_l_CE, dot_l_SE] = read_muscle(muscle_sub, params)
muscle_msg = receive(muscle_sub);
l_CE = params.CE.l_CEopt + muscle_msg.ContractileDisplacement * 0.006 * pi / 6.88319; % tansform from rad to meters
%l_SEE = params.SEE.l_SEE0 + muscle_msg.ElasticDisplacement / 66666.666; % from ticks to meters
delta_l_SEE = muscle_msg.ElasticDisplacement / 66666.666;
dot_l_CE = muscle_msg.ActuatorVel * 0.006 * pi / 6.28319; % from rad/s to m/s
dot_l_SE = muscle_msg.ElasticVel; % m/s
%l_MTC = l_CE + l_SEE;
end


function clear(fig_obj,eventDat)
if eventDat.Key == 'p'
    waitforbuttonpress;
else
    clf(fig_obj);
    setGlobalStep(0);
end
end

function setGlobalStep(val)
global step
step = val;
end

function r = getGlobalStep
global step
r = step;
end