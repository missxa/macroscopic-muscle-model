function calculate_force()

    if not(robotics.ros.internal.Global.isNodeActive)
        rosinit('localhost');
    end

    name = 'triceps';
    topic = strcat('/myo_blink/muscles/', name, '/sensors');
    muscle_sub = rossubscriber(topic);
    move_motor = rossvcclient('/myo_blink/move');
    request = rosmessage(move_motor);
    request.Action = 'keep';
    request.Muscle = name;
    MP = muscle_param_mus01;

    f = figure();
    scrsz = get(groot,'ScreenSize');
    %figure('Position',[1 scrsz(4) scrsz(3) scrsz(4)])
    f.Position = [2000 scrsz(4) scrsz(3) scrsz(4)];
    m = 1;
    n = 1;
    hold on;
    set(f,'KeyPressFcn', @clear);
    setGlobalStep(0);

        while 1
            hold on;
            [l_CE, delta_l_SEE, dot_l_CE_emp, dot_l_SE_emp] = read_muscle(muscle_sub, MP);
            %lower_joint_angle = joint_msg.Data;
            dot_l_MTC =  dot_l_CE_emp + dot_l_SE_emp;
            [F_MTC, dot_l_CE, F_elements] = mtu_model_matlab(l_CE, dot_l_CE_emp, delta_l_SEE, 1, MP);
         
            if F_MTC < 38
                F_MTC_emp = 38;
            else
                F_MTC_emp = F_MTC;
            end
            request.Setpoint = F_MTC_emp;
            call(move_motor, request);
            
%             %plot(dot_l_CE, F_MTC, '.'); 
%             subplot(m,n,1);
%             title('muscle model velocity');
%             %i_dot_l_CE = interp1(step, dot_l_CE);
%             plot(getGlobalStep, dot_l_CE, 'o');%, stepq, i_dot_l_CE, ':.');
% 
%             hold on;
% 
%             subplot(m,n,2);
%             title('emprical velocity');
%             plot(getGlobalStep, dot_l_CE_emp, '*');
%             hold on;
% 
%             subplot(m,n,3);
%             title('CE force');
%             plot(getGlobalStep,  F_elements(4), 'x');
%             hold on;
% 
%             subplot(m,n,4);
%             title('SEE force');
%             plot(getGlobalStep, F_elements(1), '*');
%             hold on;
% 
%             subplot(m,n,5);
%             title('PEE force');
%             plot(getGlobalStep, F_elements(2), '*');
%             hold on;
% 
%             subplot(m,n,6);
%             title('CE length');
%             plot(getGlobalStep, l_CE, '*');
%             hold on;
% 
            subplot(m,n,1);
            title('force-velocity');
            plot(dot_l_CE_emp,  F_MTC, 'x');
            %axis([-0.01 0.01 0 1000])
            hold on;

            setGlobalStep(getGlobalStep + 1);
            %pause(0.1);
        end
end

function [l_CE, delta_l_SEE, dot_l_CE, dot_l_SE] = read_muscle(muscle_sub, params)
muscle_msg = receive(muscle_sub);
l_CE = params.CE.l_CEopt + muscle_msg.ContractileDisplacement * 0.006 * pi / 6.28319; % tansform from rad to meters
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