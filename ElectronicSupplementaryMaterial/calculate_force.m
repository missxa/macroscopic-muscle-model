function calculate_force()

    if not(robotics.ros.internal.Global.isNodeActive)
        rosinit('localhost');
    end

    name = 'biceps';
    topic = strcat('/myo_blink/muscles/', name, '/sensors');
    muscle_sub = rossubscriber(topic);
    move_motor = rossvcclient('/myo_blink/move');
    request = rosmessage(move_motor);
    request.Action = 'keep';
    request.Muscle = name;
    MP = muscle_param_mus01;
    clf;
%     f = figure();
%     scrsz = get(groot,'ScreenSize');
    %figure('Position',[1 scrsz(4) scrsz(3) scrsz(4)])
%     f.Position = [2000 scrsz(4) scrsz(3) scrsz(4)];
    m = 2;
    n = 2;
    hold on;
%     set(f,'KeyPressFcn', @clear);
    setGlobalStep(0);

    wrestle_topic = '/myo_blink/wrestle/start';
    wrestle_sub = rossubscriber(wrestle_topic);
    receive(wrestle_sub);
    prev_l_CE = 0.09;
        while 1
            
            if (wrestle_sub.LatestMessage.Data == 0)
                request.Setpoint = 38;
                call(move_motor, request);
                break
            end
            
            hold on;
            [l_CE, delta_l_SEE, dot_l_CE_emp, dot_l_SE_emp] = read_muscle(muscle_sub, MP);
            
            %lower_joint_angle = joint_msg.Data;
            dot_l_MTC =  dot_l_CE_emp + dot_l_SE_emp;
            activation=0.8;%input('activation: ')
            [F_MTC, dot_l_CE, F_elements] = mtu_model_matlab(l_CE, dot_l_CE_emp, delta_l_SEE, activation, MP);
         
            if F_MTC < 38
                F_MTC_emp = 38;
            elseif    F_MTC > 99
                F_MTC_emp = 99; 
            else
                F_MTC_emp = F_MTC;
            end
            
%             if F_MTC > 99
%                     F_MTC_emp = 99; 
%             else        
%                 F_MTC_emp = F_MTC;
%             end
            request.Setpoint = F_MTC_emp;
            call(move_motor, request);
            
            if l_CE >= MP.PEE.l_PEE0
                l_PEE = l_CE-MP.PEE.l_PEE0;
            else
                l_PEE = MP.PEE.l_PEE0;
            end   
            
            % waiting for the requested force to be applied
%             while(1)
%                 if (abs(muscle_sub.LatestMessage.ElasticDisplacement*0.2 + 38 - F_MTC_emp) < 3)
%                     break
%                 end
%             end
            
            
%             %plot(dot_l_CE, F_MTC, '.'); 
%             subplot(m,n,1);
%             title('muscle model velocity');
%             %i_dot_l_CE = interp1(step, dot_l_CE);
%             plot(getGlobalStep, dot_l_CE, 'o');%, stepq, i_dot_l_CE, ':.');
% 
%             hold on;
% 
%             subplot(m,n,1);
%             title('emprical velocity');
%             plot(getGlobalStep, dot_l_CE, '*');
%             hold on;
% 
            subplot(m,n,3);
            title('l_P_E_E');
            plot(getGlobalStep, l_PEE, 'x');
            hold on;
% 
            subplot(m,n,2);
            title('F_M_T_C');
            plot(getGlobalStep, F_MTC,'o');
            hold on;
% 
            subplot(m,n,4);
            title('delta l_S_E_E');
            plot(getGlobalStep, delta_l_SEE, '*');
            hold on;
% 
%             subplot(m,n,2);
%             title('CE length');
%             plot(getGlobalStep, l_CE, '*');
%             hold on;
% 
            subplot(m,n,1);
            title('force-velocity');
            plot(dot_l_CE_emp,  F_MTC, 'x');
            hold on;

            setGlobalStep(getGlobalStep + 1);
%             pause(1);
            prev_l_CE = l_CE;
        end
end

function [l_CE, delta_l_SEE, dot_l_CE, dot_l_SE] = read_muscle(muscle_sub, params)
    muscle_msg = receive(muscle_sub);
    l_CE = params.CE.l_CEopt + muscle_msg.ContractileDisplacement * 0.006 * pi / 6.28319 % tansform from rad to meters
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