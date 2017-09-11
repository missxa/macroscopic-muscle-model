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
    clf;
%     f = figure();
%     scrsz = get(groot,'ScreenSize');
    %figure('Position',[1 scrsz(4) scrsz(3) scrsz(4)])
%     f.Position = [2000 scrsz(4) scrsz(3) scrsz(4)];
    m = 1;
    n = 1;
    hold on;
%     set(f,'KeyPressFcn', @clear);
    setGlobalStep(0);

    wrestle_topic = '/myo_blink/wrestle/start';
    wrestle_sub = rossubscriber(wrestle_topic);
    activ_sub = rossubscriber('/activation');
    
    proprio_topic = strcat('/myo_blink/muscles/', name, '/afferents/discharge/rate');
    proprio_pub = rospublisher(proprio_topic, 'myo_blink/afferents');
    f = {};
    l = {};
    Ia = {};
    II = {};
    l = {};
    
    receive(wrestle_sub);
    tic
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
            
            activation=activ_sub.LatestMessage.Data;
            if(activation < 0)
                activation = 0;
            end
            [F_MTC, dot_l_CE, F_elements] = mtu_model_matlab(l_CE, dot_l_CE_emp, delta_l_SEE, activation, MP);
            
            F_MTC_emp = F_MTC + 38;
            if F_MTC_emp < 38
                F_MTC_emp = 38;
            elseif    F_MTC_emp > 99
                F_MTC_emp = 99; 
            end
            F_MTC
            
            request.Setpoint = F_MTC_emp;
            call(move_motor, request);
            
            if l_CE >= MP.PEE.l_PEE0
                l_PEE = l_CE-MP.PEE.l_PEE0;
            else
                l_PEE = MP.PEE.l_PEE0;
            end   
            
            proprio_msg = rosmessage(proprio_pub);
            proprio_msg.Ia = delta_l_SEE;
            proprio_msg.II = l_PEE;
            send(proprio_pub, proprio_msg);
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
            II = [II, l_PEE];
%             subplot(m,n,4);
%             title('l_P_E_E');
%             plot(getGlobalStep, l_PEE, 'x');
%             hold on;
% % 
%             subplot(m,n,2);
%             title('F_M_T_C');
%             plot(getGlobalStep, F_MTC,'o');
%             hold on;
% % 
%             Ia = [Ia, delta_l_SEE];
%             subplot(m,n,3);
%             title('delta l_S_E_E');
%             plot(getGlobalStep, delta_l_SEE, '*');
%             hold on;
% 
%             l_MTC = l_CE + delta_l_SEE;
%             l = [l,l_MTC];
%             subplot(m,n,2);
%             title('MTC length');
%             plot(getGlobalStep, l_MTC, '*');
%             hold on;
%               
            f = [f, F_elements(2)];
            l = [l, l_CE];
            subplot(m,n,1);
            title('Concentric contraction (shortening)');
            xlabel('CE velocity [m/s]');
            ylabel('CE force [N]');
%             plot(dot_l_CE_emp,  F_elements(), 'x');
%             
%             subplot(m,n,1);
%             title('Force-length=pee');
%             xlabel('length');
%             ylabel('force [N]');
            
            plot(l_CE,  F_elements(2)/90, 'x');
            hold on;

            setGlobalStep(getGlobalStep + 1);
        end
        toc
        f = cell2mat(f);
        l = cell2mat(l);
        out = [double(l') double(f')];
        save('../../msc-thesis/graphics/fl-pee.txt', 'out',   '-ASCII' );
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