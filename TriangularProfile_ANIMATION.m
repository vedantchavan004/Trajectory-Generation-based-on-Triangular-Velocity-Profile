
%% Visualization of a 1D Trajectory  
%% Trajectory Generation based on a Triangular Profile


% *******************************************
clear all % clear workspace
close all % close all plots / figures
clc % clear command window 

%% Structure of this file
% 1 - calculation of profile parameters (= planning)
% 2 - calculation of position q(t) for every sample point (= interpolation)
% 3 - animation of 1D movement and plot of motion profile q(t), dq(t), ddq(t) 
% 
% Note: the animation can be stored as a video
%       saveAnimation = 0; --> no video created and stored
%       saveAnimation = 1; --> video created and stored in workspace as *.avi
saveAnimation = 0;


% *******************************************
%% Calculation of parameters necessary for triang. profile
% *******************************************
%% Input paramters to the planning problem:
t0 = 2;  % Starting time in sec (not necessarily equal to zero now)

% Parameters for the motion profile q(t)
q0 = 0.5;  % Starting position in m
q1 = 2;  % End position in m
aa = 0.2;% Acceleration level in m/sec^2 

%% Calculation of profile parameters (i.e. switching times)
DeltaQ = q1-q0;
t1 = t0+ (2*sqrt((DeltaQ)/aa));
ta = (t0+t1)/2;


a2=aa/2;
a1=-aa*t0;
a0=q0 +(aa/2)*(t0^2);

 c2=-aa/2;
 c1=aa*t1;
 c0=q1 -(aa/2)*(t1^2);

%% Calculation of switching position related to switching time ta
% (distances from start positon p0 in the direction of the line)
% for visualization only
% i.e. calculate the position q(ta) at t = ta here:
qta = a2*ta^2+a1*ta+a0;

%% Calculation of switching velocity related to switching time ta
% i.e. calculate the velocity v(ta) at t = ta here:
vta = 2*a2*ta+a1;

% *******************************************
% Definition of variables for animation (DO NOT CHANGE)
% *******************************************
% Define rectangle representing linear motor for animation
al=0.1; % half length of slide 
% Coordiantes of edges for the box representing lin. motor
% (x-position in first line, y-position in second line)
Q0=[al -al -al  al
    al  al -al -al ];

%% To save the animation - create video
if (saveAnimation == 1) % ONLY if animation shall be saved as video
myVideo = VideoWriter('MyAnimationTriangularProfile_2D'); %create video file
myVideo.FrameRate = 5;  %5 - 10 works well typically
open(myVideo)
end

% *******************************************
%% Parts 2 (interpolation) & 3 (animation) for every sample point in time 
% *******************************************
% Parameters 
DeltaT = 0.05; % Sampling time for trajectory in seconds

%% Evaluation / interpolation for each time step
% (here in a FOR loop as we are not working on a real-time system)
% Variables:
% - Variable tAct (= t in the equations) is the current time for the evaluation of the profile
% - Variable qtAct (= q(t)) is the current position on the line at time tAct
    

for tAct = t0:DeltaT:t1
    
    %%Interpolation 
    %% Evaluation of triangular vel. profile 
    % Calculate current position q at current time tAct
   
    
    % Based on derived equations:
    if ( tAct<ta )    % tAct is in SECTION 1 (acceleration)
    qtAct = a2*(tAct^2)+ a1*tAct+a0;  % Current position q(t)
    dotQtAct = 2*a2*tAct+ a1; % Current velocity dot q(t)
    ddotQtAct = 2*a2; % Current acceleration ddot q(t)
    
    elseif ( tAct>ta ) % tAct is in SECTION 2 (deceleration)
    qtAct = c2*(tAct^2)+ c1*tAct+c0;  % Current position q(t)
    dotQtAct = 2*c2*tAct+ c1; % Current velocity dot q(t)
    ddotQtAct = 2*c2; % Current acceleration ddot q(t)
    end
    
    %% Animation (NO CHANGES NEEDED BELOW)
    %% Draw current postion of linear motor ("box") at time tAct

    % Calculate actual positon of all edges of the "box" shifted together
    % with the center of the box
    currentPos = qtAct;
    Qx = Q0(1,:) + currentPos; % x-position of box is moving
    Qy = Q0(2,:); % y-position of box is not changing
    
    % Generate graphics showing actual position of the "box"
    subplot(4,1,1) % to plot 4 plots in one figure
    plot([Qx,Qx(1)],[Qy,Qy(1)],'Linewidth',10)
    hold on
    plot(qtAct,0,'rd','Linewidth',2) % plot red diamond showing the center of the lin. motor slide
    hold on
    plot([qta qta],[+0.2 -0.2],'--r') % red lines showing switching position qta
    hold off
    xlim([q0-0.5,q1+0.5]) % limit x-axis of plot
    ylim([-0.5, +0.5])% limit y-axis of plot
    grid on % activate grid in plot
    xlabel('Position p_x(t) [m]')
    ylabel('Position p_y(t) [m]')
    title('Animation of Triangular Velocity Profile')
    drawnow % better animation and limitation to 20 updates per second of the plot
    
    %% Visualization of profile q(t), dot q(t), ddot q(t)
    % Collect all values q, dq, ddq over time until the current time tAct
    k = round(tAct / DeltaT)+1; % number of current sample +1
    tColl(k)=tAct;
    qColl(k)=qtAct;
    dotQColl(k)=dotQtAct;
    ddotQColl(k)=ddotQtAct;
    
    %% Plot Animation of q, dq, ddq
    % Position plot
    subplot(4,1,2)
    plot(tColl, qColl,'Linewidth',1.5)
    title('Motion Profile q(t)')
    xlim([t0,t1])
    ylim([q0-0.5,q1+0.5])
    xlabel('Time t in sec')
    ylabel('Position in m')
    grid on
    
    % Velocity plot
    subplot(4,1,3)
    plot(tColl, dotQColl,'Linewidth',1.5)
    xlim([t0,t1])
    ylim([0,1.1*vta])
    xlabel('Time t in sec')
    ylabel('Vel. in m/sec')
    grid on
    
    % Acceleration plot
    subplot(4,1,4)
    plot(tColl, ddotQColl,'Linewidth',1.5)
    xlim([t0,t1])
    ylim([-aa,+aa])
    xlabel('Time t in sec')
    ylabel('Accel. in m/sec^2')
    grid on
    
    %% write video
    if (saveAnimation == 1) % ONLY if animation shall be saved as video
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    end
    
end

%% close video
if (saveAnimation == 1) % ONLY if animation shall be saved as video
close(myVideo)
end

