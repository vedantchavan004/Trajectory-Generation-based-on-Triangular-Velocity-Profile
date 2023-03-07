%% Visualization of a Planar RP Robot (first joint revolute, second joint prismatic) 
%% Trajectory Generation based on Triangular Profile
%*******************************************

clear all % clear workspace
close all % close all plots / figures
clc % clear command window 


%% Structure of this file
% 1 - calculation of profile parameters (= planning)
% 2 - calculation of position q(t) AND p(t) for every sample point (= interpolation)
% 3 - calculation of joint angles/positions theta1(t), d from inverse 
%     kinematic solution 
% 4 - animation of 2D movement and plots 
% 
% Note: the animation can be stored as a video
%       saveAnimation = 0; --> no video created and stored
%       saveAnimation = 1; --> video created and stored in workspace as *.avi
saveAnimation = 0;

% *******************************************
%% Robot Parameters
% *******************************************
L1 = 1; % link lenght in m
dmax = 0.8; % maximum value for prismatic joint reach
L3 = 0.3; % link lenght in m

% *******************************************
%% Part 1 - Calculation of parameters necessary for triang. profile
% *******************************************
%% Input paramters to the planning problem:
t0 = 3;  % Starting time 0sec 

% Some sets of start / end points for testing:
p0 = [1.1 , 0.3]'; % Stat position of TCP in xy-coordinates
p1 = [1.1 , 1]'; % End position of TCP in xy-coordintes

%% Check whether start and end point are within reach of robot
% Once you are able to generate a trajectory and everything is working fine
% here as a last (!) part of this exercise some lines of code which check
% whether the start point and the end point is reachable by the robot can be added. 
% Hint: You can use the related section from the planar 2-arm robot example from
% lecture here as a template.


%% Distance from p0 to p1
qDist = sqrt((p0(1)-p1(1))^2+(p0(2)-p1(2))^2);

%% Parameters for the triangular vel. profile q(t) on the straight line
q0 = 0;  % Starting position in m
q1 = qDist;  % End position in m
aa = 0.2;% Acceleration level in m/sec^2 in the direction of the line

%% Calculation of profile parameters (i.e. switching times) for triangular profile
DeltaQ = q1-q0; % overall travel distance
t1 = t0+ (2*sqrt((DeltaQ)/aa)); % end time
ta = (t0+t1)/2; % switching time from accel. to decel. phase


%% Calculation of switching velocity related to switching time ta 
%  --> maximum velocity within the movement: v(ta)
a2=aa/2;
a1=-aa*t0;
a0=q0 +(aa/2)*(t0^2);

c2=-aa/2;
c1=aa*t1;
c0=q1 -(aa/2)*(t1^2);
vta = 2*a2*ta+a1;
vMax=vta;

%% To save the animation - create video
if (saveAnimation == 1) % ONLY if animation shall be saved as video
myVideo = VideoWriter('MyAnimationTriangVel_2ArmRobot_2D'); %create video file
myVideo.FrameRate = 5;  %5 - 10 works well typically
open(myVideo)
end

% *******************************************
%% Parts 2 (interpolation), 3 (inv. kin. sol.) & 3 (animation) for every sample point in time 
% *******************************************
% Parameters 
DeltaT = 0.05; % Sampling time for trajectory in seconds

%% Evaluation / interpolation for each time step
% (here in a FOR loop as we are not working on a real-time system)
% Variables:
% - Variable tAct (= t in the equations) is the current time for the evaluation of the profile
% - Variable qtAct (= q(t)) is the current position on the line at time tAct
% - Variable ptAct (= p(t)) is the current position of the TCP in xy-coordinates 
% - Variable theta1Act (= theta1(t)) is the joint angle at time tAct, 
%            dAct (= d(t)) is the current position of the prismatic joint
%   
for tAct = t0:DeltaT:t1
    
    %%Interpolation 
    %%Evaluation of triangular vel. profile 
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
    
    %%Calculation of position p(t) in xy-coordinates 
    % Calculate current position p(t) at current time tAct
    % Based on equations from lecture:
    %b=(L3)/sin(delta)
    %ptAct = [b*cos(theta1+ delta),b*sin(theta1+ delta)];
%     bsq= pxAct^2+pyAct^2
%     d=sqrtr(bsq-(L3)^2)-L1
    %d=sqrt(L1^2-L3^2)-L1
    %pAct=[(L1+d)*cos(theta1)-(L3*sin(theta1)),(L1+d)*sin(theta1)+(L3*cos(theta1))]
    ptAct=p0+(qtAct/qDist)*(p1-p0)
    %%
    %%************************************
    %% Inverse kinematic solution
    pxAct = ptAct(1); % Current Cartesian position of the TCP in x-direction
    pyAct = ptAct(2); % Current Cartesian position of the TCP in y-direction
    % Position of prismatic joint    
    bsq= pxAct^2+pyAct^2
    d=sqrt(bsq-(L3)^2)-L1
    dAct = d;
    % Angle of first joint theta1Act:
    delta=atan2(L3,L1+dAct) 
    theta1= atan2(pyAct,pxAct)-delta
    theta1Act = theta1;
    
    
    %%Animation (NO NEED FOR CHANGES BELOW)
    % Draw current postion of TCP (shape: "red diamond = rd") at time tAct
    % Generate graphics showing actual position of the "box"
    figure(1)
    subplot(4,1,1) % to plot 4 plots in one figure
    plot(ptAct(1),ptAct(2),'rd','Linewidth',2) % plot red diamond showing the center of the lin. motor slide
    hold on 
    xlim([p0(1)-0.5,p1(1)+0.5]) % limit x-axis of plot
    ylim([p0(2)-0.5, p1(2)+0.5])% limit y-axis of plot
    grid on % activate grid in plot
    xlabel('Position p_x(t) [m]')
    ylabel('Position p_y(t) [m]')
    title('Animation of Triangular Velocity Profile')
    drawnow % better animation and limitation to 20 updates per second of the plot
    
    %% Visualization of profile q(t), dot q(t), ddot q(t)
    % Collect all values q, dq, ddq over time until the current time tAct
    k = round(tAct / DeltaT)+1; % number of current sample +1
    tColl(k)=tAct; % collect time values in vector tColl
    qColl(k)=qtAct; % collect position values in vector qColl
    dotQColl(k)=dotQtAct; % collect vel. values in vector dotQColl
    ddotQColl(k)=ddotQtAct;% collect accel. values in vector ddotQColl
    
    %% Plot Animation of q, dq, ddq
    % Position plot
    subplot(4,1,2)
    plot(tColl, qColl,'Linewidth',1.5)
    title('Motion Profile q(t) along line')
    xlim([t0,t1])
    ylim([q0-0.5,q1+0.5])
    xlabel('Time t in sec')
    ylabel('Position in m')
    grid on
    
    % Velocity plot
    subplot(4,1,3)
    plot(tColl, dotQColl,'Linewidth',1.5)
    xlim([t0,t1])
    ylim([0,1.1*vMax])
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
    
    %% Larger separated animation in 2D
    figure(2)
    plot(ptAct(1),ptAct(2),'rd','Linewidth',2) % plot red diamond showing the center of the lin. motor slide
    hold on 
    % plot line for first link from origin to end of 1st link
    plot([0,L1*cos(theta1Act)],[0,L1*sin(theta1Act)],'b','Linewidth',1.5)
    % plot line for second link from end of 1st link to end of second link
    pEndLink1 = [L1*cos(theta1Act),L1*sin(theta1Act)]';
    pEndLink2 = [(L1+dAct)*cos(theta1Act), (L1+dAct)*sin(theta1Act)]';
    plot([pEndLink1(1),pEndLink2(1)],[pEndLink1(2),pEndLink2(2)],'g','Linewidth',1.5)
    % plot line for 3rd "link" (fixed length L3) from end of 2nd link to TCP
    pEndLink2 = [(L1+dAct)*cos(theta1Act), (L1+dAct)*sin(theta1Act)]';
    pTCP_fwd  = [(L1+dAct)*cos(theta1Act)-L3*sin(theta1Act),...
                 (L1+dAct)*sin(theta1Act)+L3*cos(theta1Act)]';
    plot([pEndLink2(1),pTCP_fwd(1)],[pEndLink2(2),pTCP_fwd(2)],'k','Linewidth',1.5)
    
    
    % Straight line from p0 to p1 in magenta, dashed:
    plot([p0(1),p1(1)],[p0(2),p1(2)],'--m')
    hold off
    xlim([-L1-L1,L1+L1]) % limit x-axis of plot
    xlim([-L1-L1,L1+L1])% limit y-axis of plot
    axis equal
    grid on % activate grid in plot
    xlabel('Position p_x(t) [m]')
    ylabel('Position p_y(t) [m]')
    title('Animation of Triangular Velocity Profile')
    drawnow
    
    %% Visualization of trajectories for the position p_x(t), p_y(t)
    % Collect all values over time until the current time tAct
    k = round(tAct / DeltaT)+1; % number of current sample +1
    tColl(k)=tAct; % collect time values in vector tColl
    pXColl(k)=ptAct(1); % collect desired pos. x direction
    pYColl(k)=ptAct(2); % collect desired pos. y direction
    
    %% Visualization of trajectories for the position theta1(t), d(t)
    % Collect all values over time until the current time tAct
    k = round(tAct / DeltaT)+1; % number of current sample +1
    tColl(k)=tAct; % collect time values in vector tColl
    theta1Coll(k) = theta1Act; % collect desired angle joint 1 theta1(t)
    dColl(k) = dAct; % collect desired pos. joint 2 d(t)
    
    %% Write video
    if (saveAnimation == 1) % ONLY if animation shall be saved as video
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
    end
end

%% Close video
if (saveAnimation == 1) % ONLY if animation shall be saved as video
close(myVideo)
end

%% Plot Cartesian Trajectories for xy
    % Position plots
    % x-position
    figure
    subplot(3,1,1)
    plot(tColl, pXColl,'Linewidth',1.5)
    title('Desired Pos. Trajectory p_x(t) for x-drive')
    xlim([t0, t1]) % Limits of time axis
    ylim([p0(1)-0.5,p1(1)+0.5]) % Limits of position in x-direction
    xlabel('Time t in sec')
    ylabel('Position in m')
    grid on

    % y-position
    subplot(3,1,2)
    plot(tColl, pYColl,'Linewidth',1.5)
    title('Desired Pos. Trajectory p_y(t) for y-drive')
    xlim([t0, t1])    % Limits of time axis
    ylim([p0(2)-0.5,p1(2)+0.5])% Limits of position in y-direction
    xlabel('Time t in sec')
    ylabel('Position in m')
    grid on
    
    % xy-position
    subplot(3,1,3)
    plot(pXColl, pYColl,'Linewidth',1.5)
    title('Desired Pos. Trajectory p(t) in xy-Plane')
    xlim([p0(1)-0.5,p1(1)+0.5])% Limits of position in x-direction
    ylim([p0(2)-0.5,p1(2)+0.5])% Limits of position in y-direction
    xlabel('X-Position in m')
    ylabel('Y-Position in m')
    grid on
    
    %% Plot Cartesian Trajectories for xy
    % Position plots
    % theta 1
    figure
    subplot(2,1,1)
    plot(tColl, theta1Coll *180/pi,'Linewidth',1.5)
    xlim([t0, t1]) % Limits of time axis
    xlabel('Time t in sec')
    ylabel('Angle in deg')
    title('Trajectory for revolute joint 1 (theta1)')
    grid on

    % d
    subplot(2,1,2)
    plot(tColl, dColl,'Linewidth',1.5)
    xlim([t0, t1]) % Limits of time axis
    xlabel('Time t in sec')
    ylabel('Position in m')
    title('Trajectory for prismatic joint 2')
    grid on
    
    