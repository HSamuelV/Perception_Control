function Draw_MPC_point_stabilization_obs (t,xx,xx1,u_cl,xs,N,rob_length,rob_width,obs_x,obs_y,obs_diam)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
x_r_1 = [];
y_r_1 = [];

r = obs_diam/2;  % obstacle radius
ang=0:0.005:2*pi;
xp_obs=r*cos(ang);
yp_obs=r*sin(ang);

figure(500)
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

for k = 1:size(xx,2)
    w_t = rob_length; l_t=rob_width; % robot parameter
    
    % Extracting the reference state variables
    x1 = xs(1); y1 = xs(2); th1 = xs(3);

    % Compute the vertices of the reference rectangle
    half_wt = w_t / 2;
    half_ht = l_t / 2;

    x1_rect = [x1 + half_wt * cos(th1) - half_ht * sin(th1), ...
        x1 - half_wt * cos(th1) - half_ht * sin(th1), ...
        x1 - half_wt * cos(th1) + half_ht * sin(th1), ...
        x1 + half_wt * cos(th1) + half_ht * sin(th1)];

    y1_rect = [y1 + half_wt * sin(th1) + half_ht * cos(th1), ...
        y1 - half_wt * sin(th1) + half_ht * cos(th1), ...
        y1 - half_wt * sin(th1) - half_ht * cos(th1), ...
        y1 + half_wt * sin(th1) - half_ht * cos(th1)];

    % Plot the reference state as a rectangle
    fill(x1_rect, y1_rect, 'g'); % plot reference state
    hold on;

    % Extracting the actual state variables
    x1 = xx(1,k,1); y1 = xx(2,k,1); th1 = xx(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];

    % Compute the vertices of the actual rectangle
    x1_rect = [x1 + half_wt * cos(th1) - half_ht * sin(th1), ...
        x1 - half_wt * cos(th1) - half_ht * sin(th1), ...
        x1 - half_wt * cos(th1) + half_ht * sin(th1), ...
        x1 + half_wt * cos(th1) + half_ht * sin(th1)];

    y1_rect = [y1 + half_wt * sin(th1) + half_ht * cos(th1), ...
        y1 - half_wt * sin(th1) + half_ht * cos(th1), ...
        y1 - half_wt * sin(th1) - half_ht * cos(th1), ...
        y1 + half_wt * sin(th1) - half_ht * cos(th1)];

    % Plot the actual trajectory as a rectangle
    plot(x_r_1, y_r_1, '-r', 'linewidth', line_width); % plot exhibited trajectory
    
    if k < size(xx,2) % plot prediction
        plot(xx1(1:N,1,k),xx1(1:N,2,k),'r--*')
    end
    fill(x1_rect, y1_rect, 'r'); % plot actual state

    plot(obs_x+xp_obs,obs_y+yp_obs,'--b'); % plot robot circle  
   
    hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-1.5 10 -1.5 10])
    pause(0.1)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
end
%close(gcf)
%viobj = close(aviobj)
%video = VideoWriter('exp.avi','Uncompressed AVI');

% video = VideoWriter('exp.avi','Motion JPEG AVI');
% video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
% open(video)
% writeVideo(video,F)
% close (video)
 

figure
subplot(211)
stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -1.1 1.1])
xlabel('time (seconds)')
ylabel('v (rad/s)')
grid on
subplot(212)
stairs(t,u_cl(:,2),'r','linewidth',1.5); axis([0 t(end) -1.5 4.0])
xlabel('time (seconds)')
ylabel('\omega (rad/s)')
grid on
