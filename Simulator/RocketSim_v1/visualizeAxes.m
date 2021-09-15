function P = visualizeAxes(S)
% visualizeAxes : Takes in an input structure S and visualizes the resulting
%                 body-axis 3D motion in approximately real-time.  Outputs 
%                 the data used to form the plot.
%                 
%
% INPUTS
%
% S ---------- Structure with the following elements:
%
%           eMat = Mx3 matrix of rocket attitudes, in radians
%
%           tVec = Mx1 vector of times, in seconds
%
%         bounds = 6x1, the 3d axis size vector
%
%         tStep = simulation time increment, seconds 
%
%
% OUTPUTS
%
% P ---------- Structure with the following elements:
%
%          x = Nx3 matrix of body x-axis unit vectors, in inertial frame
%
%          y = Nx3 matrix of body y-axis unit vectors, in inertial frame
%
%          z = Nx3 matrix of body z-axis unit vectors, in inertial frame
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  Francis Lara
%+==============================================================================+
prompt = 'Run Body-Axis Visualization? Type Y or N and press enter/return: ';
str = input(prompt, 's');
if or(str == 'Y', str == 'y')
    disp('Running Visualization...')
    disp('Press Ctrl + C to cancel')
    % Body-Axis Visualization
    for i = 1:length(S.eMat)
        RBI = euler2dcm(S.eMat(i,:));
        P.x(i,:) = RBI(1,:);
        P.y(i,:) = RBI(2,:);
        P.z(i,:) = RBI(3,:);
    end
    figure(5)
    % Initial body axis plot
    % x-axis = red, y-axis = blue, z-axis = green
    h1 = plot3([0 P.x(1,1)], [0 P.x(1,2)], [0 P.x(1,3)], 'r');
    hold on
    h2 = plot3([0 P.y(1,1)], [0 P.y(1,2)], [0 P.y(1,3)], 'b');
    h3 = plot3([0 P.z(1,1)], [0 P.z(1,2)], [0 P.z(1,3)], 'g');
    hold off
    grid on; box on
    % Setting graph limits
    axis(S.bounds)
    % Changing body axis unit vectors
    for i = 1:length(P.x)
           tic
           set(h1, 'XData', [0 P.x(i,1)], 'YData', [0 P.x(i,2)], 'ZData', [0 P.x(i,3)]);
           set(h2, 'XData', [0 P.y(i,1)], 'YData', [0 P.y(i,2)], 'ZData', [0 P.y(i,3)]);
           set(h3, 'XData', [0 P.z(i,1)], 'YData', [0 P.z(i,2)], 'ZData', [0 P.z(i,3)]);
           if floor(S.tVec(i))==S.tVec(i)
               title(['t = ', num2str(S.tVec(i)), ' sec'])
           end
           drawnow
           dt = toc;
           pause(max((S.tStep)-dt,0))
    end
    title(['t = ', num2str(S.tVec(i)), ' sec'])
    disp('Visualization complete!')
elseif or(str == 'N', str == 'n')
    disp('Cancelling visualization')
    P = [];
    return
else
    disp('Error! Wrong input')
    P = [];
end
end