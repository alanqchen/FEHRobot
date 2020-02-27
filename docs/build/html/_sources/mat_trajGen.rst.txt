Trajectory Profile Generation
=============================

.. code-block:: matlab
   :linenos:

   OFFSET = 0;
    MOTOR_ANGLE_1 = 0*pi/180+OFFSET;
    MOTOR_ANGLE_2 = 120*pi/180+OFFSET;
    MOTOR_ANGLE_3 = 240*pi/180+OFFSET;
    THETA = pi/6;
    MAX_RPM = 236.5398195;
    %R = 0.11176;
    %R = 0.1143;
    %R = 0.087;
    R = 0.0869;
    %R = 0.089284;
    %r = 0.03662751;
    %r = 0.035334;
    %r =  0.027534;
    r = 0.027;
    %r = 0.021;
    %r = 0.021; 
    DELTA_T = .1;

    Kp = 20;
    Ki = 0;

    FILE_NAME = 'CorrX.txt';

    % Waypoints
    %wpts = [0 0; 0 0.4064; THETA THETA];
    % TO JUKEBOX LIGHT
    %wpts = [0 0.4318; 0 -0.1397; THETA THETA];
    % TO RED JUKEBOX BUTTON
    %wpts = [0 0.0254 0.0254 0.0254 0; 0 0 0.13335 0 0; THETA THETA THETA THETA THETA];
    % TO BLUE JUKEBOX BUTTON
    %wpts = [0 -0.0508  -0.0508; 0 0 0.13335; THETA THETA THETA];
    % START
    %wpts = [0 0.2667; 0 0.22987; THETA THETA];
    % SINK
    %wpts = [0 -0.254; 0 -0.1905; THETA THETA];
    %wpts = [0 0.31242; 0 0.2667; THETA THETA];
    %wpts = [0 -0.2286; 0 -0.1651; THETA THETA];
    %wpts = [0 0; 0 .3; THETA THETA];
    % Rotate 90 CC
    %wpts = [0 0; 0 0; THETA THETA+pi/2];
    % Rotate 90 Clockwise
    %wpts = [0 0; 0 0; THETA THETA-pi/2];
    % To slide
    %wpts = [0 0.635; 0 0.1016; THETA THETA];
    % Up ramp 2
    %wpts = [0 0.9144; 0 0; THETA THETA];
    % Turn pi/6
    %wpts = [0 0; 0 0; THETA THETA+pi/6];
    % SLIDE TICKET 4 in
    %wpts = [0 0.127; 0 0; THETA THETA];
    % TO LEVER
    %
    % Small sidestep
    %wpts = [0 0.009; 0 0; THETA THETA];
    % CORRECT X (RPS)
    wpts = [0 0.0127; 0 0; THETA THETA];
    % CORRECT X (RPS) NEGATIVE
    %wpts = [0 -0.0127; 0 0; THETA THETA];

    % Timestamps for waypoints
    tpts = [0, .2];

    % Time update rate
    tvec = 0:0.1:.2;

    % Calc Trajectory Profile (cubic)
    [q, qd, qdd, pp] = cubicpolytraj(wpts, tpts, tvec);
    %[q, qd, qdd, pp] = trapveltraj(wpts, 31);

    % Plot positions from traj profile
    figure(1)
    plot(tvec, q)
    hold all
    plot(tpts, wpts, 'x')
    xlabel('t')
    ylabel('Positions')
    legend('X-positions','Y-positions', 'Theta')
    title('Robot Position')
    hold off

    % Plot velocities from traj profile
    figure(2)
    plot(tvec, qd)
    legend('X-vel','Y-vel', 'Phi')
    figure(3)
    plot(tvec, qdd)
    legend('X-Accel','Y-Accel', 'Phi-Accel')
    title('Robot Velocity')

    % Kinematic Relationships to turn into wheel angular velocities
    phiVel1 = (-sin(q(3,:)+MOTOR_ANGLE_1).*cos(q(3,:)).*qd(1,:)+cos(q(3,:)+MOTOR_ANGLE_1).*cos(q(3,:)).*qd(2,:)+R.*qd(3,:))/r;
    phiVel2 = (-sin(q(3,:)+MOTOR_ANGLE_2).*cos(q(3,:)).*qd(1,:)+cos(q(3,:)+MOTOR_ANGLE_2).*cos(q(3,:)).*qd(2,:)+R.*qd(3,:))/r;
    phiVel3 = (-sin(q(3,:)+MOTOR_ANGLE_3).*cos(q(3,:)).*qd(1,:)+cos(q(3,:)+MOTOR_ANGLE_3).*cos(q(3,:)).*qd(2,:)+R.*qd(3,:))/r;
    figure(4)
    plot(tvec, phiVel1, tvec, phiVel2, tvec, phiVel3);
    legend('Motor 1 Vel', 'Motor 2 Vel', 'Motor 3 Vel')
    title('Motor Velocities')

    % Numerical Integration to turn angular vel into angular positions
    phiRef1(1) = 0;
    phiRef2(1) = 0;
    phiRef3(1) = 0;
    for i=2:length(phiVel1)
        phiRef1(i)= phiRef1(i-1)+(DELTA_T/2)*(phiVel1(i)+phiVel1(i-1));
        phiRef2(i)= phiRef2(i-1)+(DELTA_T/2)*(phiVel2(i)+phiVel2(i-1));
        phiRef3(i)= phiRef3(i-1)+(DELTA_T/2)*(phiVel3(i)+phiVel3(i-1));
    end

    figure(5)
    plot(tvec, phiRef1, tvec, phiRef2, tvec, phiRef3);
    legend('Motor 1 Ang Pos', 'Motor 2 Ang Pos', 'Motor 3 Ang Pos')
    title('Angular Position')

    % Numerical Integration to turn angular vel into angular positions
    phiRef1(1) = 0;
    phiRef2(1) = 0;
    phiRef3(1) = 0;
    for i=2:length(phiVel1)
        phiRef1(i)= phiRef1(i-1)+abs((DELTA_T/2)*(phiVel1(i)+phiVel1(i-1)));
        phiRef2(i)= phiRef2(i-1)+abs((DELTA_T/2)*(phiVel2(i)+phiVel2(i-1)));
        phiRef3(i)= phiRef3(i-1)+abs((DELTA_T/2)*(phiVel3(i)+phiVel3(i-1)));
    end

    figure(6)
    plot(tvec, phiRef1, tvec, phiRef2, tvec, phiRef3);
    legend('Motor 1 Ang Pos', 'Motor 2 Ang Pos', 'Motor 3 Ang Pos')
    title('Angular Position Absolute')

    % Transpose into column vector
    phiRef1 = phiRef1';
    phiRef2 = phiRef2';
    phiRef3 = phiRef3';

    % Write pos ref to file
    fileID = fopen(FILE_NAME,'w');
    for i=1:length(phiRef1)
        fprintf(fileID, '%f\t%f\t%f\t%f\t%f\t%f\r\n', abs(phiRef1(i)), abs(phiRef2(i)), abs(phiRef3(i)), phiVel1(i), phiVel2(i), phiVel3(i));
        %fprintf(fileID, '%f\t%f\t%f\n', abs(phiRef1(i)), abs(phiRef2(i)), abs(phiRef3(i)));
    end

    figure(7)
    plot(tvec, abs(phiRef1), tvec, abs(phiRef2), tvec, abs(phiRef3));
    legend('Motor 1 Ang Pos', 'Motor 2 Ang Pos', 'Motor 3 Ang Pos')
