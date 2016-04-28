%% Ceryse Devaney
%% CSC-432 Intro to Sim and Modeling
%% Final Project: Reynolds Flocking Modeling Simplified
%% Only condition:  steer towards the average heading of local flock mates
%% 500 birds randomly distributed between (0,0) and (1,1) pointing in different directions
%% v (speed) is the constant for x and y-velocities of all birds
%% Agents attempt to steer toward average direction of birds within a radius 'R'
%% Agents' directions are also affected by an 'alpha' agent by a factor of 'w' which is unaffected by the other agents
%% Alpha is constantly traveling in circles

%In addition to the angle of the alpha affecting the other
%agents velocity by a factor of weight 'w', I had the distance from the alpha
%affect their position by a factor of weight 'w'


Nboid = 500;                  %500 birds
Nstp = 200;                   %number of time steps
xx = zeros(Nboid, Nstp);      %array of x-positions for each bird
yy = zeros(Nboid, Nstp);      %array of y-positions for each bird
vx = zeros(Nboid,Nstp);       %array of x- and y-velocitites for each bird at each time step
vy = zeros(Nboid,Nstp);
v=1;                          %speed remains the same for each bird

%%One ALPHA bird constantly circling where:
%%row 1: x-position (AX)
%%row 2: y-position (AY)
%%row 3: x-velocity (AVX)
%%row 4: y-velocity (AVY)
%%row 5: angle in radians (for velocity direction) (ANG)
%%and columns are time-steps
alpha = zeros(5,Nstp);        %Alpha agent position/velocity/direction matrix for each time step
ax = 1; ay = 2; avx =3; avy =4; ang = 5;

R = .1;                       %radius within which bird looks at neighbors

w = .7;                       %weight for/degree to which Alpha affects other birds

for ii=1:Nboid
    %% random init of position and velocity for all agents in Nboid
    xx(ii,1) = rand;
    yy(ii,1) = rand;
    theta = 2*pi*rand;
    vx(ii,1) = v*cos(theta);
    vy(ii,1) = v*sin(theta);
end

%% Alpha initialization
%% Deliberately centered
alpha(ax,1)=.5;
alpha(ay,1)=.5;
alpha(avx,1)=0;
alpha(avy,1)=-1;
alpha(ang,1)=0;

%% plot initial vectors of alpha and all Nboid agents
quiver(alpha(ax,1),alpha(ay,1),alpha(avx,1),alpha(avy,1),'Color',[102/255 0 102/255],'AutoScaleFactor',.25,'LineWidth',2'MaxHeadSize',8);
hold on;                      %keep alpha plot while displaying Nboid agents
quiver(xx(:,1),yy(:,1),vx(:,1),vy(:,1),'AutoScaleFactor',5);
axis([-5,15,-10,10]);
hold off;
pause (.1);

for t=2:Nstp
    for ii=1:Nboid
        x = xx(ii,t-1);                         %where x is previous x-pos
        y = yy(ii,t-1);                         %where y is previous y-pos
        %% find distance from Nboid agent to alpha agent
        radius = sqrt((alpha(ax,t-1)-x)^2+(alpha(ay,t-1)-y)^2);
        jx = 1;
        jy = 1;                       %adjusting x and y velocities
        xdiff = (alpha(ax,t-1)-x);
        ydiff = (alpha(ay,t-1)-y);
        %radius/tolerance of distance from alpha grows if w decreases
        %(if less weight/influence is given to the alpha)
        if radius>(10*(1-w))
             %% adjust x-vel and y-vel so if the agent is outside the given radius from alpha,
             %% the agent moves toward the alpha (if it is not moving toward the alpha already)
             if (xdiff < 0 && vx(ii,t-1)> 0)||(xdiff > 0 && vx(ii,t-1)<0)
                jx = -jx;
             end
             if (ydiff < 0 && vy(ii,t-1)> 0)||(ydiff > 0 && vy(ii,t-1)<0)
                jy= -jy;
             end
        end

        xx(ii,t) = xx(ii,t-1) + jx*vx(ii,t-1);    %new x-position
        yy(ii,t) = yy(ii,t-1) + jy*vy(ii,t-1);    %new y-position

        thetapre = acos(vx(ii,t-1)/v);            %previous angle
        thetasum = 0;                             %sum of all neighbour angles
        nei = 0;                                  %total number of neighbours within radius

        for jj=1:Nboid
            dist = (sqrt((x -(xx(jj,t-1)))^2+(y - (yy(jj,t-1)))^2));
            if(ii~=jj)
               if (dist < R)
                    %% add the angle in radians of agent jj to thetasum and count
                    %% neighbour
                    thetasum = thetasum + acos((vx(jj,t-1)/v));   %angle theta = inverse cosine of the x-vel over velocity
                    nei = nei + 1;
                end
            end
        end

        %%AFTER YOU FIND THE NEIGHBOURS: FIND THE AVG RADIAN OF NEIGHBOURS
        if nei~=0
            thetaavg = (((1-w)*thetasum+w*alpha(5,t-1))/(nei+1));
            new = thetapre + ((thetapre-thetaavg)/2);
        else
            new = thetapre + (((1-w)*thetapre-w*alpha(5,t-1))/2);
        end
        %%theta used in velocity changes depending on where agent's
        %%neighbors are facing
        %%shift previous angle towards neighbours/alpha

        vx(ii,t) = v*cos(new);      %x-velocity given new angle
        vy(ii,t) = v*sin(new);      %y-velocity given new angle


    end

        %% Update alpha agent position and velocity
        if alpha(ang,t-1)==2*pi
            alpha(ang,t)=0;
        else
            alpha(ang,t) = (alpha(ang,t-1))+(pi/16);            %alpha bird turns pi/16 radians each time step
        end
        alpha(avx,t) = (sin(alpha(ang,t)));                     %x velocity of bird at new radian
        alpha(avy,t) = -cos(alpha(ang,t));                      %y velocity of bird at new radian
        alpha(ay,t) = (alpha(ay,t-1)) + alpha(avy,t-1);         %current x-pos based on last x-vel
        alpha(ax,t) = (alpha(ax,t-1)) + alpha(avx,t-1);         %current y-pos based on last step's y-vel

        %% Plot vectors for all agents
        quiver(alpha(ax,t),alpha(ay,t),alpha(avx,t-1),alpha(avy,t-1),'color',[102/255 0 102/255],'AutoScaleFactor',3,'LineWidth',2,'MaxHeadSize',8);
        hold on;
        quiver(xx(:,t),yy(:,t),vx(:,t),vy(:,t),'AutoScale','off');
        axis([-5,15,-10,10]);
        hold off;
        pause(.01);

end

% %Verifying trajectories (checking randomness, alpha circling, steering influenced by neighbours)
%
% %plot trajectory of random agent element
% bird1 = randi(Nboid);
% plot(xx(bird1,:),yy(bird1,:),'color',[153/255, 0, 153/255])
% hold on

% %plot trajectory of another agent
% bird2 = randi(Nboid);
% plot(xx(bird2,:),yy(bird2,:),'color',[102/255 0 0])
% hold on

% %plot trajectory of alpha bird
% plot(alpha(ax,:),alpha(ay,:),'color',[102/255 51/255 202/255])
% hold on
% axis([-5,15,-10,10]);
