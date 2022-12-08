%get planet params
planet

%initial conditions
altitude = 254*1.6*1000; %meters

x0 = R + altitude; %alt of planet
y0 = 0; 
z0 = 0; 

inclination = 51.6*pi/180; 
semi_major = norm([x0;y0;z0]);
vcircular = sqrt(mu/semi_major); %distance from center of earth to sat

xdot0 = 0;
ydot0 = vcircular * cos(inclination);
zdot0 = -vcircular * sin(inclination); 

stateinitial = [x0;y0;z0;xdot0;ydot0;zdot0];

%time window
period = 2 * pi/sqrt(mu) * semi_major^(3/2);
number_of_orbits = 3; 
tspan = [0 period * number_of_orbits];

%integrate equation of motion
[tout, stateout] = ode45(@Satellite, tspan, stateinitial);

%Convert to km
stateout = stateout / 1000;

%extract state vector
xout = stateout(:,1);
yout = stateout(:,2);
zout = stateout(:,3);

%make earth
[X,Y,Z] = sphere(100);
X = X*R/1000;
Y = Y*R/1000;
Z = Z*R/1000;

%plot 3D orbit
fig = figure(); 
set(fig, 'color', 'white');
plot3(xout, yout, zout, 'b-', 'LineWidth', 4);
grid on;
hold on;
surf(X,Y,Z, 'EdgeColor', 'none'); 
axis equal; 