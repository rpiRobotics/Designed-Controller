X = p_eef(1,:);
Y = p_eef(2, :);
Z = p_eef(3, :);
plot3(X,Y,Z, 'o');
hold on
dispObstacles('sphere', radius, 0, x1, y1, z1);
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
text(1.3,0.2,1.8, 'Initial position');
title('Trajectory of robot end-effector');