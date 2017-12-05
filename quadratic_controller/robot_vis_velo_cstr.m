close all;
clear;

%% Start RR Services
xbox = RobotRaconteur.Connect('rr+tcp://localhost:5437/?nodename=input_devices.xbox_controller&service=xbox_controller');
robot = RobotRaconteur.Connect('rr+tcp://localhost:62354/?service=EGM');

%% Initialize Robot Parameters
[ex,ey,ez,n,P,~,H,type,dq_bounds] = robotParams();

%% Initialize Control Parameters
% initial joint angles (unit: radian)
q = [0,-pi/6,pi/6,0,0,0]';

[R,pos] = fwdkin(q,type,H,P,n);
orien = R2q(R)';
pos_v = [0,0,0]';
ang_v = [1,0,0,0];
dq = zeros(n,1);

% desired end-effector orientation (here set it to be the initial
% orientation)
R_des = R;

% joint limits
lower_limit = [-17*pi/18 -65*pi/180 -pi -300*pi/180 -120*pi/180 2*pi]';
upper_limit = [17*pi/18 85*pi/180 70*pi/180 300*pi/180 120*pi/180 2*pi]';

% initialize inequality constraints
h = zeros(15, 1);
dhdq = [eye(6) zeros(6, 1) zeros(6, 1); -eye(6) zeros(6, 1) zeros(6, 1); zeros(1, 8)];

% plot options
view_port = [-30 30];
axes_lim = [-1.5 3.2 -1.5 3.2 -1.5 3.2];

% velocities
w_t = [0,0,0]';
v_t = [0,0,0]';

% keyboard controls
% define position and angle step size
inc_pos_v = 0.01; % m/s
inc_ang_v = 0.2/180*pi; % rad/s

% optimization params
er = 0.05;
ep = 0.05;
epsilon = 0;%legacy param for newton iters

% parameters for inequality constraints
c = 0.5;
eta = 0.1;
epsilon_in = 0.15;
E = 0.001;

% feedback coefficients for maintaining end-effector orientation
Ke = 0.9;

% create a handle of these parameters for interactive modifications
params = ControlParams(ex,ey,ez,n,P,H,type,dq_bounds,q,dq,pos,orien,pos_v,ang_v,w_t,v_t,...
    epsilon,view_port,axes_lim,inc_pos_v,inc_ang_v,0,er,ep,0);

%% Display Robot Initial Position
%plot out with plotting settings
%figure('keypressfcn',{@func_keypress,params});
%hold on
[pp,RR] = robot_3d(q);
view(view_port);
axis(axes_lim);
grid on
dt = 0;
counter = 0;

% cache of real velocity v_now
v_real = [];

% cache of desired input velocity (scaled by alpha) v_scaled
v_d = [];

% matrix of the position of end-effector params.controls.pos
p_eef = [];

% display loop
while ~params.controls.stop
    if (counter~=0)        
        dt = toc;        
    end    
    tic
    counter = counter + 1;
    if (counter~=0)
        % plots
        hold off;
        params.controls.q = params.controls.q + params.controls.dq*dt;
        [pp,RR] = robot_3d(params.controls.q);
        view(params.plots.view_port);
        axis(params.plots.axes_lim);
        grid on
        
        % pseudo obstacles
        x1 = 1.9; y1 = 0; z1 = 2.0; radius = 0.2;
        dispObstacles('sphere', radius, 0, x1, y1, z1);

        drawnow
        
        % update current position and orientation
        params.controls.pos = pp(:,end);
        params.controls.orien = R2q(RR(:,:,end))';
        
        % compute Jacobian
        J = getJacobian(params.controls.q, params.def.type, params.def.H, params.def.P, params.def.n);
        
        % compute new joint velocities
        axang = quat2axang(params.controls.ang_v);
        vr = (axang(4)*axang(1:3))';
        H = getqp_H(params.controls.dq, J, vr, params.controls.pos_v, params.opt.er, params.opt.ep);
        f = getqp_f(params.controls.dq,params.opt.er,params.opt.ep);
            
        % equality constraints 
        A_eq = [J(1:3,:) zeros(3, 2)];
        w_skew = logm(RR(:,:,end)*R_des');
        w = [w_skew(3, 2) w_skew(1, 3) w_skew(2, 1)];
        b_eq = -Ke*w;
        
        % inequality constrains
        h(1:6) = params.controls.q - lower_limit;
        h(7:12) = upper_limit - params.controls.q;

        dx = abs(params.controls.pos(1) - x1);
        dy = abs(params.controls.pos(2) - y1);
        dz = abs(params.controls.pos(3) - z1);
        
        dist = sqrt(dx^2 + dy^2 + dz^2) - radius;
        
        % derivative of dist w.r.t time
        der = [dx*(dx^2 + dy^2 + dz^2)^(-1/2) dy*(dx^2 + dy^2 + dz^2)^(-1/2) dz*(dx^2 + dy^2 + dz^2)^(-1/2)];
           
        h(13) = dist-0.3;
        
        dhdq(13, 1:6) = -der*J(4:6,:);
        sigma(1:12) = inequality_bound(h(1:12), c, eta, epsilon_in, E);
        sigma(13) = inequality_bound(h(13), c, eta, epsilon_in, E);
        
        A = -dhdq;
        b = -sigma;
               
        % bounds for qp
        if(params.opt.upper_dq_bounds)
            bound = params.def.dq_bounds(2,:);
        else
            bound = params.def.dq_bounds(1,:);
        end            
            
        LB = [-0.1*bound,0,0]';
        UB = [0.1*bound,1,1]';
        
        % quadratic programming
        options = optimset('Display', 'off');
       
        %dq_sln = quadprog(H,f,A,b,[],[],LB,UB,params.controls.dq,options);   
        dq_sln = quadprog(H,f,A,b,A_eq,b_eq,LB,UB,params.controls.dq,options); 

        % update joint velocities
        params.controls.dq = dq_sln(1:params.def.n);
        
        % desired velocity
        V_desired = params.controls.pos_v
        V_now = J(4:6,:)*params.controls.dq
        v_real = [v_real V_now];
        V_scaled = dq_sln(end)*V_desired
        v_d = [v_d V_scaled];
        p_eef = [p_eef params.controls.pos];
        
        params.controls.orien
        
        if norm(V_now)>=1e-10 && norm(V_scaled)>=1e-10
            direrr =  1-abs(dot(V_now/norm(V_now),V_scaled/norm(V_scaled)));
            if direrr<1-cosd(1)
                disp('following direction');
            elseif direrr<1-cosd(5)
                
                fprintf('following direction imprecisely, dir mismatch(degrees): %.2f, max joint velo: %.4f, ap: %4f\n',acosd(1-direrr),max(abs(params.controls.dq)),dq_sln(end)); 
            else
                fprintf('Wrong direction, dir mismatch(degrees): %.2f, max joint velo: %.4f, ap: %4f\n',acosd(1-direrr),max(abs(params.controls.dq)),dq_sln(end));
            end
        else
            fprintf('Zero Velocity Occurs\n');
            params.controls.dq = zeros(6, 1);
        end
        
        % current joint position
        %s=robot.joint_angle_setpoint;
        
        xBoxInput = xbox.controller_input;
        plus = xBoxInput.back_button;
        minus = xBoxInput.start_button;
        
        vx = (double(xBoxInput.A));
        vy = (double(xBoxInput.B));
        vz = (double(xBoxInput.X));

        button = [plus, minus, vx, vy, vz];
        func_xbox(button, params);
        
        % update joint position and send to EGM
        robot.joint_angle_setpoint = 180*params.controls.q / pi;
    end
end