function F0 = cost_trim_cessna(Z, Va_target, h)

u_vel = Z(1);
w_vel = Z(2);
q     = Z(3);
theta = Z(4);
de    = Z(5);
uth   = Z(6);

X = [u_vel; w_vel; q; theta; 0; -h];
U = [de; uth];

xdot = cessna182_model(X,U);

Va    = sqrt(u_vel^2 + w_vel^2);
alpha = atan2(w_vel,u_vel);
gam   = theta - alpha;

Q = [xdot(1);      % udot = 0
     xdot(2);      % wdot = 0
     xdot(3);      % qdot = 0
     Va - Va_target;
     gam;
     q];            

H  = diag(ones(1,6));
F0 = Q'*H*Q;

