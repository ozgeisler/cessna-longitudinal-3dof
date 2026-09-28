clear
clc
close all

h = 5000*0.3048;                     % 5000 ft to SI unit
TAS_list = [30 40 50 60 70 80];

% First Guess: [u; w; q; theta; de; uth]
Z_guess = [TAS_list(1); 0; 0; 0; 0; 0];

results = zeros(6,9);

for i = 1:length(TAS_list)

    Va_target = TAS_list(i);

    costfun = @(Z) cost_trim_cessna(Z, Va_target, h);

    [ZStar,f0] = fminsearch(costfun, Z_guess, ...
        optimset('TolX',1e-10,'TolFun',1e-12,'MaxFunEvals',20000,'MaxIter',20000)); 
    % It gives 

    u_vel = ZStar(1);
    w_vel = ZStar(2);
    q     = ZStar(3);
    theta = ZStar(4);
    de    = ZStar(5);
    uth   = ZStar(6);

    XStar = [u_vel; w_vel; q; theta; 0; -h];
    UStar = [de; uth];

    % Validation
    XdotStar = cessna182_model(XStar,UStar)

    Va    = sqrt(u_vel^2 + w_vel^2);
    alpha = atan2(w_vel,u_vel);
    gam   = theta - alpha;

    %Dynamic pressure and aerodynamic coefficients in trim points
    rho  = 1.225*(1 - 2.25577e-5*h)^4.2559;
    qbar = 0.5*rho*Va^2;

    cbar = 1.49;
    qhat = q*cbar/(2*Va);

    CL = 0.307 + 4.41*alpha + 3.9*qhat + 0.43*de;
    CD = 0.0270 + 0.121*alpha;
    Cm = 0.04 - 0.613*alpha - 12.4*qhat - 1.122*de;

    results(i,:) = [Va, rad2deg(alpha), rad2deg(theta), rad2deg(gam), ...
                     qbar, CL, CD, Cm, rad2deg(de)];

    fprintf('TAS=%d m/s  ->  alpha=%.3f deg  theta=%.3f deg  de=%.3f deg  uth=%.4f  f0=%.3e\n',...
        TAS_list(i), rad2deg(alpha), rad2deg(theta), rad2deg(de), uth, f0);

    % bir sonraki hiz icin baslangic tahmini olarak bu sonucu kullan
    Z_guess = ZStar;

    save(['trim_' num2str(TAS_list(i)) 'ms.mat'],'XStar','UStar');
end

T = array2table(results,'VariableNames', ...
    {'TAS','alpha_deg','theta_deg','gamma_deg','qbar','CL','CD','Cm','de_deg'});
disp(T)
