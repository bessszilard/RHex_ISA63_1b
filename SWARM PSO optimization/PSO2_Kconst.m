% Goal is to optimized the walking parameters, energy losses, walking speed and the leg’s spring 
% constant using particle swarm optimization.
% Evaluation results are calculated with RHex1v3_Z_0v1b simulink model
% code was written based on https://www.youtube.com/watch?v=sB1n9a9yxJk
% Author:   Szilard Bessenyei
% Year:		2016
% Place:	Subotica

clc;
clear;
close all;

%% Problem definition

CostFunction = @(x) SMCostFun(x);     % Cost function

nVar = 4;               % number of Unknowon (Deciison) Variables - dimensions of search space

VarSize = [1 nVar];     % Matrix Size of Decision Variables

tpLim = [0.5 3];
tsLim = [0.35 0.55];
psziSLim = [0.1 30];
alphaLim = [1000 0.9*140000/(64*pi)*180];
% k_Lim = [300 2000]; 

VarMin = [tpLim(1) tsLim(1) psziSLim(1) alphaLim(1)];    % Lower Bound of Decision Variable
VarMax = [tpLim(2) tsLim(2) psziSLim(2) alphaLim(2)];    % Upper Bound of Decision Variable

%% Parameters of PSO

MaxIt = 60;   % maximum number of iterations

nPop  = 15;    % Population Size (Swarm Size)

w = 1;         % inertia Coefficient
wdamp = 0.99;  % damping ration of Inertia Coefficient
c1 = 2;        % Personal Acceleration Coefficient
c2 = 2;        % Social Acceleration Coefficient

MaxVelocity = 0.5*(VarMax-VarMin);
MinVelocity = -MaxVelocity;


%% Initialization

% The Particle Template
empty_particle.Position = [];
empty_particle.Velocity = [];
empty_particle.Cost = [];
empty_particle.Best.Position = [];
empty_particle.Best.Cost = [];

% Create Population Array
particle = repmat(empty_particle, nPop, 1);

% Initilize Global Best
GlobalBest.Cost = inf;      % in we minimilaizaite
% GlobalBest.Position = [];

% Run Simulink Model
model = 'RHex1v3_Z_0v1b';
load_system(model);

% Initalize Population Members
for ind=1:nPop
%     if ind == 1
%         particle(ind).Position = [1.5 0.45 5 2500 660];  
%     elseif ind == 2
%         particle(ind).Position = [1.24 0.44 0.65 2379 946.5];
%     elseif ind == 3
%         particle(ind).Position = [1.4 0.59 0.50 2494 1011.1];
%     elseif ind == 4
%         particle(ind).Position = [1.187 0.46 0.5, 822.4 947.89;];
%     else
        % Generate Random Solution
        for j=1:nVar
           %particle(i).Position(j) = unifrnd(VarMin(j), VarMax(j), VarSize); 
           particle(ind).Position(j) = unifrnd(VarMin(j), VarMax(j),[1 1]); 
        end

%     end
    % Initialize Velocity
    particle(ind).Velocity = zeros(VarSize);

%% Evaluation
    x=particle(ind).Position;
    groundContact_initv2();
    tp = x(1);
    ts = x(2)*tp;
    psziS = x(3);
    a = x(4);
    K = 865;
    [psziL psziR t] = bClocGen1v3_fun(tp, ts, psziS, a);
    sim(model);

    startTime = 0.1;
    startInd = 1;
    startPer = 1;
    for iplot=1:length(ScopePos.time)
        if ScopePos.time(iplot) <= startTime
            startInd = iplot;
        end
        if ScopePos.time(iplot) <= tp
            startPer = iplot;
        end
    end

    maxaTor = max(max(abs(ScopeTau.signals.values)));
    avgTor = mean(mean(abs(ScopeTau.signals.values)));
    maxw= max(max(ScopeVel.signals.values(startPer:end,2)))/360*60;
    avgVel = mean(ScopeBodyVel.signals.values(startPer:end,2))*1000;
    maxFz = max(max(ScopeFz.signals.values(startInd:end,3)));
    cost = inf;
    
    U = 12;
    km = 13.4/1000;
    m = 2;
    g = 9.81;
     if maxaTor <= 5.0 && maxw < 120
%         cost = -avgVel;
        cost = avgTor/(avgVel);
    end
    
    particle(ind).Cost = cost;
    fprintf('[%1.1f %1.2f %2.2f %4.0f %3.1f]',tp, ts/tp, psziS, a, K);
    fprintf(' %2.2f %3.3f %3.3f %3.3f \n',ind/nPop, maxaTor, maxw, cost);
%%
    % update the Personal Best
    particle(ind).Best.Position = particle(ind).Position;
    particle(ind).Best.Cost = particle(ind).Cost;

    % Update Global Best
    if particle(ind).Best.Cost < GlobalBest.Cost
      GlobalBest = particle(ind).Best;
    end
end
% Array to Hold BEst Cost Value on Each Iteration
BestCosts = zeros(MaxIt, 1);

%% Main Loop of PSO

for it=1:MaxIt
   
    for ind=1:nPop
        
        % Update Velocity
        particle(ind).Velocity = w*particle(ind).Velocity ...
            + c1*rand(VarSize).*(particle(ind).Best.Position - particle(ind).Position) ...
            + c2*rand(VarSize).*(GlobalBest.Position - particle(ind).Position);
       
        % Aqpply velocity Limits
        particle(ind).Velocity = max(particle(ind).Velocity, MinVelocity);
        particle(ind).Velocity = min(particle(ind).Velocity, MaxVelocity);
        
        % Update Position
        particle(ind).Position = particle(ind).Position + particle(ind).Velocity;
        
        % Apply lower and Upper Bound Limits
        particle(ind).Position = max(particle(ind).Position, VarMin);
        particle(ind).Position = min(particle(ind).Position, VarMax);
        
        % Evaluation
        x=particle(ind).Position;
        groundContact_initv2();
        tp = x(1);
        ts = x(2)*tp;
        psziS = x(3);
        a = x(4);
        K =865;
        [psziL psziR t] = bClocGen1v3_fun(tp, ts, psziS, a);
        sim(model);

        startTime = 0.1;
        startInd = 1;
        startPer = 1;
        for plotind=1:length(ScopePos.time)
            if ScopePos.time(plotind) <= startTime
                startInd = plotind;
            end
            if ScopePos.time(plotind) <= tp
                startPer = plotind;
            end
        end

         maxaTor = max(max(abs(ScopeTau.signals.values)));
        avgTor = mean(mean(abs(ScopeTau.signals.values)));
        maxw= max(max(ScopeVel.signals.values(startPer:end,2)))/360*60;
        avgVel = mean(ScopeBodyVel.signals.values(startPer:end,2))*1000;
        maxFz = max(max(ScopeFz.signals.values(startInd:end,3)));
        cost = inf;
        U = 12;
        km = 13.4/1000;
        m = 2;
        g = 9.81;
         if maxaTor <= 5.0 && maxw < 120
%             cost = -avgVel;
              cost = avgTor/(avgVel);
        end

       particle(ind).Cost = cost;
       fprintf('[%1.1f %1.2f %2.2f %4.0f %3.1f]',tp, ts/tp, psziS, a, K);
       fprintf(' %2.2f %3.3f %3.3f %3.3f \n',ind/nPop+it, maxaTor, maxw, cost);

        
        % Update Pesonal Best
        if particle(ind).Cost < particle(ind).Best.Cost
           
            particle(ind).Best.Position = particle(ind).Position;
            particle(ind).Best.Cost = particle(ind).Cost;
            
            % Update Global Best
            if particle(ind).Best.Cost < GlobalBest.Cost
                GlobalBest = particle(ind).Best;
            end
        end
        
    end
    
    % Store the BEst Cost Value
    BestCosts(it) = GlobalBest.Cost;
    
    % Display Iteration Informaiton
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
    
    % Damping Inertia Coefficient;
    w = w*wdamp;
end

%% Results
GlobalBest
figure;
semilogy(BestCosts,'LineWidth',2);
xlabel('Iterations');
ylabel('Best Cost');