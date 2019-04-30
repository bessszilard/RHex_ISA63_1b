clc;
clear;
close all;

%% Problem definition

CostFunction = @(x) SMCostFun(x);     % Cost function

nVar = 4;               % number of Unknowon (Deciison) Variables - dimensions of search space

VarSize = [1 nVar];     % Matrix Size of Decision Variables

kpLim = [150 300];
kiLim = [0 200];
kdLim = [0 200];
knLim = [0 3];
% k_Lim = [300 2000]; 

% def values:
kp = 100;
ki =   2;
kd =  10;
kn =   0.00001;

VarMin = [kpLim(1) kiLim(1) kdLim(1) knLim(1)];% k_Lim(1)];    % Lower Bound of Decision Variable
VarMax = [kpLim(2) kiLim(2) kdLim(2) knLim(2)];% k_Lim(2)];    % Upper Bound of Decision Variable

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
model = 'RHex1v3_Z_0v1b_PID';
load_system(model);


% avgVel = 420.127 mm/s
tp = 1.6;
ts = 0.44*tp;
psziS = 0.15;
a = 4714;
K = 525.4;
groundContact_initv2();
[psziL psziR t] = bClocGen1v3_fun(tp, ts, psziS, a);


% Initalize Population Members
for ind=1:nPop
% Generate Random Solution
	for j=1:nVar
	   %particle(i).Position(j) = unifrnd(VarMin(j), VarMax(j), VarSize); 
	   particle(ind).Position(j) = unifrnd(VarMin(j), VarMax(j),[1 1]); 
	end

% Initialize Velocity
    particle(ind).Velocity = zeros(VarSize);

%% Evaluation
    x=particle(ind).Position;
    groundContact_initv2();
    kp = x(1);
    ki = x(2);
    kd = x(3);
    kn = x(4);
    fprintf('[%5.1f %3.1f %3.1f %0.4f] ', x(1), x(2), x(3), x(4));
	
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
	errorI = max(ScopeErrorI.signals.values)*1000;
    cost = inf;
	
    if maxaTor <= 5.0 && maxw < 120
%         cost = -avgVel;
%        cost = avgTor/(avgVel);
		cost = errorI;
    end
    
    particle(ind).Cost = cost;
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
		kp = x(1);
		ki = x(2);
		kd = x(3);
		kn = x(4);
		fprintf('[%5.1f %3.1f %3.1f %0.4f] ', x(1), x(2), x(3), x(4));		
		
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
		errorI = max(ScopeErrorI.signals.values)*1000;
		cost = inf;
		
		 if maxaTor <= 5.0 && maxw < 120
	%         cost = -avgVel;
	%        cost = avgTor/(avgVel);
			cost = errorI;
		end
		
		particle(ind).Cost = cost;
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