close all
RES = 100;
% tp    = 2.0;   %[ms] peridus hossza
% ts    = tp/3; %[ms] stance hossza
% psziS = 5;     %[deg] stance szoge

% tp    = 1.0;   %[ms] peridus hossza
% ts    = tp/3; %[ms] stance hossza
% % psziS = 5;     %[deg] stance szoge
% a = 2000; %[deg/s^2]

% tp=1.5;
% ts = 40*tp/100; %[ms] stance hossza
% psziS = 5;
% a = 2000; %[deg/s^2]

% tp=1.5;
% ts = 55*tp/100; %[ms] stance hossza
% psziS = 30;
% a = 2000; %[deg/s^2]

% tp = 1.221;
% ts = 43.97/100*tp;
% psziS =1;
% a = 2737;
% K = 1102.3;

% tp = 1.597;
% ts = 0.45*tp;
% psziS = 0.116;
% a = 5.3376e+03;
% K = 580.8064;

% tp = 1.5845;
% ts = 0.4419*tp;
% psziS = 0.1000;
% a = 4.4428e+03;
% K = 605.9639;

tp =3;
ts = 2/3*tp;
psziS = 60;
a = 1000;
K = 1000;

time  = 1/RES:1/RES:tp; %[ms] ido
t = 1/RES:1/RES:tp;
tf = tp-ts;

wS = psziS/ts; % [deg/ms]stance-ben a szogsebesseg
wF = (360-psziS)/(tp-ts); % [deg/ms]Flight-ban a szogsebesseg
% wS = -a*(ts-sqrt(ts^2+2*psziS/a)); % kezdoseb + gyorsulas
% wF = a*(tf-sqrt(tf^2-2*(360-psziS)/a)); %kezdoseb - gyorsulas

% wS = a/3*(ts+sqrt(ts^2+6*psziS/a)); % kezdoseb + gyorsulas
% wF = a*(tf-sqrt(tf^2-2*(360-psziS)/a)); %kezdoseb - gyorsulas
% 
% wS = a*(-ts+sqrt(ts^2+2*psziS/a)); % kezdoseb + gyorsulas
% wF = a*(tf-sqrt(tf^2-2*(360-psziS)/a)); %kezdoseb - gyorsulas


tInPer = mod(time, tp);

psziL = zeros(1,tp);
dpsziL = zeros(1,tp);
psziR = zeros(1,tp);
dpsziR = zeros(1,tp);

tl1 = (tp-ts)/2 - (wF-wS)/a/2;
tl2 = (tp-ts)/2 + (wF-wS)/a/2;
tl3 = (tp+ts)/2 - (wF-wS)/a/2;
tl4 = (tp+ts)/2 + (wF-wS)/a/2;
tl5 = tp;

p_tl1 = tl1 * wF;
p_tl2 = p_tl1 + (tl2-tl1)*wF - a*(tl2-tl1)^2/2;
p_tl3 = p_tl2 + (tl3-tl2)*wS;
p_tl4 = p_tl3 + (tl4-tl3)*wS + a*(tl4-tl3)^2/2;
p_tl5 = p_tl4 + (tl5-tl4)*wF;

tr1 = ts/2 - (wF-wS)/a/2;
tr2 = ts/2 + (wF-wS)/a/2;
tr3 = tp-ts/2 - (wF-wS)/a/2;
tr4 = tp-ts/2 + (wF-wS)/a/2;
tr5 = tp;
 
p_tr1 = tr1 * wS;
p_tr2 = p_tr1 + (tr2-tr1)*wS + a*(tr2-tr1)^2/2;
p_tr3 = p_tr2 + (tr3-tr2)*wF;
p_tr4 = p_tr3 + (tr4-tr3)*wF - a*(tr4-tr3)^2/2;
p_tr5 = p_tr4 + (tr5-tr4)*wS;


for i=1:tp*RES
   if tInPer(i) < tl1 % flight and omega = const
       psziL(i) = tInPer(i)*wF;
   elseif tInPer(i) < tl2 % flight and deceleration
       psziL(i) = wF*(tInPer(i)-tl1) - (tInPer(i)-tl1)^2*a/2 + p_tl1;
   elseif tInPer(i) < tl3   % stance
       psziL(i) = (tInPer(i)-tl2)*wS + p_tl2;
   elseif tInPer(i) < tl4   % stance and acceleration
       psziL(i) = wS*(tInPer(i)-tl3) + (tInPer(i)-tl3)^2*a/2 + p_tl3;
   else                 % flight
       psziL(i) = wF*(tInPer(i)-tl4) + p_tl4;
   end
   
   if tInPer(i) < tr1 % stance and omega = const
       psziR(i) = tInPer(i)*wS;
   elseif tInPer(i) < tr2 % stance and acceleration
       psziR(i) = wS*(tInPer(i)-tr1) + (tInPer(i)-tr1)^2*a/2 + p_tr1;
   elseif tInPer(i) < tr3   % flight
       psziR(i) = wF*(tInPer(i)-tr2) + p_tr2;
   elseif tInPer(i) < tr4   % flight and deceleration
       psziR(i) = wF*(tInPer(i)-tr3) - (tInPer(i)-tr3)^2*a/2 + p_tr3;
   else                 % stance and omega = const
       psziR(i) = wS*(tInPer(i)-tr4) + p_tr4;
   end
         
end
% psziR = psziR/max(psziR)*360;
% for i = 1 : length(psziR)
%    if 179 < psziR(i)
%        psziR(i) = psziR(i) - 360;
%    end
% end
psziR(end) = psziR(end-1)+psziR(end-1)-psziR(end-2);
psziL(end) = psziL(end-1)+psziL(end-1)-psziL(end-2);

% psziL = psziL/max(psziL)*360;
psziL = psziL - 180;
% psziR = psziR/max(psziR)*180;
psziL = [-180 psziL];
psziL(end) = [];
% ksziL(length(ksziL)-10:end)=200;

hold on;
plot(psziL,'g','Linewidth',2);
plot(psziR,'--','Linewidth',2);
% plot(psziR,'g')
legend('psziL','psziR');
set(gca,'fontsize',12)
xlabel('t [ms]')
ylabel('Pszi [deg]')

dpsziL = RES*[(psziL(2:end)-psziL(1:end-1)) (psziL(end)-psziL(end-1))];
dpsziR = RES*[(psziR(2:end)-psziR(1:end-1)) (psziR(end)-psziR(end-1))];

% for i=1:length(dpsziR)
%    if (dpsziR(i)) < -100
%        dpsziR(i) = dpsziR(i+1);
%    end
% end


figure(2);
hold on;
% plot(dpsziL1v2);
plot(dpsziL,'g','Linewidth',2);
plot(dpsziR,'--','Linewidth',2);
legend('omegaL','omegaR');
set(gca,'fontsize',12)
xlabel('t [ms]')
ylabel('omega [deg/s]')

axis([0 length(dpsziL) 0 1.4*max(dpsziL)]);

t = [t tp+t 2*tp+t 3*tp+t];
psziL=[psziL 360+psziL 720+psziL 1080+psziL];
psziR=[psziR 360+psziR 720+psziR 1080+psziR];
dpsziL = [dpsziL dpsziL dpsziL];
dpsziR = [dpsziR dpsziR dpsziR];

figure(4);
hold on;
plot(psziL,'g','Linewidth',2);
plot(psziR,'--','Linewidth',2);

close all

