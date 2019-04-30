function [psziL psziR t] = bClocGen1v3_fun(tp, ts, psziS, a)
% tp [sec]
% ts [sec]
% psziS [deg]
% a [deg/s2]

close all
RES = 100;

time  = 1/RES:1/RES:tp; %[ms] ido
t = 1/RES:1/RES:tp;
tf = tp-ts;

wS = psziS/ts; % [deg/ms]stance-ben a szogsebesseg
wF = (360-psziS)/(tp-ts); % [deg/ms]Flight-ban a szogsebesseg

tInPer = time;

psziL  = zeros(1,tp);
dpsziL = zeros(1,tp);
psziR  = zeros(1,tp);
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
psziR(end) = psziR(end-1)+psziR(end-1)-psziR(end-2);
psziL(end) = psziL(end-1)+psziL(end-1)-psziL(end-2);

psziL = psziL - 180;
psziL = [-180 psziL];
psziL(end) = [];

hold on;
plot(psziL,'g','Linewidth',2);
plot(psziR,'--','Linewidth',2);
legend('psziL','psziR');
set(gca,'fontsize',12)
xlabel('t [ms]')
ylabel('Pszi [deg]')

dpsziL = RES*[(psziL(2:end)-psziL(1:end-1)) (psziL(end)-psziL(end-1))];
dpsziR = RES*[(psziR(2:end)-psziR(1:end-1)) (psziR(end)-psziR(end-1))];

figure(2);
hold on;
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