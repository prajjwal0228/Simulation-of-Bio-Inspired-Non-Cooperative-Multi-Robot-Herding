%intial parameter
    %i)Sheep position
       s_1=[1,1];
       s_2=[1,2];
       s_avg=0.5.*(s_1+s_2);
    %ii)Point offset position
        %a)Length from center
        l=0.01;
        %b)Position
        p=s_avg-(l/norm(s_avg)).*(s_avg);
     %iii)Dogs position
        d1=[1,0];
        d2=[0,0];
        d3=[1,4];
        d4=[0,3];
      %iv)Heading angle
      if s_avg(end,1)<0 & s_avg(end,2)<0
          phi(1,:)=[atan(s_avg(end,2)/s_avg(end,1))];
      elseif s_avg(end,1)>0 & s_avg(end,2)>0
          phi(1,:)=[pi+atan(s_avg(end,2)/s_avg(end,1))];
      elseif s_avg(end,1)<0 & s_avg(end,2)>0
          phi(1,:)=[-atan(abs(s_avg(end,2)/s_avg(end,1)))];
      elseif s_avg(end,1)>0 & s_avg(end,2)<0
          phi(1,:)=[pi-atan(abs(s_avg(end,2)/s_avg(end,1)))];
      elseif s_avg(end,1)==0 & s_avg(end,2)==0
          phi(1,:)=0;
      elseif s_avg(end,2)==0
          phi(1,:)=0; 
      elseif s_avg(end,1)==0
          phi(1,:)=(3*pi)/2;
      end
      %v)Angular separation
      delta=[];
      delta_1=[];
      delta_2=[];
      delta_3=[];
      delta_4=[];
      %vi)Time
      t=0:0.1:250;
      %vii) Controller parameters
      k=0.1;
      k_d=1;
      %viii)Radius of herd
        %a)desired radius
        r_0=0.5;
        r=r_0;
      %ix)No of herders members
      m=4;
      d1_star=[];
      d2_star=[];
      d3_star=[];
      d4_star=[];
      %parameter defined later
      p_f=[];
      s1_dot=[];
      s2_dot=[];
      savg_dot=[];
curve1=animatedline('Color','Red','LineStyle',':','LineWidth',1.5);
set(gca,'Xlim',[-3 3],'Ylim',[-3 3]);
grid on
curve2=animatedline('Color','Black','LineStyle',':','LineWidth',1.5);
set(gca,'Xlim',[-3 3],'Ylim',[-3 3]);
grid on
curve2_1=animatedline('Color','Black','LineStyle','-','LineWidth',1.5);
set(gca,'Xlim',[-3 3],'Ylim',[-3 3]);
grid on
curve3=animatedline('Color','Yellow','LineStyle',':','LineWidth',1.5);
set(gca,'Xlim',[-3 3],'Ylim',[-3 3]);
grid on
curve4=animatedline('Color','magenta','LineStyle',':','LineWidth',1.5);
set(gca,'Xlim',[-3 3],'Ylim',[-3 3]);
grid on
curve5=animatedline('Color','Green','LineStyle',':','LineWidth',1.5);
set(gca,'Xlim',[-3 3],'Ylim',[-3 3]);
grid on
curve6=animatedline('Color','cyan','LineStyle',':','LineWidth',1.5);
set(gca,'Xlim',[-3 3],'Ylim',[-3 3]);
grid on      
for i=1:length(t)
    
    if i==1

    else
    p(i,:)=s_avg(i-1,:)-s_avg(i-1,:).*(l/norm(s_avg(i-1,:)));
    v_star=norm(-k.*[cos(phi(end)),sin(phi(end))]*transpose(p(i,:))); 
    syms x
    x=solve(v_star*(r^2)==sin((m*x)/(2*(1-m)))/sin((x)/(2*(1-m))),x); 
    x=mod(real(vpa(x)),6.28);
    delta(end+1)=real(x(2));
    delta_1(end+1)=(delta(end)*(1-m))/(2*(m-1));
    delta_2(end+1)=(delta(end)*(3-m))/(2*(m-1));
    delta_3(end+1)=(delta(end)*(5-m))/(2*(m-1));
    delta_4(end+1)=(delta(end)*(7-m))/(2*(m-1));
     
    d1_star=s_avg(end,:)+ r.*[-cos(phi(end)+delta_1(end)),-sin(phi(end)+delta_1(end))];
    d2_star=s_avg(end,:)+ r.*[-cos(phi(end)+delta_2(end)),-sin(phi(end)+delta_2(end))];
    d3_star=s_avg(end,:)+ r.*[-cos(phi(end)+delta_3(end)),-sin(phi(end)+delta_3(end))];
    d4_star=s_avg(end,:)+ r.*[-cos(phi(end)+delta_4(end)),-sin(phi(end)+delta_4(end))];
    d1(end+1,:)=d1(end,:)+(k_d*(t(i)-t(i-1))).*(d1_star-d1(end,:));
    d2(end+1,:)=d2(end,:)+(k_d*(t(i)-t(i-1))).*(d2_star-d2(end,:));
    d3(end+1,:)=d3(end,:)+(k_d*(t(i)-t(i-1))).*(d3_star-d3(end,:));
    d4(end+1,:)=d4(end,:)+(k_d*(t(i)-t(i-1))).*(d4_star-d4(end,:));
    
    k=s_1(i-1,:)-s_2(i-1,:);
    
    s1_dot_part_1=(s_1(i-1,:)-d1(end,:))./(norm(s_1(i-1,:)-d1(end,:))^3)+(s_1(i-1,:)-d2(end,:))./(norm(s_1(i-1,:)-d2(end,:))^3)+(s_1(i-1,:)-d3(end,:))./(norm(s_1(i-1,:)-d3(end,:))^3)+(s_1(i-1,:)-d4(end,:))./(norm(s_1(i-1,:)-d4(end,:))^3);
    
    s1_dot=1e-3.*(s1_dot_part_1)-k.*(tanh(norm(k)-0.1));
    
    s2_dot_part_1=(s_2(i-1,:)-d1(end,:))./(norm(s_2(i-1,:)-d1(end,:))^3)+(s_2(i-1,:)-d2(end,:))./(norm(s_2(i-1,:)-d2(end,:))^3)+(s_2(i-1,:)-d3(end,:))./(norm(s_2(i-1,:)-d3(end,:))^3)+(s_2(i-1,:)-d4(end,:))./(norm(s_2(i-1,:)-d4(end,:))^3);
    
    s2_dot=1e-3*s2_dot_part_1+k.*(tanh(norm(k)-0.1));
    
    savg_dot=(s1_dot+s2_dot)./2;

    s_1(i,:)=s_1(i-1,:)+(s1_dot).*(t(i)-t(i-1));
    s_2(i,:)=s_2(i-1,:)+(s2_dot).*(t(i)-t(i-1));
    s_avg(i,:)=s_avg(i-1,:)+(savg_dot).*(t(i)-t(i-1));
    
    if s_avg(end,1)<0 & s_avg(end,2)<0
          phi(end+1)=[atan(s_avg(end,2)/s_avg(end,1))];
      elseif s_avg(end,1)>0 & s_avg(end,2)>0
          phi(end+1)=[pi+atan(s_avg(end,2)/s_avg(end,1))];
      elseif s_avg(end,1)<0 & s_avg(end,2)>0
          phi(end+1)=[-atan(abs(s_avg(end,2)/s_avg(end,1)))];
      elseif s_avg(end,1)>0 & s_avg(end,2)<0
          phi(end+1)=[pi-atan(abs(s_avg(end,2)/s_avg(end,1)))];
      elseif s_avg(end,1)==0 & s_avg(end,2)==0
          phi(end+1)=0;
      elseif s_avg(end,2)==0
          phi(end+1)=0;
      elseif s_avg(end,1)==0
          phi=(3*pi)/2;
      end

    r_dot=(r_0-r)+(s_1(i,:)-s_avg(i,:))*transpose(s1_dot-savg_dot)+(s_2(i,:)-s_avg(i,:))*transpose(s2_dot-savg_dot);
    r=r+(r_dot*(t(i)-t(i-1)));
    end


   addpoints(curve1,real(p(i,1)),real(p(i,2)))
   if mod(i,5)==0
       drawnow;
   end
   addpoints(curve2,real(s_1(i,1)),real(s_1(i,2))) 
if mod(i,5)==0
       drawnow;
end   
addpoints(curve2_1,real(s_2(i,1)),real(s_2(i,2))) 
   
   addpoints(curve3,real(d1(i,1)),real(d1(i,2))) 
   if mod(i,5)==0
       drawnow;
   end
   addpoints(curve4,real(d2(i,1)),real(d2(i,2))) 
   if mod(i,5)==0
       drawnow;
   end
   addpoints(curve5,real(d3(i,1)),real(d3(i,2))) 
   if mod(i,5)==0
       drawnow;
   end
   addpoints(curve6,real(d4(i,1)),real(d4(i,2))) 
   if mod(i,5)==0
       drawnow;
   end

   %  addpoints(curve1,real(p(i,1)),real(p(i,2))) 
   % drawnow;
   % addpoints(curve2,real(s_1(i,1)),real(s_1(i,2))) 
   % drawnow;
   % addpoints(curve2_1,real(s_2(i,1)),real(s_2(i,2))) 
   % drawnow;
   % addpoints(curve3,real(d1(i,1)),real(d1(i,2))) 
   % drawnow;
   % addpoints(curve4,real(d2(i,1)),real(d2(i,2))) 
   % drawnow;
   % addpoints(curve5,real(d3(i,1)),real(d3(i,2))) 
   % drawnow;
   % addpoints(curve6,real(d4(i,1)),real(d4(i,2))) 
   % drawnow;
end 

  
