%intial parameter
    %i)Sheep position
       s=[-0.5,0.5];
    %ii)Point offset position
        %a)Length from center
        l=0.01;
        %b)Position
        p=s-(l/norm(s)).*(s);
     %iii)Dogs position
        d1=[-0.5,1];
        d2=[0,1];
        d3=[-0.5,0.75];
        d4=[0,0.5];
      %iv)Heading angle
      if s(end,1)<0 & s(end,2)<0
          phi=[atan(s(end,2)/s(end,1))];
      elseif s(end,1)>0 & s(end,2)>0
          phi=[pi+atan(s(end,2)/s(end,1))];
      elseif s(end,1)<0 & s(end,2)>0
          phi=[-atan(abs(s(end,2)/s(end,1)))];
      elseif s(end,1)>0 & s(end,2)<0
          phi=[pi-atan(abs(s(end,2)/s(end,1)))];
      elseif s(end,1)==0 & s(end,2)==0
          phi=0;
      elseif s(end,2)==0
          phi=0; 
      elseif s(end,1)==0
          phi=(3*pi)/2;
      end    
      %v)Angular separation
      delta=[];
      delta_1=[];
      delta_2=[];
      delta_3=[];
      delta_4=[];
      %vi)Time
      t=0:0.1:50;
      %vii) Controller parameters
      k=0.1;
      k_d=1;
      %viii)Radius of herd
        %a)desired radius
        r_0=0.1;
        %b)current radius
        r=[(norm(s-d1)+norm(s-d2)+norm(s-d3)+norm(s-d4))/4];
      %ix)No of herders members
      m=4;
      d1_star=[];
      d2_star=[];
      d3_star=[];
      d4_star=[];
      %parameter defined later
      p_f=[];
      s_dot=[];
curve1=animatedline('Color','Red','LineStyle',':','LineWidth',3);
set(gca,'Xlim',[-1 1],'Ylim',[-1 1]);
grid on
curve2=animatedline('Color','Black','LineStyle',':','LineWidth',3);
set(gca,'Xlim',[-1 1],'Ylim',[-1 1]);
grid on
curve3=animatedline('Color','Yellow','LineStyle',':','LineWidth',3);
set(gca,'Xlim',[-1 1],'Ylim',[-1 1]);
grid on
curve4=animatedline('Color','magenta','LineStyle',':','LineWidth',3);
set(gca,'Xlim',[-1 1],'Ylim',[-1 1]);
grid on
curve5=animatedline('Color','Green','LineStyle',':','LineWidth',3);
set(gca,'Xlim',[-1 1],'Ylim',[-1 1]);
grid on
curve6=animatedline('Color','cyan','LineStyle',':','LineWidth',3);
set(gca,'Xlim',[-1 1],'Ylim',[-1 1]);
grid on      
for i=1:length(t)
    
    if i==1

    else
    p(i,:)=s(i-1,:)-s(i-1,:).*(l/norm(s(i-1,:)));
    v_star=norm(-k.*[cos(phi(end)),sin(phi(end))]*transpose(p(i,:))); 
    syms x
    x=solve(v_star*(r_0^2)==sin((m*x)/(2*(1-m)))/sin((x)/(2*(1-m))),x);
    x=mod(vpa(x),6.28);
    delta(end+1)=real(x(2));
    delta_1(end+1)=(delta(end)*(1-m))/(2*(m-1));
    delta_2(end+1)=(delta(end)*(3-m))/(2*(m-1));
    delta_3(end+1)=(delta(end)*(5-m))/(2*(m-1));
    delta_4(end+1)=(delta(end)*(7-m))/(2*(m-1));
     
    d1_star=s(end,:)+ r_0.*[-cos(phi(end)+delta_1(end)),-sin(phi(end)+delta_1(end))];
    d2_star=s(end,:)+ r_0.*[-cos(phi(end)+delta_2(end)),-sin(phi(end)+delta_2(end))];
    d3_star=s(end,:)+ r_0.*[-cos(phi(end)+delta_3(end)),-sin(phi(end)+delta_3(end))];
    d4_star=s(end,:)+ r_0.*[-cos(phi(end)+delta_4(end)),-sin(phi(end)+delta_4(end))];
    d1(end+1,:)=d1(end,:)+(k_d*(t(i)-t(i-1))).*(d1_star-d1(end,:));
    d2(end+1,:)=d2(end,:)+(k_d*(t(i)-t(i-1))).*(d2_star-d2(end,:));
    d3(end+1,:)=d3(end,:)+(k_d*(t(i)-t(i-1))).*(d3_star-d3(end,:));
    d4(end+1,:)=d4(end,:)+(k_d*(t(i)-t(i-1))).*(d4_star-d4(end,:));
    s_dot=(s(i-1,:)-d1(end,:))./(norm(s(i-1,:)-d1(end,:))^3)+(s(i-1,:)-d2(end,:))./(norm(s(i-1,:)-d2(end,:))^3)+(s(i-1,:)-d3(end,:))./(norm(s(i-1,:)-d3(end,:))^3)+(s(i-1,:)-d4(end,:))./(norm(s(i-1,:)-d4(end,:))^3);
    s(i,:)=s(i-1,:)+(s_dot*1e-4).*(t(i)-t(i-1));
     if s(end,1)<0 & s(end,2)<0
          phi(end+1)=[atan(s(end,2)/s(end,1))];
      elseif s(end,1)>0 & s(end,2)>0
          phi(end+1)=[pi+atan(s(end,2)/s(end,1))];
      elseif s(end,1)<0 & s(end,2)>0
          phi(end+1)=[-atan(abs(s(end,2)/s(end,1)))];
      elseif s(end,1)>0 & s(end,2)<0
          phi(end+1)=[pi-atan(abs(s(end,2)/s(end,1)))];
      elseif s(end,1)==0 & s(end,2)==0
          phi(end+1)=0;
      elseif s(end,2)==0
          phi(end+1)=0;
      elseif s(end,1)==0
          phi=(3*pi)/2;
      end
    end
   addpoints(curve1,real(p(i,1)),real(p(i,2)))
   drawnow;
   addpoints(curve2,real(s(i,1)),real(s(i,2)))
   drawnow;
   addpoints(curve3,real(d1(i,1)),real(d1(i,2)))
   drawnow;
   addpoints(curve4,real(d2(i,1)),real(d2(i,2)))
   drawnow;
   addpoints(curve5,real(d3(i,1)),real(d3(i,2)))
   drawnow;
   addpoints(curve6,real(d4(i,1)),real(d4(i,2))) 
   drawnow;
  
end

   
     

