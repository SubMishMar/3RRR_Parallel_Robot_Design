    R=125;
    a=54.019;
    l1=62*2;
    l2=42*2;
    A1=[-sqrt(3)*R/2;-R/2];
    A2=[+sqrt(3)*R/2;-R/2];
    A3=[0;R];
    baseLoc=[A1';A2';A3'];
for i=1:300
    figure(1);
    plot(baseLoc(:,1),baseLoc(:,2),'+','Markersize',16);
    hold on;
    axis([-160 160 -160 160]);
    axis square
    grid
    theta=linspace(0,2*pi,200);
    xC=R*cos(theta);
    yC=R*sin(theta);
    plot(xC,yC,'.')
    hold on;
    Rp=50;
    xC=Rp*cos(theta);
    yC=Rp*sin(theta);
    plot(xC,yC,'*')
    hold on;
    x=[linspace(0,50,50),xC,linspace(50,0,50)];
    y=[zeros(1,length(linspace(0,50,50))),yC,zeros(1,length(linspace(0,50,50)))];
    phi=[linspace(0,60,150),linspace(60,0,150)]*pi/180;%variable orientation
    %phi=(pi/15)*ones(1,300);%constant orientation
    trajectory=[x' y' phi'];
    X=trajectory(i,1);
    Y=trajectory(i,2);
    Phi=trajectory(i,3);
   
    %Leg 1
    C1=[X-(a/sqrt(3))*cos(Phi+pi/6);Y-(a/sqrt(3))*sin(Phi+pi/6)];
    A1C1=sqrt((A1(1)-C1(1))^2+(A1(2)-C1(2))^2);
    alpha1=acos((A1C1^2+l1^2-l2^2)/(2*A1C1*l1));
    theta1=atan2((C1(2)-A1(2)),(C1(1)-A1(1)))-alpha1;
    B1=A1+[l1*cos(theta1);l1*sin(theta1)];
    x=linspace(A1(1),B1(1),100);
    y=linspace(A1(2),B1(2),100);
    figure(1);
    plot(x,y,'b');
    hold on;
    x=linspace(B1(1),C1(1),100);
    y=linspace(B1(2),C1(2),100);
    figure(1);
    plot(x,y,'g');
    hold on;
    %Leg 2
    C2=[X-(a/sqrt(3))*cos(Phi+pi/6)+a*cos(Phi);Y-(a/sqrt(3))*sin(Phi+pi/6)+a*sin(Phi)];
    A2C2=sqrt((A2(1)-C2(1))^2+(A2(2)-C2(2))^2);
    alpha2=acos((A2C2^2+l1^2-l2^2)/(2*A2C2*l1));
    theta2=atan2((C2(2)-A2(2)),(C2(1)-A2(1)))-alpha2;
    B2=A2+[l1*cos(theta2);l1*sin(theta2)];
    x=linspace(A2(1),B2(1),100);
    y=linspace(A2(2),B2(2),100);
    figure(1);
    plot(x,y,'b');
    hold on;
    x=linspace(B2(1),C2(1),100);
    y=linspace(B2(2),C2(2),100);
    figure(1);
    plot(x,y,'g');
    hold on;
    %Leg 3
    C3=[X-(a/sqrt(3))*cos(Phi+pi/6)+a*cos(Phi+pi/3);Y-(a/sqrt(3))*sin(Phi+pi/6)+a*sin(Phi+pi/3)];
    A3C3=sqrt((A3(1)-C3(1))^2+(A3(2)-C3(2))^2);
    alpha3=acos((A3C3^2+l1^2-l2^2)/(2*A3C3*l1));
    theta3=atan2((C3(2)-A3(2)),(C3(1)-A3(1)))-alpha3;
    B3=A3+[l1*cos(theta3);l1*sin(theta3)];
    x=linspace(A3(1),B3(1),100);
    y=linspace(A3(2),B3(2),100);
    figure(1);
    plot(x,y,'b');
    hold on;
    x=linspace(B3(1),C3(1),100);
    y=linspace(B3(2),C3(2),100);
    figure(1);
    plot(x,y,'g');
    hold on;
    %Triangular Moving PF
    x=[C1(1) C2(1) C3(1)];
    y=[C1(2) C2(2) C3(2)];
    fill(x,y,'r');
    plot(X,Y,'.');
    pause(0.001);
    hold off;
    A=[cross([C1;0],[B1;0]-[C1;0])'./norm(cross([C1;0],[B1;0]-[C1;0])) ([B1;0]-[C1;0])'./norm(([B1;0]-[C1;0]));
       cross([C2;0],[B2;0]-[C2;0])'./norm(cross([C2;0],[B2;0]-[C2;0])) ([B2;0]-[C2;0])'./norm(([B2;0]-[C2;0]));
       cross([C3;0],[B3;0]-[C3;0])'./norm(cross([C3;0],[B3;0]-[C3;0])) ([B3;0]-[C3;0])'./norm(([B3;0]-[C3;0]))];
   A=A(:,3:5);
   condnoA(i)=cond(A);
   B=[dot(cross([A1;0]-[C1;0],[B1;0]-[C1;0]),[0;0;1])./norm(cross([A1;0]-[C1;0],[B1;0]-[C1;0])) 0 0;
       0 dot(cross([A2;0]-[C2;0],[B2;0]-[C2;0]),[0;0;1])./norm(cross([A2;0]-[C2;0],[B2;0]-[C2;0])) 0;
       0 0 dot(cross([A3;0]-[C3;0],[B3;0]-[C3;0]),[0;0;1])./norm(cross([A3;0]-[C3;0],[B3;0]-[C3;0]))];
   condnoB(i)=cond(B);
end

