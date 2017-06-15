theta=linspace(0,2*pi,281);
x=[0:5:45,50*cos(theta),45:-5:0];
y=[zeros(1,10),50*sin(theta),zeros(1,10)];

theta=[linspace(-60,60,301)];
max=301*ones(length(x),1);
table=[x' y' theta' max];