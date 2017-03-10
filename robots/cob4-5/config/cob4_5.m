clc;
%clear;
care_o_bot_arm_config;
run('/home/bbrito/rvctools/startup_rvc.m');
%% Denavit-Hart Parameters

%%theta d a alfa joint type 0=revolute 1=prismatic

H=[ 0 0.386 0.14743 atan2(0.386-0.310153,0.14743) 0;%TORSO
    0 0 -0.14743 -atan2(0.386-0.310153,0.14743) 0;%TORSO
    0 1.0066-0.386 0 -pi/2 0;%ARM
    0 0.24464 0 -pi/2 0;%ARM
    0 0 0 pi/2 0;%ARM - pi/2 for the other arm
   0 0.69825-0.24464 0 -pi/2 0;
   0 0 0 pi/2 0;
   0 0.99684-0.69825 0 -pi/2 0;
   0 0 0 pi/2 0;
   0 0 0 0 0;];
j=-1;
n=size(H)

for i=1:1:n(1)
    L(i)=Link(H(i,:));
end

robot=SerialLink(L, 'name', 'schunk')
robot.links(1).offset=0;
robot.links(2).offset=0;
%% Calculate all homogenoues transformations for each joint for the simulink
%%model
a=[];
for i=1:1:n(1)
    a=cat(n(1),a,robot.A(i,zeros(1,n(1))));
end

Torso_Link=Link([0 0.45657-0.386 0 0 0]);
Torso_Link.offset(1)=0;
torso_link=Torso_Link.A(0);

Shoulder1_Link=Link([0 0.139 0 0 0]);
Shoulder1_Link.offset(1)=pi;
shoulder1_link=Shoulder1_Link.A(0);

PowerBall1_Link=Link([0 0 0 0 0]);
PowerBall1_Link.offset(1)=0;
powerball1_link=PowerBall1_Link.A(0);

Arm_Link1=Link([0 0 0 0 0]);
Arm_Link1.offset(1)=pi;
arm_link1=Arm_Link1.A(0);

Gripper_Link=Link([0 0.05 0 0 0]);
Gripper_Link.offset(1)=pi;
gripper_link=Arm_Link1.A(0);
robot.fkine(zeros(1,10))

% Second ARM Config
H2=[0 1.0066-0.386 0 pi/2 0;%TORSO
    0 0.24464 0 pi/2 0;%ARM
    0 0 0 -pi/2 0;%ARM - pi/2 for the other arm
   0 0.69825-0.24464 0 pi/2 0;
   0 0 0 -pi/2 0;
   0 0.99684-0.69825 0 pi/2 0;
   0 0 0 -pi/2 0;
   0 0.1 0 0 0;];

n=size(H2)

for i=1:1:n(1)
    L(i)=Link(H2(i,:));
end

robot2=SerialLink(L, 'name', 'schunk2')

%%Calculate all homogenoues transformations for each joint for the simulink
%%model
a2=[];
for i=1:1:n(1)
    a2=cat(n(1),a2,robot2.A(i,zeros(1,n(1))));
end
Shoulder2_Link=Link([0 0.139 0 0 0]);
Shoulder2_Link.offset(1)=0;
shoulder2_link=Shoulder2_Link.A(0);
