% E is synonymous with the axes, so there is no need for plot_axes(trf_E_axes, 'E');
clear all
close all
clf
handle_axes= axes('XLim', [-0.4,0.4], 'YLim', [-0.2,0.4], 'ZLim', [0,0.4]);

xlabel('e_1'); 
ylabel('e_2');
zlabel('e_3');

view(-130, 26);
grid on;
axis equal
camlight
axis_length= 0.05;

%% Root frame E
trf_E_axes= hgtransform('Parent', handle_axes); 
% The root-link transform should be created as a child of the axes from the
% beginning to avoid the error "Cannot set property to a deleted object".
% E is synonymous with the axes, so there is no need for plot_axes(trf_E_axes, 'E');

%%A1 Main Body 

trf_link0_E= make_transform([0, 0, 0], 0, 0, 90, trf_E_axes);  
plot_axes(trf_link0_E, 'L_0', false, axis_length); 

A1= make_transform([0, 0, -0.04], 0, 0, 0, trf_link0_E); %Used for Robot Translation 
length0= 0.05; radius0= 0.035;                           %Used for Scaling 
h(1)= link_cylinder(radius0, length0, A1, [ 1 0 1]);     %To change the colour 
%plot_axes(A1, ' ', true, axis_length); % V_0

%%A2 Ctlinder 1 
A2= make_transform([0, 0, 0], 0, 0, 0); 
length1= 0.1; radius1= 0.02;
h(2)= link_cylinder(radius1, length1, A2, [0, 0, 1]); 

%%A3 SMALLER JOINT
A3= make_transform([0, 0, 0], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
length1= 0.025; radius1= 0.02; %0.025; radius1= 0.02;
h(2)= link_cylinder(radius1, length1, A3, [1, 0, 1]); 

%%A4 Ctlinder 2
A4= make_transform([0, 0, 0], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
length1= 0.09; radius1= 0.0075;
h(2)= link_cylinder(radius1, length1, A4, [1, 0, 1]); 

%%A5  SMALLER JOINT 2
A5= make_transform([0, 0, 0], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
length1= 0.045; radius1= 0.015;
h(2)= link_cylinder(radius1, length1, A5, [1, 0, 1]);

%%A6 Ctlinder 3
A6= make_transform([0, 0, 0], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
length1= 0.07; radius1= 0.01;
h(2)= link_cylinder(radius1, length1, A6, [0, 0, 1]);

%% A7 Recatangular Joint 
A7= make_transform([0, 0, 0], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
h(3)= link_box([0.05, 0.02, 0.02], A7, [0, 0, 1]); %0.2, 0.02, 0.04
%plot_axes(A7, ' ', true, axis_length); % V_{1-2}

%%A8  SMALLER JOINT 3
A8= make_transform([0, 0, 0], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
length1= 0.025; radius1= 0.0095;
h(2)= link_cylinder(radius1, length1, A8, [1, 0, 1]);

%%A9  SMALL JOINT 4
A9= make_transform([0, 0, 0], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
length1= 0.0075; radius1= 0.0035;
h(2)= link_cylinder(radius1, length1, A9, [1, 0, 1]);

%%A10 Ctlinder 4 (horizontal connector) 
A10= make_transform([0, 0, 0], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
length1= 0.05; radius1= 0.0025;
h(2)= link_cylinder(radius1, length1, A10, [0, 0, 1]);

%%A11 Ctlinder 5 (vertical) 
A11= make_transform([0.02, 0, 0], 0, pi/2, 0); % Do not specify parent yet: It will be done in the joint
length1= 0.04; radius1= 0.0015;  %length1= 0.04; radius1= 0.0015;
h(2)= link_cylinder(radius1, length1, A11, [1, 1, 0])

%%A12 Ctlinder 6 (vertical) 
A12= make_transform([0.04, 0.04, 0.035], 0, 0, pi/2); % Do not specify parent yet: It will be done in the joint
length1= 0.04; radius1= 0.0015;
h(2)= link_cylinder(radius1, length1, A12, [1, 1, 0])

%% Link-End-Effector
link_end= make_transform([0, 0, 0.001], 0, 0, 0); % Do not specify parent yet: It will be done in the joint
h(10)= link_sphere(0.005, link_end, [1, 0, 0]); 





%% Joint 1: Revolute (To rotate the object in a circular manner)
j11_rot_axis_C1= [0,0,1]';
j11_rot_angle= 0; % [-pi/2, pi/2]

trf_joint1_link0= make_transform([0, 0, 0.06], 0, 0, 0, A2); 
C1= make_transform_revolute(j11_rot_axis_C1, j11_rot_angle, trf_joint1_link0); 
make_child(C1, A3);
%plot_axes(B1, 'L_1', false, axis_length); 

% Joint 2:  Revolute
j2_rot_axis_B2= [1,0,0]';
j2_rot_angle= pi/2; % [-pi/2, pi/2]

trf_joint2_link1= make_transform([0, 0.04, 0.005], 0, 0, 0, A3); 
B2= make_transform_revolute(j2_rot_axis_B2, j2_rot_angle, trf_joint2_link1); 
%plot_axes(B1, 'L_1', false, axis_length); 
make_child(B2, A4);



%% Joint 3:Revolute
j3_rot_axis_B3= [0,0,1]';
j3_rot_angle= 0; % [-pi/2, pi/2]

trf_joint1_link0= make_transform([0, -0.015, -0.035], pi/2, 0, 0, A4); 
B3= make_transform_revolute(j3_rot_axis_B3, j3_rot_angle, trf_joint1_link0); 
%plot_axes(B1, 'L_1', false, axis_length);
make_child(B3, A5);

%% Joint 4:Revolute
j4_rot_axis_B4= [1,0,0]';
j4_rot_angle= pi/2; % [-pi/2, pi/2]

trf_joint1_link0= make_transform([0, -0.02, 0.01], 0, 0, 0, A5); 
B4= make_transform_revolute(j4_rot_axis_B4, j4_rot_angle, trf_joint1_link0); 
%plot_axes(B1, 'L_1', false, axis_length);
make_child(B4, A6);

%% Joint 5: Revolute
j5_rot_axis_B5= [1,0,0]';
j5_rot_angle= pi/2; % [-pi/2, pi/2]

trf_joint1_link0= make_transform([0, 0, 0.045], 0, 0, 0, A6); 
B5= make_transform_revolute(j5_rot_axis_B5, j5_rot_angle, trf_joint1_link0); 
%plot_axes(B1, 'L_1', false, axis_length);
make_child(B5, A7);

%% Joint 6: Revolute
j6_rot_axis_B6= [1,0,0]';
j6_rot_angle= 0; % [-pi/2, pi/2]

trf_joint1_link0= make_transform([0, 0, -0.02], 0, pi, 0, A7); 
B6= make_transform_revolute(j6_rot_axis_B6, j6_rot_angle, trf_joint1_link0); 
%plot_axes(B1, 'L_1', false, axis_length);
make_child(B6, A8);

%% Joint 7: Revolute
j7_rot_axis_B7= [1,0,0]';
j7_rot_angle= pi; % [-pi/2, pi/2]

trf_joint1_link0= make_transform([0, 0, 0.015], 0, 0, 0, A8); 
B7= make_transform_revolute(j7_rot_axis_B7, j7_rot_angle, trf_joint1_link0); 
%plot_axes(B1, 'L_1', false, axis_length);
make_child(B7, A9);

%% Joint 8: Revolute
j8_rot_axis_B8= [1,0,0]';
j8_rot_angle= pi/2; % [-pi/2, pi/2]

trf_joint1_link0= make_transform([0, 0, 0.], 0, 0, 0, A9); 
B8= make_transform_revolute(j8_rot_axis_B8, j8_rot_angle, trf_joint1_link0); 
plot_axes(B8, 'L_1', false, axis_length);
make_child(B8, A10);



%% Joint 9 Prismatic (To move the object in a vertical/horizontal manner)
j9_translation_axis_B9= [1,0,0]';
j9_translation= 0; % [-0.04, 0.04]

trf_joint4_link3= make_transform([0, -0.02, 0.02], 0, 0, pi/2, A10); 
B9= make_transform_prismatic(j9_translation_axis_B9, j9_translation, trf_joint4_link3);
% plot_axes(trf_link4_joint4, 'L_4', false, axis_length); 
make_child(B9, A11);

%% Joint 10: Prismatic 
j10_translation_axis_B10= [0,0,1]';
j10_translation= 0; % [-0.04, 0.04]

trf_jointa12= make_transform([0, -0.04, -0.036], 0, 0, 0, A11); 
B10= make_transform_prismatic(j10_translation_axis_B10, j10_translation, trf_jointa12);
% plot_axes(trf_link4_joint4, 'L_4', false, axis_length); 
make_child(B10, A12);

%% Joint: Fixed (Ending)
trf_linkEE_link4= make_transform([0, 0, 0], 0, 0, 0, A12); 
make_child(trf_linkEE_link4, link_end);

% Animation, one joint at a time:

 for q1=[linspace(0, -pi/2, 50), linspace(-pi/2, pi/2, 50), linspace(pi/2, 0, 30)]
    set(handle_axes, 'XLim', [-0.2,0.2], 'YLim', [-0.15,0.3], 'ZLim', [-0.2,0.2]);
    trf_q1= makehgtform('axisrotate', j11_rot_axis_C1, q1);
    set(C1, 'Matrix', trf_q1);
    drawnow;
    pause(0.02);
 end


% %  
 for q2=[linspace(0, -pi/2, 20), linspace(-pi/2, 0, 20), linspace(0, pi/2, 20), linspace(pi/2, 0, 20)]
    set(handle_axes, 'XLim', [-0.2,0.2], 'YLim', [-0.15,0.3], 'ZLim', [-0.2,0.2]);
    trf_q2= makehgtform('axisrotate', j3_rot_axis_B3, q2);
    set(B3, 'Matrix', trf_q2);
    drawnow;
    pause(0.08);
end
%  
%Prismatic joint animation (two vertical rods. moving in z-axis
%simultaneously)
for a2=[linspace(0, 0.018, 30), linspace(0.018, -0.018, 30), linspace(-0.018, 0, 30)]
    set(handle_axes, 'XLim', [-0.2,0.2], 'YLim', [-0.15,0.3], 'ZLim', [-0.2,0.2]);
    trf_a2= makehgtform('translate', j9_translation_axis_B9*a2);
    set(B9, 'Matrix', trf_a2);
    drawnow;
    pause(0.02);
end