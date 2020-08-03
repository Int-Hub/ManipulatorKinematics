%%inverse.m
%%%This function calculates the inverse of a transformation matrix
function matrix = inverse(x)
    if size(x) ~= [4,4]
        display("Matrix is of invalid length");
        return
    else
        tempRot = (x(1:3, 1:3))';
        tempTrans = - tempRot * x(1:3, 4);
        
        matrix(1:3,1:3) = tempRot;
        matrix(1:3, 4) = tempTrans;
        matrix(4,4) = 1;
        matrix(4,1:3) = 0;     
end

%%plotWorkspace.m
%%%This piece of code plots the 2D or 3D workspace of the manipulator
 
%%%Link length variables
a2=1; a3=1; a4=1;
 
%index for plot points matrix column
i = 1;    
 
%%%Nested for loops to iterate through joint angles
%%%for 2D plot change first for loop to b = 0:180:180 and increase other
%%%the granularity of other loops
      for b = 0:5:180
           for c = 0:16:180
              for d = 0:16:180
                for e = 0:16:180
                    
                    %Calculate only the translation of the homogenous
                    %transformation matrix
                  T0e = [ cosd(b)*(a3*cosd(c+d) + a2*cosd(c) + a4*cosd(c+d+e));
                          sind(b)*(a3*cosd(c+d) + a2*cosd(c)+ a4*cosd(c+d+e));
                               a3*sind(c+d) + a2*sind(c) + a4*sind(c+d+e);
1	];
2	
                   %Assign points to plot-variable
                   coord(i,:) = [T0e(1), T0e(2), T0e(3)];
                   
                   i = i+1;
           
                end 
              end      
           end 
      end
      
      %%%plot the points
      hold on;
      grid on;  
      xlabel("X-axis");
      ylabel("Y-axis");
      zlabel("Z-axis");
      scatter3(coord(:,1), coord(:,2), coord(:,3), "r","*");
      scatter3(coord(:,1), coord(:,2), coord(:,3), "b","o");
      scatter3(coord(:,1), coord(:,2), coord(:,3), "g", ".");
      scatter3(0,0,0, "filled", "k");
      text(0.1, 0.1,0.1, "Origin");
            

%%invk.m
%%% this function carries out the IK and returns the joint variabe values
function angles = invk(T)
    n1 = T(1,1); o1 = T(1,2); g1 = T(1,3); px = T(1,4);
    n2 = T(2,1); o2 = T(2,2); g2 = T(2,3); py = T(2,4);
    n3 = T(3,1); o3 = T(3,2); g3 = T(3,3); pz = T(3,4);
 
    a2 = 1; %5.75*0.0254; 
    a3 = 1; %7.375*0.0254; 
    a4 = 1; %2.75*0.0254;
 
    Q1 = atan2(py,px);  c1 = cos(Q1);  s1 = sin(Q1);
    
    Q234 = atan(g3/(c1*g1 + g2*s1));
    c234 = cos(Q234); s234 = sin(Q234);
    
    c3 = (((px*c1 + py*s1 - a4*c234)^2) + ((pz - a4*s234)^2) - (a2^2) - (a3^2))/(2*a2*a3);
    s3 = nthroot((1 - c3^2), 1);  Q3 = atan2(s3,c3);
    
    Q2 = atan2(((c3*a3 + a2)*(pz - s234*a4)-s3*a3*(px*c1 + py*s1 - c234*a4)), ((c3*a3 + a2)*(px*c1 + py*s1- c234*a4)+ s3*a3*(pz - s234*a4)));
    
    Q4 = Q234 - (Q2 + Q3);
    
    Q5 = atan2((s234*(o1*c1 + o2*c1) - o3*c234), (n3*c234 - s234*(n1*c1 - n2*s1)));
    
    angles = [rad2deg(Q1), rad2deg(Q2) rad2deg(Q3) rad2deg(Q4) rad2deg(Q5)];
    
end


%%lintraj.m
%%%This function calculates the linear trajectory for the 5 points using
%%%linspace()
function traj = lintraj(pt, num)
    traj(1:num, 1) = linspace(pt(1,1), pt(2,1), num);
    traj(1:num, 2) = linspace(pt(1,2), pt(2,2), num);
    traj(1:num, 3) = linspace(pt(1,3), pt(2,3), num);
    traj(num+1 : 2*num, 1) = linspace(pt(2,1), pt(3,1), num);
    traj(num+1 : 2*num, 2) = linspace(pt(2,2), pt(3,2), num);
    traj(num+1 : 2*num, 3) = linspace(pt(2,3), pt(3,3), num);
    traj((2*num)+1 : 3*num, 1) = linspace(pt(3,1), pt(4,1), num);
    traj((2*num)+1 : 3*num, 2) = linspace(pt(3,2), pt(4,2), num);
    traj((2*num)+1 : 3*num, 3) = linspace(pt(3,3), pt(4,3), num);    
    traj((3*num)+1 : 4*num, 1) = linspace(pt(4,1), pt(5,1), num);
    traj((3*num)+1 : 4*num, 2) = linspace(pt(4,2), pt(5,2), num);
    traj((3*num)+1 : 4*num, 3) = linspace(pt(4,3), pt(5,3), num);    
end 




%%trsfm.h
%%%This equation computes the forward kinematics for the given set of joint
%%%variables Q
function  temp = trsfm(Q,n)
  
pi_ = 90;
rot_x = [pi_ 0 0 pi_ 0];
 
for i = 1:n  
    %link lenght variables
    a2=1; a3=1; a4=1
                                 
    T_01 = [cosd(Q(i,1)) -sind(Q(i,1))*cosd(rot_x(1)) sind(Q(i,1))*sind(rot_x(1))                                                                                                                                                                                                                                                                                                0;
            sind(Q(i,1))  cosd(Q(i,1))*cosd(rot_x(1))  -cosd(Q(i,1))*sind(rot_x(1))     0;
            0                 sind(rot_x(1))        cosd(rot_x(1))             0;
            0                       0                     0                   1 ];
 
    T_12 = [ cosd(Q(i,2)) -sind(Q(i,2))*cosd(rot_x(2)) sind(Q(i,2))*sind(rot_x(2)) a(2)*cosd(Q(i,2));
             sind(Q(i,2))  cosd(Q(i,2))*cosd(rot_x(2)) -cosd(Q(i,2))*sind(rot_x(2))   a(2)*sind(Q(i,2));
               0                 sind(rot_x(2))                   cosd(rot_x(2))                       0;
               0                       0                                 0                             1       											];
 
    T_23 = [ cosd(Q(i,3)) -sind(3*d)*cosd(Q(i,3)) sind(Q(i,3))*sind(rot_x(3))                             a(3)*cosd(Q(i,3));
             sind(Q(i,3))  cosd(3*d)*cosd(Q(i,3))  -cosd(Q(i,3))*sind(rot_x(3))                             a(3)*sind(Q(i,3));
                 0            sind(rot_x(3))             cosd(rot_x(3))                     0;
                 0                       0                                  0                             1       											];
 
    T_34 = [ cosd(Q(i,4)+pi_) -sind(Q(i,4)+pi_)*cosd(rot_x(4)) sind(Q(i,4)+pi_)*sind(rot_x(4))   a(4)*cosd(Q(i,4));
                             
             sind(Q(i,4)+pi_)  cosd(Q(i,4)+pi_)*cosd(rot_x(4))  -cosd(Q(i,4)+pi_)*sind(rot_x(4))   a(4)*sind(Q(i,4));
                    0                 sind(rot_x(4))                                                                                                   cosd(rot_x(4))                    0;
                     0                       0                                 0                          1             ];
 
    T_4e = [ cosd(rot_z(5)) -sind(rot_z(5))*cosd(rot_x(5))  sind(rot_z(5))*sind(rot_x(5))            0;
             sind(rot_z(5))  cosd(rot_z(5))*cosd(rot_x(5))  -cosd(rot_z(5))*sind(rot_x(5))            0;
                                 0                 sind(rot_x(5))                   cosd(rot_x(5))                    0;
                                 0                       0                                 0                          1       ];
 
    T0e = T_01*T_12*T_23*T_34*T_4*e;


%%%This can be used to directly calculate the transformation matrix from th base to the end effector                    
%     T0e = [ sin(Q(i,1))*sin(Q(i,5)) + cos(Q(i,1))*cos(Q(i,5))*cos(Q(i,2) + Q(i,3) + Q(i,4)),   cos(Q(i,5))*sin(Q(i,1))-cos(Q(i,1))*cos(Q(i,2) + Q(i,3) + Q(i,4))*sin(Q(i,5)), cos(Q(i,1))*sin(Q(i,2) + Q(i,3) + Q(i,4)), cos(Q(i,1))*(a3*cos(Q(i,2) + Q(i,3)) + a2*cos(Q(i,2)) + a4*cos(Q(i,2) + Q(i,3) + Q(i,4)));
%            cos(Q(i,5))*cos(Q(i,2) + Q(i,3) + Q(i,4))*sin(Q(i,1)) - cos(Q(i,1))*sin(Q(i,5)),   -cos(Q(i,1))*cos(Q(i,5))-cos(Q(i,2) + Q(i,3) + Q(i,4))*sin(Q(i,1))*sin(Q(i,5)), sin(Q(i,1))*sin(Q(i,2) + Q(i,3) + Q(i,4)), sin(Q(i,1))*(a3*cos(Q(i,2) + Q(i,3)) + a2*cos(Q(i,2)) + a4*cos(Q(i,2) + Q(i,3) + Q(i,4)));
%                                       cos(Q(i,5))*sin(Q(i,2) + Q(i,3) + Q(i,4)),                            -sin(Q(i,5))*sin(Q(i,2) + Q(i,3) + Q(i,4)),        -cos(Q(i,2) + Q(i,3) + Q(i,4)),           a3*sin(Q(i,2) + Q(i,3)) + a2*sin(Q(i,2)) + a4*sin(Q(i,2) + Q(i,3) + Q(i,4));
%                                                               0,                                                     0,                         0,                                                            1  ];
     temp(i,:) = T0e(1:3, 4);

  end
 end


%%iksyms.m
%%%This piece of code is used to obtain the homogenous transformation
%%%matrix and the linear systems of equatins that allow for solving for the
%%%joint variables
 
%%% Symbolic variables
%%% Q1-5 and Q5 represent the different twitter matrices
syms Q1 Q2 Q3 Q4 Q5 Q234 a2 a3 a4 n1 o1 g1 px n2 o2 g2 py n3 o3 g3 pz p s234  s1 s2 s3 s4 s5 c234 c1 c2 c3 c4 c5 real 
 
% Constants
%a2 = 5.75*0.0254;
%a3 = 7.375*0.0254;
%a4 = 2.75*0.0254;
%p = pi/2; 
 
%%%Symbolic Homogeneous Transformation Matrix
FT = [ n1 o1 g1 px;
       n2 o2 g2 py;
       n3 o3 g3 pz;
       0  0  0  1  ];
 
% Transformations
T1 = [ cos(Q1) -sin(Q1)*cos(p)  sin(Q1)*sin(p)   0;
       sin(Q1)  cos(Q1)*cos(p) -cos(Q1)*sin(p)   0;
           0        sin(p)          cos(p)       0;
           0          0               0          1  ];
T2 = [ cos(Q2) -sin(Q2)*cos(0)  sin(Q2)*sin(0)   a2*cos(Q2);
       sin(Q2)  cos(Q2)*cos(0) -cos(Q2)*sin(0)   a2*sin(Q2);
          0          sin(0)          cos(0)           0;
          0            0               0              1     ];
T3 = [ cos(Q3) -sin(Q3)*cos(0)  sin(Q3)*sin(0)   a3*cos(Q3);
       sin(Q3)  cos(Q3)*cos(0) -cos(Q3)*sin(0)   a3*sin(Q3);
          0          sind(0)         cos(0)         0;
          0             0              0            1       ];                        
T4 = [ cos(Q4) -sin(Q4)*cos(p)  sin(Q4)*sin(p)   a4*cos(Q4);
       sin(Q4)  cos(Q4)*cos(p) -cos(Q4)*sin(p)   a4*sin(Q4);
            0          sin(p)            cos(p)     0;
            0             0                  0      1       ];
Te = [ cos(Q5) -sin(Q5)*cos(0)  sin(Q5)*sin(0)   0;
       sin(Q5)  cos(Q5)*cos(0) -cos(Q5)*sin(0)   0;
            0          sin(0)       cos(0)       0;
            0             0            0         1 ]; 
        
%%%Final Transformation Matrix                         
T0e =  T1 * T2 * T3 * T4 * Te;
T0e = simplify(subs(T0e, p, pi/2))
% T0e = simplify(subs(T0e, sin(Q2+Q3+Q4), s234));
% T0e = simplify(subs(T0e, cos(Q2+Q3+Q4), c234));
% T0e = simplify(subs(T0e, cos(Q1), c1));
% T0e = simplify(subs(T0e, sin(Q1), s1));
% T0e = simplify(subs(T0e, sin(Q2), s2));
% T0e = simplify(subs(T0e, cos(Q2), c2));
% T0e = simplify(subs(T0e, sin(Q3), s3));
% T0e = simplify(subs(T0e, cos(Q3), c3));
% T0e = simplify(subs(T0e, sin(Q4), s4));
% T0e = simplify(subs(T0e, cos(Q4), c4));
% T0e = simplify(subs(T0e, sin(Q5), s5));
% T0e = simplify(subs(T0e, cos(Q5), c5))
 
%%%Uncomment if trying to solve for joint angles
% inv1 = inverse(T1);
% inv2 = inverse(T2);
% inv3 = inverse(T3);
% inv4 = inverse(T4);
 
%%%This block is used to obtain the linear system of equations used
%%%to solve IK joint variable algebraically
% LHS1 = inv1*FT;
% LHS1 = simplify(subs(LHS1, p, pi/2));
% LHS1 = simplify(subs(LHS1, sin(Q2+Q3+Q4), s234));
% LHS1 = simplify(subs(LHS1, cos(Q2+Q3+Q4), c234));
% LHS1 = simplify(subs(LHS1, cos(Q1), c1));
% LHS1 = simplify(subs(LHS1, sin(Q1), s1));
% LHS1 = simplify(subs(LHS1, sin(Q2), s2));
% LHS1 = simplify(subs(LHS1, cos(Q2), c2));
% LHS1 = simplify(subs(LHS1, sin(Q3), s3));
% LHS1 = simplify(subs(LHS1, cos(Q3), c3));
% LHS1 = simplify(subs(LHS1, sin(Q3), s4));
% LHS1 = simplify(subs(LHS1, cos(Q3), c4))
% RHS1 = simplify(subs((T2*T3*T4*Te), p, pi/2));                                                                                                   
% RHS1 = simplify(subs(RHS1, sin(Q2+Q3+Q4), s234));
% RHS1 = simplify(subs(RHS1, cos(Q2+Q3+Q4), c234));
% RHS1 = simplify(subs(RHS1, cos(Q1), c1));
% RHS1 = simplify(subs(RHS1, sin(Q1), s1));
% RHS1 = simplify(subs(RHS1, cos(Q2), c2));
% RHS1 = simplify(subs(RHS1, sin(Q2), s2));
% RHS1 = simplify(subs(RHS1, sin(Q3), s3));
% RHS1 = simplify(subs(RHS1, cos(Q3), c3));
% RHS1 = simplify(subs(RHS1, sin(Q3), s4));
% RHS1 = simplify(subs(RHS1, cos(Q3), c4))
 
% LHS4 = inv4*inv3*inv2*inv1*FT;
% LHS4 = simplify(subs(LHS4, p, pi/2));
% LHS4 = simplify(subs(LHS4, sin(Q2+Q3+Q4), s234));
% LHS4 = simplify(subs(LHS4, cos(Q2+Q3+Q4), c234));
% LHS4 = simplify(subs(LHS4, cos(Q1), c1));
% LHS4 = simplify(subs(LHS4, sin(Q1), s1));
% LHS4 = simplify(subs(LHS4, sin(Q2), s2));
% LHS4 = simplify(subs(LHS4, cos(Q2), c2));
% LHS4 = simplify(subs(LHS4, sin(Q3), s3));
% LHS4 = simplify(subs(LHS4, cos(Q3), c3));
% LHS4 = simplify(subs(LHS4, sin(Q3), s4));
% LHS4 = simplify(subs(LHS4, cos(Q3), c4))
% RHS4 = simplify(subs((Te), p, pi/2));                                                                                                   
% RHS4 = simplify(subs(RHS4, sin(Q2+Q3+Q4), s234));
% RHS4 = simplify(subs(RHS4, cos(Q2+Q3+Q4), c234));
% RHS4 = simplify(subs(RHS4, cos(Q1), c1));
% RHS4 = simplify(subs(RHS4, sin(Q1), s1));
% RHS4 = simplify(subs(RHS4, cos(Q2), c2));
% RHS4 = simplify(subs(RHS4, sin(Q2), s2));
% RHS4 = simplify(subs(RHS4, sin(Q3), s3));
% RHS4 = simplify(subs(RHS4, cos(Q3), c3));
% RHS4 = simplify(subs(RHS4, sin(Q3), s4));
% RHS4 = simplify(subs(RHS4, cos(Q3), c4))
getlintraj.m

%%This function generates the joint angles for a given set of linear trajectory points. 
function jtangles = getlintraj(traj,n)
 
%%loop through set of trajectory points
for i = 1:n
        %Compute rotation matrix given the trajectory points
        trans(1:3,1:3) = [                              1 0 0; 
                           0 cos(atan2(traj(i,3),traj(i,2))) -sin(atan2(traj(i,3),traj(i,2)));
                           0 sin(atan2(traj(i,3),traj(i,2))) cos(atan2(traj(i,3),traj(i,2)))] * [ cos(atan2(traj(i,3),traj(i,1))) 0 sin(atan2(traj(i,3),traj(i,1)));
                                                                                                                                0 1 0; 
                                                                                                  -sin(atan2(traj(i,3),traj(i,1))) 0 cos(atan2(traj(i,3),traj(i,1)))  ] * [ cos(atan2(traj(i,2),traj(i,1))) -sin(atan2(traj(i,2),traj(i,1))) 0;
                                                                                                                                                                           -sin(atan2(traj(i,2),traj(i,1)))  cos(atan2(traj(i,2),traj(i,1))) 0; 
                                                                                                                                                                                                          0 0 1                                   ]; 
                                                    
        trans(1:3, 4) = traj(i, 1:3);
        trans(4, :) = [0 0 0 1]
        
        %Obtain the joint angles using IK function
        temp = invk(trans);
        jtangles(i, 1) = temp(1); 
        jtangles(i, 2) = temp(2); 
        jtangles(i, 3) = temp(3);
        jtangles(i, 4) = temp(4);
        jtangles(i, 5) = temp(5);
end
end

%%cubictraj.m
%%This function takes in the joint variables corresonding to the desired
%%endffector positions to be moved to
function angles = cubictraj(a1,a2,a3,a4,a5)
    %%%Position 1 to 2 %%%
    %Q1 
    Q = [a1(1); 0; a2(1); 0];
    for i = 1:10 
        a12_1(i) = Q(1) + (3*(Q(3)-Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end
    % Q2
    Q = [a1(2); 0; a2(2); 0];
    for i = 1:10 
        a12_2(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end 
    %Q3
    %T = [ 1 0 0 0; 0 1 0 0; 1 1 1 1; 0 1 2 3 ];
    Q = [a1(3); 0; a2(3); 0];
    for i = 1:10 
        a12_3(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end
    %Q4
    Q = [ a1(4); 0; a2(4); 0 ];
    for i = 1:10 
        a12_4(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end 
    
    %%%Position 2 to 3%%%
    %Q1 
    Q = [a2(1); 0; a3(1); 0];
    for i = 1:10 
        a23_1(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end
    % Q2
    Q = [a2(2); 0; a3(2); 0];
    for i = 1:10 
        a23_2(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end 
    %Q3
    Q = [a2(3); 0; a3(3); 0];
    for i = 1:10 
        a23_3(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end
    %Q4
    Q = [a2(4); 0; a3(4); 0];
    for i = 1:10 
        a23_4(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end 
    
    %%%Position 3 to 4%%%
    %Q1 
    Q = [a3(1); 0; a4(1); 0];
    for i = 1:10 
        a34_1(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end
    % Q2
    Q = [a3(2); 0; a4(2); 0];
    for i = 1:10 
        a34_2(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end 
    %Q3
    Q = [a3(3); 0; a4(3); 0];
    for i = 1:10 
        a34_3(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end
    %Q4
    Q = [a3(4); 0; a4(4); 0];
    for i = 1:10 
        a34_4(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end 
    
    %%%Position 4 to 5%%%
    %Q1 
    Q = [a4(1); 0; a5(1); 0];
    for i = 1:10 
        a45_1(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end
    % Q2
    Q = [a4(2); 0; a5(2); 0];
    for i = 1:10 
        a45_2(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end 
    %Q3
    Q = [a4(3); 0; a5(3); 0];
    for i = 1:10 
        a45_3(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end
    %Q4
    Q = [a4(4); 0; a5(4); 0];
    for i = 1:10 
        a45_4(i) = Q(1) + (3*(Q(3) - Q(1)))*(i*0.1)^2 + (2*(Q(1) - Q(3)))*(i*0.1)^3;
    end 
    
    Q1 = [a12_1 a23_1 a34_1 a45_1];
    Q2 = [a12_2 a23_2 a34_2 a45_2];
    Q3 = [a12_3 a23_3 a34_3 a45_3];
    Q4 = [a12_4 a23_4 a34_4 a45_4];
    
    angles = [Q1' Q2' Q3' Q4'];
    angles(:, 5) = 0;
end
