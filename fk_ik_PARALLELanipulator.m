%kinematics for 3RRR parallel manipulator
%ini variables for 3RRR parallel manipulator
	R = 290;
	a = 0;
	r = 130;
	xc = 230;
	SA = 170;
	L = 130;
	 
	%ini plots variables 
	figure;
	axis([-150 550 -100 550])
	grid on
	title('Simulated Parallel robot workspace')	
    xlabel('mm');
	ylabel('mm');
	hold on
    %calculating workspace and initiating plot loop
for yc = -100:5:400
	   for xc = -100:5:400
	     %base coordinates
	    PB1 = [0 0];
        PB2 = [sqrt(3)*R 0];
	    PB3 = [sqrt(3)*R/2 3*R/2];
	 
	%define platform orientation
	    phi1 = a + pi/6; 
	    phi2 = a + 5*pi/6;
        phi3 = a + 3*pi/2	 
	 
	 
    %calculating ik for rb1
        r1 = (yc - r*sin(phi1)- PB1(:,2));
	    t1 = (xc - r*cos(phi1)- PB1(:,1));
        c1 = atan2(r1,t1);
        u1 = (xc - r*cos(phi1)- PB1(:,1))^2  + (yc - r*sin(phi1)- PB1(:,2))^2;
	    i1 = SA^2 - L^2 + u1;
        w1 = 2 * SA * sqrt(u1);
	    d1 = acos(i1/w1);
	    theta1 = (c1+d1);
	 
	 
	 %calculating ik for rb2
	   r2 = (yc - r*sin(phi2) - PB2(:,2));
	   t2 = (xc - r*cos(phi2) - PB2(:,1));
	   c2 = atan2(r2,t2);
	   u2 = (xc - r*cos(phi2)- PB2(:,1))^2  + (yc - r*sin(phi2)- PB2(:,2))^2;
	   i2 = SA^2 - L^2 + u2;
	   w2 = 2 * SA * sqrt(u2);
	   d2 = acos(i2/w2);
	   theta2 =(c2+d2);
	 
	 %calculating ik for rb3
	   r3 = (yc - r*sin(phi3) - PB3(:,2));
	   t3 = (xc - r*cos(phi3) - PB3(:,1));
	   c3 = atan2(r3,t3);
	   u3 = (xc - r*cos(phi3) - PB3(:,1))^2 + (yc - r*sin(phi3) - PB3(:,2))^2;
	   i3 = SA^2 - L^2 + u3;
	   w3 = 2 * SA * sqrt(u3);
	   d3 = acos(i3/w3);
	   theta3 = (c3+d3);
	        
   if(isreal([theta1 theta2, theta3]))
        rand1=isreal(theta1);
        rand2=isreal(theta2);
	    rand3=isreal(theta3);
	    disp(rad2deg(theta1))
	    disp(rad2deg(theta2))
	    disp(rad2deg(theta3))
	 
    %defining serial link coordinates
        M1x = SA*cos(theta1);
        M1y = SA*sin(theta1);
        M2x = PB2(:,1) + SA*cos(theta2);
        M2y = PB2(:,2) + SA*sin(theta2);
        M3x = PB3(:,1) + SA*cos(theta3);
        M3y = PB3(:,2) + SA*sin(theta3);
	 
	 %costants for least squares
        z1 =  (sqrt(3)*r*cos(a))-M2x;
        z2 =  (sqrt(3)*r*sin(a))-M2y;
        z3 =  (sqrt(3)*r*cos(a+pi/3))-M3x;
        z4 =  (sqrt(3)*r*sin(a+pi/3))-M3y;
	 
	%%calculating PPi values using least squares

        A = [-2*z1-2*M1x  -2*z2-2*M1y; -2*z3-2*M1x  -2*z4-2*M1y];     
        b = [-M1x^2 - M1y^2 + z1^2 + z2^2; -M1x^2 - M1y^2 + z3^2 + z4^2];
        X = A\b;
        p1x = X(1,:);
        p1y = X(2,:);
        p2x = X(1,:) + (sqrt(3)*r*cos(a));
        p2y = X(2,:) + (sqrt(3)*r*sin(a));
        p3x = X(1,:) + (sqrt(3)*r*cos(a+pi/3));
        p3y = X(2,:) + (sqrt(3)*r*sin(a+pi/3));
	          
	   %plotting for pr robot
    x = [PB1(:,1) PB2(:,1) PB3(:,1) 0];
        y = [0 PB2(:,2) PB3(:,2) 0];
        x1 = [PB1(:,1) M1x];
        y1 = [PB1(:,2) M1y];
        x2 = [PB2(:,1) M2x];
        y2 = [PB2(:,2) M2y];
        x3 = [PB3(:,1) M3x];
    y3 = [PB3(:,2) M3y];
        p1 = [M1x p1x];
        q1 = [M1y p1y];
        p2=  [M2x p2x];
    q2 = [M2y p2y];
        p3 = [M3x p3x];
        q3 = [M3y p3y];
        t1 = [p1x p2x];
        t2 = [p1y p2y];
        t3 = [p2x p3x];
        t4 = [p2y p3y];
        t5 = [p1x p3x];
        t6 = [p1y p3y];
 
     plot(x,y,'k')
     plot(xc,yc,'o')
     hold off 
	        end
       end
end  



