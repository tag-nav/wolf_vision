//info about 2D homogeneous lines and points: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/BEARDSLEY/node2.html

// clear all 
xdel(winsid());
clear;

//User Tunning params
Nw = 6; //window size
theta_th = %pi/6;
K = 3; //How many std_dev are tolerated to count that a point is supporting a line
r_stdev = 0.1; //ranging std dev

//init
result_lines = [];
line_indexes = [];
corners = [];

//invent a set of points + noise
points = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24  25 26  27 28  29 30  31 32  33  34  35  36  37 38  39  40  41  42  43;
          7 6 5 4 3 2 1 2 3 4  5  6  7  8  9  10 9  8  7  6  5  4  3  3.5 4  4.5 5  5.5 6  6.5 7  7.5 7.4 7.3 7.2 7.1 7  6.9 6.8 6.7 6.6 6.5 6.4];
points = points + rand(points,"normal")*r_stdev;
[xx Np] = size(points);

//Main loop. Runs over a sliding window of Nw points
for (i = Nw:Np)
    //set the window
    points_w = points(:,(i-Nw+1):i)

    //Found the best fitting line over the window. Build the system: Ax=0. Matrix A = a_ij
//    a_00 = sum( points_w(1,:).^2 );
//    a_01 = sum( points_w(1,:).*points_w(2,:) );
//    a_02 = sum( points_w(1,:) );
//    a_10 = a_01;
//    a_11 = sum( points_w(2,:).^2 );
//    a_12 = sum( points_w(2,:) );
//    a_20 = a_02;
//    a_21 = a_12;
//    a_22 = Nw;
//    A = [a_00 a_01 a_02; a_10 a_11 a_12; a_20 a_21 a_22; 0 0 1];
    a_00 = sum( points_w(1,:).^2 );
    a_01 = sum( points_w(1,:).*points_w(2,:) );
    a_02 = sum( points_w(1,:) );
    a_10 = a_01;
    a_11 = sum( points_w(2,:).^2 );
    a_12 = sum( points_w(2,:) );
    A = [a_00 a_01 a_02; a_10 a_11 a_12; 0 0 1];

    //solve
//    line = pinv(A)*[zeros(3,1);1];
    line = inv(A)*[0; 0; 1];

    //compute error
    err = 0;
    for j=1:Nw
        err = err + abs(line'*[points_w(:,j);1])/sqrt(line(1)^2+line(2)^2);
    end
    err = err/Nw;
    //disp("error: "); disp(err);
    
    //if error below stdev, add line to result set
    if err < K*r_stdev then
        result_lines = [result_lines [line;points_w(:,1);points_w(:,$)]];
        line_indexes = [line_indexes i]; //ray index where the segment ends
    end    
end

//num of lines
[xx Nl] = size(result_lines);

//corner detection
for (i = 2:Nl)
    //compute angle diff between consecutive segments
    cos_theta = result_lines(1:2,i-1)'*result_lines(1:2,i) / ( norm(result_lines(1:2,i-1)) * norm(result_lines(1:2,i)) )
    theta = abs(acos(cos_theta));
    
    //if angle diff greater than threshold && indexes are less than Nw, we decide corner
    if theta > theta_th then
        if (line_indexes(i)-line_indexes(i-1)) < Nw then
            //Corner found! Compute "sub-pixel" corner location as the intersection of two lines
            corner = cross(result_lines(1:3,i-1),result_lines(1:3,i))
            corner = corner./corner(3);//norlamlize homogeneous point
            corners = [corners corner];
            
            //display
            disp("theta: "); disp(theta);
            disp("index:" ); disp(line_indexes(i)-Nw+1);//line_indexes(i) indicates the end point of the segment
        end
    end
end
    
//Set figure
fig1 = figure(0);
fig1.background = 8;

//plot points
plot(points(1,:),points(2,:),"g.");

//plot lines
for i=1:Nl
    m = -result_lines(1,i)/result_lines(2,i);
    xc = -result_lines(3,i)/result_lines(2,i);
    point1 = [result_lines(4,i) m*result_lines(4,i)+xc];
    point2 = [result_lines(6,i) m*result_lines(6,i)+xc];
    xpoly([point1(1) point2(1)],[point1(2) point2(2)]);
end

//plot corners
plot(corners(1,:),corners(2,:),"ro");



