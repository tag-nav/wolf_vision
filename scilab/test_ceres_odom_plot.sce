//plot log data from ceres test

// clear all 
xdel(winsid());
clear;

// CERES ODOM BATCH
//load log file
data = read('~/Desktop/log_file_2.txt',-1,14);

//plot
fig1 = figure(0);
fig1.background = 8;
plot(data(2:$,10),data(2:$,11),"g.");
plot(data(2:$,1),data(2:$,2),"b-");
plot(data(2:$,4),data(2:$,5),"r-");
plot(data(2:$,13),data(2:$,14),"c--");

ah = gca();
ah.auto_scale = "on";
ah.x_label.text = "$x [m]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$y [m]$";
ah.y_label.font_size = 4;
lh =legend(["$GPS$";"$Optimization$";"$Ground\ Truth$";"$ODOM$"],4);
lh.font_size = 3;
title(strcat(["CERES_ODOM_BATCH - Time: ",string(data(1,1))," s"]));
ah.title.font_size = 4;

// MANAGER - THETA
//load log file
data2 = read('~/Desktop/log_file_2.txt',-1,15);
data2L = read('~/Desktop/landmarks_file_2.txt',-1,2);

disp(data2L);

//plot
fig2 = figure(1);
fig2.background = 8;
//plot(data2(2:$,13),data2(2:$,14),"g.");
plot(data2(2:$,1),data2(2:$,2),"b-");
plot(data2(2:$,4),data2(2:$,5),"r-");
plot(data2(2:$,10),data2(2:$,11),"c--");

plot(data2L(1:$,1),data2L(1:$,2),"k.");

ah = gca();
ah.auto_scale = "on";
ah.x_label.text = "$x [m]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$y [m]$";
ah.y_label.font_size = 4;
lh =legend(["$Optimization$";"$Ground\ Truth$";"$ODOM$";"$Landmarks$"],4);
lh.font_size = 3;                             
title(strcat(["CERES_MANAGER: Theta - Time: ",string(data2(1,1))," s"]));
ah.title.font_size = 4;

// MANAGER - COMPLEX ANGLE
//load log file
data3 = read('~/Desktop/log_file_3.txt',-1,15);

//plot
fig3 = figure(2);
fig3.background = 8;
plot(data3(2:$,13),data3(2:$,14),"g.");
plot(data3(2:$,1),data3(2:$,2),"b-");
plot(data3(2:$,4),data3(2:$,5),"r-");
plot(data3(2:$,10),data3(2:$,11),"c--");

ah = gca();
ah.auto_scale = "on";
ah.x_label.text = "$x [m]$";
ah.x_label.font_size = 4;
ah.y_label.text = "$y [m]$";
ah.y_label.font_size = 4;
lh =legend(["$GPS$";"$Optimization$";"$Ground\ Truth$";"$ODOM$"],4);
lh.font_size = 3;                             
title(strcat(["CERES_MANAGER: Complex Angle - Time: ",string(data3(1,1))," s"]));
ah.title.font_size = 4;

