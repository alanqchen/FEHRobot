
errData = importdata('ERRORLOG.txt');
dispData = importdata('DISPLOG.txt');
refData = importdata('toRed.txt');
tvec = 0:0.1:1.8;

figure(8)
plot(tvec, errData(:,1), tvec, errData(:,2), tvec, errData(:,3))
xlabel('Time (s)')
ylabel('Error (radians)')
legend('Motor 1 Error', 'Motor 2 Error', 'Motor 3 Error', 'Location', 'northwest')
sum1 = 0;
sum2 = 0;
sum3 = 0;

for i=1:length(dispData)
    sum1 = sum1 + dispData(i,1);
    sum2 = sum2 + dispData(i,2);
    sum3 = sum3 + dispData(i,3);
    totalDisp(i,1) = sum1;
    totalDisp(i,2) = sum2;
    totalDisp(i,3) = sum3;
end

figure(9)

plot(tvec, totalDisp(:,1), '-o', tvec, totalDisp(:,2), '-x', tvec, totalDisp(:,3), '-^', tvec, refData(:,1), '-r.', tvec, refData(:,2), '--r.', tvec, refData(:,3), '-.r.')
legend('Motor 1 Disp', 'Motor 2 Disp', 'Motor 3 Disp', 'Motor 1 Ref', 'Motor 2 Ref', 'Motor 3 Ref', 'Location', 'northwest')
xlabel('Time (s)')
ylabel('Total Displacement (radians)')

steadyStateErrorMotor1 = ((totalDisp(length(dispData),1)-refData(length(dispData),1))/refData(length(dispData),1))*100
steadyStateErrorMotor2 = ((totalDisp(length(dispData),2)-refData(length(dispData),2))/refData(length(dispData),2))*100
steadyStateErrorMotor3 = ((totalDisp(length(dispData),3)-refData(length(dispData),3))/refData(length(dispData),3))*100

overShoot1 = max(totalDisp(:,1))-refData(length(dispData),1)
overShoot2 = max(totalDisp(:,2))-refData(length(dispData),2)
overShoot3 = max(totalDisp(:,3))-refData(length(dispData),3)
textbox = sprintf('%s\n%f\n%f\n%f', 'Steady State Errors (%):', steadyStateErrorMotor1, steadyStateErrorMotor2, steadyStateErrorMotor3);
text(.1, max(max(totalDisp))/2, textbox);
textbox2 = sprintf('%s\n%f\n%f\n%f', 'Overshoots (radians):', overShoot1, overShoot2, overShoot3);
text(.1, max(max(totalDisp))/4, textbox2);
