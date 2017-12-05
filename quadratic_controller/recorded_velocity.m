
Vxd = v_d(1,1:1500);
Vxreal = v_real(1, 1:1500);
t = 1:1500;
t = t/80.0;
subplot(3,1,1);
plot(t, Vxd, 'r');
hold on;
plot(t, Vxreal, 'b');
line([13.5 13.5], [-0.05 0.05], 'Color','[0,0,0]', 'LineStyle','--');
line([14.3 14.3], [-0.05 0.05], 'Color','[0,0,0]', 'LineStyle','--');
lgd1 = legend('v_{desired}(1)','v_{actual}(1)', 'Location','NorthWest');
%legend({'A','B'},'Position',[0.2 0.6 0.1 0.2])
lgd1.FontSize = 10;

Vyd = v_d(2,1:1500);
Vyreal = v_real(2, 1:1500);
t1 = 1:1500;
t1 = t1/50.0;
subplot(3,1,2);
plot(t1, Vyd, 'r');
ylabel('v_{desired} vs. v_{actual} (m/s)');
hold on;
plot(t1, Vyreal, 'b');
line([28.5 28.5], [-0.05 0.05], 'Color','[0,0,0]', 'LineStyle','--');
line([29.5 29.5], [-0.05 0.05], 'Color','[0,0,0]', 'LineStyle','--');
lgd2 = legend('v_{desired}(2)','v_{actual}(2)', 'Location','NorthWest');
%legend({'A','B'},'Position',[0.2 0.6 0.1 0.2])
lgd2.FontSize = 10;

Vzd = v_d(3,1:1500);
Vzreal = v_real(3, 1:1500);
subplot(3,1,3);
plot(t, Vzd, 'r');
xlabel('time (sec)');
hold on;
plot(t, Vzreal, 'b');
lgd3 = legend('v_{desired}(3)','v_{actual}(3)', 'Location','NorthWest');
%legend({'A','B'},'Position',[0.2 0.6 0.1 0.2])
lgd3.FontSize = 10;
%axis([0 30 -0.01 0.04])
