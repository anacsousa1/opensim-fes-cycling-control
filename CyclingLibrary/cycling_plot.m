

%% plot
plot_title = strcat('Results for left leg (load = ', num2str(flagLoad), ')');

h(1) = figure('NumberTitle', 'off', 'Name', plot_title);

%muscles
ax1 = subplot(4,1,1); hold on;
plot(controlTime,controlActionQuadRight*flagMusclesQUAD,'b');
plot(controlTime,controlActionQuadLeft*flagMusclesQUAD,'r');
% plot(controlTime,controlActionHamsLeft*flagMusclesHAMS,'r');
% plot(controlTime,controlActionGlutLeft*flagMusclesGLUT,'y');
hold off;
xlabel('time [s]'); ylabel('Muscles');  ylim([0,1.1]);
% legend('Q', 'H', 'G');
legend('Right', 'Left');

%knee spring
ax2 = subplot(4,1,2); hold on;
plot(controlTime,controlActionTorqueRight,'b');
plot(controlTime,controlActionTorqueLeft,'r');
hold off;
xlabel('time [s]'); ylabel('Knee orthosis');  ylim([min(controlActionTorqueRight),max(controlActionTorqueRight)]);

%knee and gear angles
ax3 = subplot(4,1,3); hold on;
plot(controlTime,wrapTo360(rad2deg(-measAngle)),'k');
plot(controlTime,wrapTo360(rad2deg(-angleKneeR)),'b');
plot(controlTime,wrapTo360(rad2deg(-angleKneeL)),'r');
hold off;
xlabel('time [s]'); ylabel('Gear Angle [deg]'); ylim([-10,370]);
legend('gear', 'knee R', 'knee L');

%velocity
ax4 = subplot(4,1,4); hold on;
plot(controlTime,rad2deg(measVel),'b');
plot(controlTime,rad2deg(refVel), 'r'); hold off;
xlabel('time [s]'); ylabel('Angular speed [deg/s]'); ylim([min(rad2deg(measVel)),max(rad2deg(measVel))]);
legend('vel', 'ref');

linkaxes([ax1,ax2,ax3,ax4],'x')
