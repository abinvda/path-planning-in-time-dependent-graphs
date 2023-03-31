%% Plots
clear
load("results.mat")

%% Computation Time

x = [1, 2, 3]; % x values

% Data for the four groups of bars
timeCai = results{2}(:,1);
timeOurs = results{3}(:,1);
timeNoBud = results{4}(:,1);
timeNoRef = results{5}(:,1);

% Plot the bar graph
bar(x, [timeCai, timeNoBud, timeNoRef, timeOurs]);

% Add labels and legend
xlabel('X Values');
ylabel('Y Values');
legend('Cai', 'No Budget', 'No Refinement', 'Ours');
