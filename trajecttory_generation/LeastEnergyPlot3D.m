% Initializing Plots
h_fig = figure;h_3d = gca;
axis equal;grid on;view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(1);
set(gcf,'Renderer','OpenGL');

for iter = 1:length(state2)-1
    if iter == 1
        QP = QuadPlotnew(1, state2(1,:), p.L, 0.04, quadcolors(1,:), length(state2), h_3d);
        QP.UpdateQuadPlotnew(state2(1,:), [[0;0;0]; [0;0;0]], 0);
        h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time2(iter)));
    end
    QP.UpdateQuadPlotnew(state2(iter+1, :)', [[0;0;0]; [0;0;0]], time2(iter));
    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time2(iter)))
end
%


