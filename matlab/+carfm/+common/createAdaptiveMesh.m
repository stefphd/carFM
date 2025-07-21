function tmesh = createAdaptiveMesh(t,v,N,mr,ms,tl,th,cf)
%createAdaptiveMesh Create mesh adaptive
% t: independent variable
% v: value to use for adaptive mesh
% N: final number of mesh pts
% mr: mesh ratio
% ms: min sector len
% tl: transient len
% th: treshold
% cf: plot control figure

% Identify high v sectors
v = abs(v);
iH = v >= (th*rms(v));
iHi = find(diff(iH)==1);
iHf = find(diff(iH)==-1);
if iHi(1)>iHf(1) % starts with v>th: add index 1 to iHi
    iHi = [1; iHi];
end
if iHi(end)>iHf(end) % ends with v>th: add index numel(t) to iHf
    iHf = [iHf; numel(t)];
end
tH = [t(iHi), t(iHf)];
% merge sectors that have a short t between them
while true 
    dtH = tH(2:end,1)-tH(1:end-1,2); 
    idel = dtH<(2*tl+ms);
    if ~any(idel) % loop untill idel is empty
        break;
    end
    sHtmp = [];
    k = 1;
    while true
        if ~idel(k) % cpy
            sHtmp = [sHtmp; tH(k,:)];
            k = k + 1;
        else
            sHtmp = [sHtmp; [tH(k,1), tH(k+1,2)]];
            k = k + 2; % skip next
        end
        if k > numel(idel)
            if ~idel(end) % add last
                sHtmp = [sHtmp; tH(end,:)];
            end
            break;
        end
    end
    tH = sHtmp;
end
% check first and last sector
if (tH(1,1)-t(1))<(tl+ms)
    tH(1,1) = t(1);
end
if (t(end)-tH(end,2))<(tl+ms)
    tH(end,2) = t(end);
end
iH = any((t >= tH(:,1)') & (t <= tH(:,2)'), 2);
if all(iH) || all(~iH)
    tmesh = linspace(t(1),t(end),N);
    return
end
% Identify low v sectors
iL = ~iH;
iLi = find(diff(iL)==1);
iLf = find(diff(iL)==-1);
if iLi(1)>iLf(1)
    iLi = [1; iLi];
end
if iLi(end)>iLf(end)
    iLf = [iLf; numel(v)];
end
tL = [t(iLi), t(iLf)];
% rem transients
tL(2:end,1) = tL(2:end,1) + tl;
tL(1:end-1,2) = tL(1:end-1,2) - tl;
if iLi(1,1)~=1
    tL(1,1) = tL(1,1) + tl;
end
if iLf(end,1)~=numel(t)
    tL(end,2) = tL(end,2) - tl;
end
iL = any((t >= tL(:,1)') & (t <= tL(:,2)'), 2);
% Identify transients
iT = ~(iH | iL);
iTi = find(diff(iT)==1);
iTf = find(diff(iT)==-1);
isLH = iL(iTi);
% L->H transient
iLHi = iTi(isLH)+1;
iLHf = iTf(isLH)+1;
tLH = [t(iLHi), t(iLHf)];
iLH = any((t >= tLH(:,1)') & (t <= tLH(:,2)'), 2);
% H->L transient
iHLi = iTi(~isLH);
iHLf = iTf(~isLH);
tHL = [t(iHLi), t(iHLf)];
iHL = any((t >= tHL(:,1)') & (t <= tHL(:,2)'), 2);

% Construct mesh function
% a function that depends on t and is 1 in high v sectors and mr
% in low v sector, while increase/decrease linearly in transients
h = ones(size(t)); % init to 1
for k = 1 : numel(t)
    % high v
    if iH(k)
        % h(k) is already 1
        continue;
    end
    % low v
    if iL(k)
        h(k) = mr;
        continue;
    end
    % transient
    sk = t(k);
    if iLH(k) % L-H transient
        ik = find((sk>=tLH(:,1)) & (sk<=tLH(:,2)), 1, 'first');
        h(k) = mr - (mr-1)*(sk-tLH(ik,1))/(tLH(ik,2)-tLH(ik,1));
        continue;
    end
    if iHL(k) % H-L transient
        ik = find((sk>=tHL(:,1)) & (sk<=tHL(:,2)), 1, 'first');
        h(k) = 1 - (1-mr)*(sk-tHL(ik,1))/(tHL(ik,2)-tHL(ik,1));
        continue;
    end
end
hinterp = griddedInterpolant(t, h); % interpolant

% Create mesh
Nmesh = N-1; % num of mesh int
rN = 1; % init
for j=1:5 % Max 5 attempts
    % init
    hmesh = [];
    t0 = t(1); % init
    while true
        h0 = rN*hinterp(t0);
        hmesh(end+1) = h0;
        t0 = t0 + h0;
        if t0>t(end)
            break;
        end
    end
    rN = numel(hmesh)/Nmesh;
    if numel(hmesh)==Nmesh
        break;
    end
end
% fix if numel(hmesh) is different from N
if numel(hmesh)>Nmesh % del last vals
    hmesh = hmesh(1:Nmesh);
elseif numel(hmesh)<Nmesh % rep last val
    hmesh = [hmesh, repmat(hmesh(end), [1 Nmesh-numel(hmesh)])];
end

% Calculate mesh fractions
mesh = hmesh/sum(hmesh);
tmesh = [0, cumsum(mesh)]*(t(end)-t(1)) + t(1);

% control fig
if cf
    figure
    title('Mesh sectors')
    hold on
    plot(t, v, 'LineWidth', 1);
    pltH = xregion(tH(:,1),tH(:,2),FaceColor="#0073FD",DisplayName='Fine mesh');
    pltL = xregion(tL(:,1),tL(:,2),FaceColor="r",DisplayName='Coarse mesh');
    pltLH = xregion(tLH(:,1),tLH(:,2),FaceColor="y",DisplayName='Coarse-to-fine');
    pltHL = xregion(tHL(:,1),tHL(:,2),FaceColor="g",DisplayName='Fine-to-coarse');
    plty = yline(rms(v)*th, 'k-.', 'DisplayName', 'Threshold');
    hold off
    ylabel('v')
    yyaxis right
    ax = gca;
    ax.YAxis(2).Color = 'k';
    hold on
    pltm = plot(tmesh(1:end-1), mesh, 'k-', 'LineWidth',1, 'DisplayName','Mesh fractions');
    hold off
    ylabel('Mesh fractions');
    drawnow;
    legend([pltH(1), pltL(1) pltLH(1) pltHL(1) pltm plty]);
    xlim(t([1 end]))
    box on
    xlabel('t'), 
end

end