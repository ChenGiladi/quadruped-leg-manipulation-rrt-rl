function optpath = smoothPathWithReverification(rawStates, map, sv, options)
%SMOOTHPATHWITHREVERIFICATION  Path smoothing with a collision re-verification fall-back.
%
%   OPTPATH = SMOOTHPATHWITHREVERIFICATION(RAWSTATES, MAP, SV, OPTIONS) smooths
%   the collision-free RRT polyline RAWSTATES with optimizePath and then applies
%   the cluttered-environment fall-back described in the manuscript
%   (Section 3.2.1, response to comment A.8):
%
%     1. The smoothed path is densely re-sampled along its arc length.
%     2. Every sample is tested against the occupancy MAP through the state
%        validator SV (isStateValid).
%     3. If any sample is in collision the smoothing window is shrunk -- the
%        MaxPathStates budget is reduced and the ObstacleSafetyMargin raised --
%        and the path is refitted with optimizePath.
%     4. Steps 1-3 repeat until the re-sampled path is collision-free or the
%        smoothing window collapses back to the raw RRT polyline RAWSTATES,
%        which is collision-free by construction.
%
%   In the open environments reported in this paper the first smoothed path is
%   already clear, so the function returns after a single validation pass with
%   feasibility preserved by construction.
%
%   Inputs:
%     RAWSTATES - N-by-3 [x y theta] collision-free RRT path (pthObj.States).
%     MAP       - occupancyMap used by optimizePath.
%     SV        - validatorOccupancyMap state validator for the same map.
%     OPTIONS   - optimizePathOptions object.
%
%   This MATLAB routine mirrors the tested Python reference reverify_smoothing()
%   in ../python/rrt_planner.py. It uses only documented Robotics System /
%   Navigation Toolbox calls (optimizePath, isStateValid).

    maxPasses    = 20;
    sampleSpacing = 0.1;                  % re-sample spacing (map units) for the test
    minStates    = size(rawStates, 1);    % collapse target = raw polyline length

    optpath = optimizePath(rawStates, map, options);

    for pass = 1:maxPasses
        dense = denselyResample(optpath, sampleSpacing);
        if all(isStateValid(sv, dense))
            return;                       % collision-free -> accept smoothed path
        end

        % Shrink the smoothing window and harden the safety margin, then refit.
        options.MaxPathStates        = max(minStates, round(options.MaxPathStates * 0.7));
        options.ObstacleSafetyMargin = options.ObstacleSafetyMargin * 1.2;

        if options.MaxPathStates <= minStates
            optpath = rawStates;          % window collapsed -> raw collision-free polyline
            return;
        end
        optpath = optimizePath(rawStates, map, options);
    end

    % Passes exhausted: prefer the guaranteed-safe raw polyline if the last
    % smoothed path still collides.
    if ~all(isStateValid(sv, denselyResample(optpath, sampleSpacing)))
        optpath = rawStates;
    end
end

function dense = denselyResample(states, spacing)
%DENSELYRESAMPLE  Linear arc-length resample of [x y theta] states for testing.
    xy  = states(:, 1:2);
    seg = sqrt(sum(diff(xy, 1, 1).^2, 2));
    cum = [0; cumsum(seg)];
    if cum(end) == 0
        dense = states;
        return;
    end
    n  = max(size(states, 1), ceil(cum(end) / spacing) + 1);
    sq = linspace(0, cum(end), n).';
    dx  = interp1(cum, states(:, 1), sq);
    dy  = interp1(cum, states(:, 2), sq);
    dth = interp1(cum, unwrap(states(:, 3)), sq);
    dense = [dx, dy, dth];
end
