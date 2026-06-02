classdef plannerRRTStar_t < plannerRRT
    %PLANNERRRTSTAR  Geometric optimal RRT (RRT*) path planner
    %   plannerRRTStar creates an asymptotically-optimal RRT planner. RRT* 
    %   algorithm is derived from the RRT algorithm and it is guaranteed
    %   to converge to an optimal solution, meanwhile, its runtime is 
    %   guaranteed to be a constant factor of the runtime of RRT algorithm. 
    %   The optimality here is defined with respect to the distance function
    %   of the state space. It should be noted that RRT* algorithm can only
    %   be used to solve geometric planning problems, in other words, it
    %   requires that any two random states drawn from the state space
    %   can be connected.
    %
    %   plannerRRTStar(STATESPACE, STATEVAL) creates an RRT* planner from a 
    %   state space object, STATESPACE, and a state validator object,
    %   STATEVAL. STATEVAL's state space must be the same as STATESPACE.
    %   
    %   planner = plannerRRTStar(___,Name=Value) sets properties using one or 
    %   more name-value arguments in addition to the input arguments 
    %   in the previous syntax.
    %
    %   plannerRRTStar properties:
    %      BallRadiusConstant       - Constant used to compute ball radius for finding near neighbors 
    %      ContinueAfterGoalReached - Whether to continue optimizing path after goal is reached
    %      StateSpace               - State space for the planner
    %      StateValidator           - State validator for the planner
    %      MaxNumTreeNodes          - Max number of tree nodes
    %      MaxIterations            - Max number of iterations  
    %      MaxConnectionDistance    - Max connection distance between two states
    %      GoalReachedFcn           - Callback for determine if goal is reached
    %      GoalBias                 - Probability of choosing goal state during state sampling
    %
    %   plannerRRTStar methods:
    %      plan    - Plan a path between two states
    %      copy    - Create deep copy of the planner object
    %
    %   Example:
    %      % Create a state space
    %      ss = stateSpaceSE2;
    %
    %      % Create a occupancyMap-based state validator using the state space just created
    %      sv = validatorOccupancyMap(ss);
    %
    %      % Update the map for state validator, set map resolution 10 cells/meter
    %      load exampleMaps
    %      map = occupancyMap(simpleMap, 10);
    %      sv.Map = map;
    %
    %      % Set validation distance for the validator
    %      sv.ValidationDistance = 0.01;
    %
    %      % Update state space bounds to be the same as map limits
    %      ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    %
    %      % Create RRT* path planner and configure MaxIterations and 
    %      % MaxConnectionDistance properties
    %      planner = plannerRRTStar(ss, sv, MaxIterations=2500, MaxConnectionDistance=0.3);
    %
    %      % Allow further optimization
    %      planner.ContinueAfterGoalReached = true;
    %
    %      % Set the start and goal states
    %      start = [0.5, 0.5 0];
    %      goal = [2.5, 0.2, 0];
    %
    %      % Plan a path with default settings, set rng for repetitive result
    %      rng(100, 'twister')
    %      [pthObj, solnInfo] = plan(planner, start, goal)
    %
    %      % Show the results
    %      map.show;
    %      hold on;
    %      plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-'); % tree expansion
    %      plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2) % draw path
    %     
    %
    %   References: 
    %   [1] S. Karaman, E. Frazzoli, "Sampling-based algorithms for optimal motion planning",
    %       International Journal of Robotics Research, vol. 30, no. 7,
    %       pp. 846-894, Jun 2011
    %
    %   See also plannerRRT, plannerBiRRT.
    
    %   Copyright 2019-2022 The MathWorks, Inc.
    
    %#codegen

    properties (Constant, Access = {?nav.algs.internal.InternalAccess})
        %ClassName
        ClassName = 'plannerRRTStar_t'
    end
    
    properties
        %BallRadiusConstant Constant used to estimate the near neighbors search radius
        %   The radius is estimated as:
        %   
        %       r =  min( (gamma*log(n)/n))^(1/d), ita), where
        %
        %           gamma   :   BallRadiusConstant
        %           ita     :   MaxConnectionDistance
        %           d       :   Dimension of the state space
        %           n       :   Current number of nodes in the tree
        %   
        %   gamma is defined as:
        %
        %       gamma = 2^d*(1+1/d)*(vFree/vBall), where
        %
        %           vFree   :   Approximate free volume in search-space
        %           vBall   :   Volume of unit ball in d dimensions
        %      
        %   The formulae above define a BallRadiusConstant of "appropriate"
        %   size for a given space, meaning as the number of nodes filling
        %   the space grows and the radius shrinks, the expected number of
        %   neighbors grows logarithmically. Higher values will result in a 
        %   higher average number of neighbors within the d-ball per 
        %   iteration, leading to more rewire candidates, but values below 
        %   this suggested minimum could lead to a single nearby neighbor, 
        %   which fails to produce asymptotically optimal results.
        %
        %   Default: 100
        BallRadiusConstant
        
        %ContinueAfterGoalReached Continue to optimize path after goal is reached
        %   Planning will stop after the max number of iterations or max
        %   number of tree nodes is reached.
        %
        %   Default: false
        ContinueAfterGoalReached
    end
    properties (Access = protected)
        %PathCostAtIteration Path cost for each iteration for the nodes. 
        % Consist of the best cost for the each iteration, this is
        % needed in case where multiple iterations are done to find most
        % suitable path.
        PathCostAtIteration
    end
    
    methods
        function obj = plannerRRTStar_t(ss, sv, varargin)
            %PLANNERRRTSTAR Constructor
            narginchk(2,16);

            nvPairs = plannerRRTStar_t.parseInputs(varargin{:});
            obj@plannerRRT(ss, sv);

            obj.MaxNumTreeNodes = nvPairs.MaxNumTreeNodes;
            obj.MaxIterations = nvPairs.MaxIterations;
            obj.MaxConnectionDistance = nvPairs.MaxConnectionDistance;
            obj.GoalBias = nvPairs.GoalBias;
            obj.BallRadiusConstant = nvPairs.BallRadiusConstant;
            obj.ContinueAfterGoalReached = nvPairs.ContinueAfterGoalReached;
           
            obj.PathCostAtIteration = nan(obj.MaxIterations, 1);

            % codegen does not allow redefinition of function handles once
            % initialized. Hence for codegen workflow initializing
            % GoalReachedFcn in 'plan' function if the user does not
            % provide custom GoalReachedFcn via NV Pairs.
            if (isa(nvPairs.GoalReachedFcn,"function_handle") && strcmp( func2str(nvPairs.GoalReachedFcn),'undefined'))       
                if coder.target('MATLAB')
                    obj.GoalReachedFcn = @nav.algs.checkIfGoalIsReached;
                end
            else
                obj.GoalReachedFcn = nvPairs.GoalReachedFcn;
            end
            
            
        end
        
        function [pathObj, solnInfo] = plan(obj, startState, goalState)
            %plan Plan a path between two states
            %   PATH = plan(PLANNER, STARTSTATE, GOALSTATE) tries to find  
            %   a valid path between STARTSTATE and GOALSTATE. The planning
            %   is carried out based on the underlying state space and state
            %   validator of PLANNER. The output, PATH, is returned as a 
            %   navPath object.
            %
            %   [PATH, SOLNINFO] = plan(PLANNER, ...) also returns a struct,
            %   SOLNINFO, as a second output that gives additional    
            %   details regarding the planning solution.
            %   SOLNINFO has the following fields:
            %
            %   IsPathFound:  Boolean indicating whether a path is found
            %
            %      ExitFlag:  A number indicating why planner terminates
            %                 1 - 'GoalReached'
            %                 2 - 'MaxIterationsReached'
            %                 3 - 'MaxNumNodesReached'
            %
            %      NumNodes:  Number of nodes in the search tree when
            %                 planner terminates (not counting the root
            %                 node).
            %
            % NumIterations:  Number of "extend" routines executed
            %
            %      TreeData:  A collection of explored states that reflects
            %                 the status of the search tree when planner
            %                 terminates. Note that nan values are inserted
            %                 as delimiters to separate each individual
            %                 edge.
            %
            %     PathCosts:  Contains the Cost of the path at each
            %                 iteration. Value for iterations when path has
            %                 not reached the goal is denoted by a NaN.
            %                 Size of the array is NumIterations-by-1. Last
            %                 element contains the cost of the final path.
                                                          
            % Wipe the slate clean for the this execution
            obj.PathCostAtIteration = nan(obj.MaxIterations, 1);
            
            [pathObj, solnInfo] = plan@plannerRRT(obj, ...
                startState, goalState);
            
            % Default value in case num iterations are zero. It also allows
            % for PathCosts to  be added to solnInfo without reading it,
            % helps with codegen.
            defaultVal = 0;
            coder.varsize("defaultVal");
            solnInfo.PathCosts = defaultVal;
            if solnInfo.NumIterations
                % Update Path costs if extend function was called.
                solnInfo.PathCosts = ...
                    obj.PathCostAtIteration(1:solnInfo.NumIterations);
            end
        end

        function objCopied = copy(obj)
            %COPY Create a deep copy of the plannerRRTStar object
            %   PLANNERCOPIED = COPY(PLANNER) creates a deep copy of the
            %   existing plannerRRTStar object, PLANNER, and returns
            %   the new object in PLANNERCOPIED.
            
            validator = obj.StateValidator.copy;
            
            objCopied = plannerRRTStar_t(validator.StateSpace, validator);
            obj.copyProperties(objCopied);
        end
    end
    
    methods (Access = {?nav.algs.internal.InternalAccess})
        
        function preLoopSetup(obj, treeInternal)
            %preLoopSetup
            treeInternal.setBallRadiusConstant(obj.BallRadiusConstant);
            treeInternal.setMaxConnectionDistance(obj.MaxConnectionDistance);
            
        end
        
        function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
            %extend The optimal RRT "Extend" routine

            statusCode = obj.InProgress;
            newNodeId = nan;
            
            idNN = treeInternal.nearestNeighbor(randState);
            stateNN = treeInternal.getNodeState(idNN);
            costNN = treeInternal.getNodeCostFromRoot(idNN);
            
            d = obj.StateSpace.distance(stateNN, randState);
            if d < 1e-10 % avoid adding the same state multiple times
                statusCode = obj.ExistingState;
                return;
            end
            stateNew = randState;
            
            % steer
            if d > obj.MaxConnectionDistance
                stateNew = obj.StateSpace.interpolate(stateNN, randState, obj.MaxConnectionDistance/d);  % L/d*(randState - nnState) + nnState;
                d = obj.MaxConnectionDistance;
            end
            costNew = costNN + d+2;

            % check motion validity
            if ~obj.StateValidator.isMotionValid(stateNN, stateNew)
                statusCode = obj.MotionInCollision;
                return;
            else
                nearIndices = treeInternal.near(stateNew);

                % look around at the near neighbors and insert stateNew
                % under the one that has lowest cost
                costMin = costNew+2;
                idMin = -1;
                
                for j = 1:length(nearIndices)
                    idNear = nearIndices(j);
                    stateNear = treeInternal.getNodeState(idNear);
                    costNear = treeInternal.getNodeCostFromRoot(idNear);

                    costNewTenative = costNear + obj.StateSpace.distance(stateNear, stateNew);

                    if costMin > costNewTenative ...
                        && obj.StateValidator.isMotionValid(stateNear, stateNew)
                        
                        costMin = costNewTenative;
                        idMin = idNear;
                    end
                end
                
                if idMin >= 0
                    idNew = treeInternal.insertNode(stateNew, idMin);
                else
                    idNew = treeInternal.insertNode(stateNew, idNN);
                end
                newNodeId = idNew;

                
                % now see if it's possible to rewire any of the near
                % neighbors under the newly added node
                if idMin >= 0
                    nearIndices = nearIndices(nearIndices~=idMin);
                end

                for k = 1:length(nearIndices)
                    idNear = nearIndices(k);
                    stateNear = treeInternal.getNodeState(idNear);
                    costNear = treeInternal.getNodeCostFromRoot(idNear);
                    if costNear > costNew + obj.StateSpace.distance(stateNear, stateNew) && ...
                        obj.StateValidator.isMotionValid(stateNear, stateNew)
                        rewireStatus = treeInternal.rewire(idNear, idNew); %#ok<NASGU>
                        %fprintf('attempted to rewire node %d under node %d, result: [%f]\n', int32(idNear), int32(idNew), rewireStatus);
                    end
                end
                
            end
            
            if newNodeId == obj.MaxNumTreeNodes
                statusCode = obj.MaxNumTreeNodesReached;
                return
            end
            
            if obj.GoalReachedFcn(obj, stateNew, obj.CurrentGoalState)
                if obj.ContinueAfterGoalReached
                    statusCode = obj.GoalReachedButContinue;
                else
                    statusCode = obj.GoalReached;
                end
                return;
            end
        end

    end
    
    methods(Access = protected)
        function cname = getClassName(obj) %#ok<MANU>
            %getClassName
            cname = 'plannerRRTStar_t';
        end
        
        function copyProperties(obj, copyObj)
            %copyProperties Copy property data from this object to a new object

            % Copy all properties of the base RRT class
            copyProperties@plannerRRT(obj, copyObj);

            % Copy plannerRRTStar specific properties
            copyObj.BallRadiusConstant = obj.BallRadiusConstant;
            copyObj.ContinueAfterGoalReached = obj.ContinueAfterGoalReached;
        end
        
        function collectPathCosts(obj, tree, nodeIds, iterationNum)
            %collectPathCosts calculates min cost in tree for IDs and save it.
            % 
            % collectPathCosts(OBJ, TREE, ID, iterationNum) computes the
            % min cost amongst the node Ids specified by ID in the TREE.
            % After computing the min cost it assigns it to
            % PathCostAtIteration property at the appropriate index.

            costs = zeros(size(nodeIds));
            for i = 1:numel(nodeIds)
                costs(i) = tree.getNodeCostFromRoot(nodeIds(i));
            end

            % When tentativeGoalIds is empty expression becomes min([nan]),
            % which returns nan, as expected for the "not reached goal"
            % case.
            % When tentativeGoalIds is not empty, the expression takes the
            % form min([nan a b ...]) which ignores nan and assigns the min
            % cost from root amongst the tentative goals as the iteration
            % cost.
            minCost = min([nan, costs]);
            assignPathCostForIteration(obj, minCost, iterationNum);
        end

        function assignPathCostForIteration(obj, cost, iterationNum)
            %assignPathCostForIteration assigns cost to the element
            %corresponding to iterationNum in PathCostAtIteration property.

            obj.PathCostAtIteration(iterationNum) = cost;
        end
    end
    
    % setters
    methods
        function set.BallRadiusConstant(obj, radiusConstant)
            %set.BallRadiusConstant
            robotics.internal.validation.validatePositiveNumericScalar(radiusConstant, getClassName(obj), 'BallRadiusConstant');
            obj.BallRadiusConstant = radiusConstant;
        end
        
        function set.ContinueAfterGoalReached(obj, contFlag)
            %set.ContinueAfterGoalReached
            robotics.internal.validation.validateLogical(contFlag, getClassName(obj), 'ContinueAfterGoalReached');
            obj.ContinueAfterGoalReached = logical(contFlag);
        end
    end
        methods (Hidden, Static)
        
        function nvPairs = parseInputs(varargin)
            % Retrieve default properties
            defaults = plannerRRTStar_t.propertyDefaults;
            
            % Parse inputs and return NV-pairs
            pstruct = coder.internal.parseParameterInputs(defaults,struct(),varargin{:});
            nvPairs = coder.internal.vararginToStruct(pstruct,defaults,varargin{:});
        end
        
        function defaults = propertyDefaults()
            defaults = struct(...
            'MaxNumTreeNodes',1e4,...
            'MaxIterations',1e4,...
            'MaxConnectionDistance',0.1,...
            'GoalBias', 0.05,...
            'GoalReachedFcn',@undefined, ... %Setting a random value. Default value will be set in constructor          
            'BallRadiusConstant', 100,...
            'ContinueAfterGoalReached',false);

        end
    end
end

