#!/usr/bin/env julia167 
# Change to 'julia' instead of 'julia167' if you don't have a symlink setup

using RobotOS
using iLQGameSolver
using SparseArrays
using LinearAlgebra


@rosimport geometry_msgs.msg: Twist, Pose
@rosimport gazebo_msgs.srv.GetModelState ## get the initial state
## Use the service or just use a ros subscirber for GetModelState
## or Odom topic 
rostypegen()
using .geometry_msgs.msg: Twist, Pose

function solvegame()
    # Setup the problem
    dt = 0.1                    # Step size [s]
    H = 40.0                    # Horizon [s]
    k_steps = Int(H/dt)         # Number of steps (knot points)
    println(k_steps)

    # Initial and final states
    # x₁, y₁, θ₁     

    x₀= [5.0; 0.0; pi/2; 0.0; 0.0; 5.0; 0.0; 0.0]        # Initial state
    xgoal = [5.0; 10.0; pi/2; 0.0; 10.0; 5.0; 0.0; 0.0] # Final state

    # Define cost matrices 
    Q1 = sparse(zeros(8,8))     # State cost for agent 1
    Q1[1:4,1:4] = 1.0*I(4); Q1[3,3] = 6.0; #Q1[4,4] = 6.0
    Qn1 = 10*Q1                    # Terminal cost for agent 1

    Q2 = sparse(zeros(8,8))     # State cost for agent 2
    Q2[5:8,5:8] = 1.0*I(4); Q2[7,7] = 6.0; #Q2[8,8] = 5.0;
    Qn2 = 10*Q2                    # Terminal cost for agent 2

    R11 = 6.0*I(2)              # Control cost for player 1
    R22 = 6.0*I(2)              # Contorl cost for player 2
    R12 = sparse(zeros(2,2))    # Control cost for player 1 associated with player 2's controls
    R21 = sparse(zeros(2,2))    # Control cost for player 2 associated with player 1's controls

    dmax = 0.5                  # Distance that both agents should keep between each other [m]
    ρ = 500.0                   # Penalty factor for violating the distance constraint

    # Input constraints
    umax = [0.1, 0.6, 0.1, 0.6]            
    umin = [-1.0, -0.6, -1.0, -0.6]

    u1goal = [0.0, 0.0]
    u2goal = [0.0, 0.0]

    xₜ, uₜ = iLQGameSolver.solveILQGame(iLQGameSolver.diff_drive, iLQGameSolver.costPointMass,
                                         x₀, xgoal, u1goal, u2goal, Q1, Q2, Qn1, Qn2, R11, R12, 
                                         R21, R22, umin, umax, dmax, ρ, dt, H)
    
    return xₜ, uₜ
end

function commands(pub0, pub1)
    agent1 = Twist()
    agent2 = Twist()

    xₜ, uₜ = solvegame()

    rate = Rate(10) # 10 Hz
    for i = 1:size(uₜ)[1]
        agent1.linear.x = xₜ[i,4]#*cos(rad2deg(xₜ[i,3]))
        #agent1.linear.y = uₜ[i,1]*sin(rad2deg(xₜ[i,3]))
        agent1.angular.z = uₜ[i,2]


        agent2.linear.x = xₜ[i,8]#*cos(rad2deg(xₜ[i,6]))
        #agent2.linear.y = uₜ[i,3]*sin(rad2deg(xₜ[i,6]))
        agent2.angular.z = uₜ[i,4]

        # loginfo(hello_str)
        publish(pub1, agent1)
        publish(pub0, agent2)
        rossleep(rate)
    end
end

function main()
    init_node("turtlebot3_cmd") # node name
    pub0 = Publisher{Twist}("/tb3_0/cmd_vel", queue_size=10) # topic name
    pub1 = Publisher{Twist}("/tb3_1/cmd_vel", queue_size=10) # topic name

    commands(pub0, pub1)
end

if !isinteractive()
    main()
end
