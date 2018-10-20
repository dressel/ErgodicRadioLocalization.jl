
struct ErgodicPolicy <: FEBOL.Policy
    execution_horizon::Int      # how long should we execute

    em::ErgodicManagerR2
    tm::TrajectoryManager
end

function ErgodicPolicy()

    # ergodic manager
    N = N
    d = Domain(21)
    em = ErgodicManagerR2(d, K)

    # trajectory manager
    ci = ConstantInitializer([0.0,0.0])
    h = 1.
    N = 30
    x0 = [0.5,0.5]
    tm = TrajectoryManager(x0,h,N,ci)
    tm.barrier_cost = 1.0

    return ErgodicPolicy(5, em, tm)
end

function FEBOL.action(m::SearchDomain, x::Vehicle, o, f::AbstractFilter, p::ErgodicPolicy)

    if p.steps_remaining == 0
        # recalculate trajectory
    end

    return a
end
