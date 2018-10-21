
mutable struct ErgodicPolicySE2 <: FEBOL.Policy
    execution_horizon::Int      # how long should we execute
    horizon_index::Int          # 

    em::ErgodicManagerSE2
    tm::TrajectoryManager

    cache::Array{Float64,4}
    visualize::Bool

    solver::Symbol

    xd::VVF
    ud::VVF
end

function ErgodicPolicySE2(f::DF;
                       K::Int = 5,
                       planning_horizon::Int = 50,
                       execution_horizon::Int = 5,
                       visualize::Bool=true,
                       solver::Symbol=:PTO
                      )

    # ergodic manager
    N = planning_horizon
    d = Domain(f.n)
    em = ErgodicManagerSE2(d, K)

    # trajectory manager
    ci = ConstantInitializer([0.0,0.0])
    h = 1.
    x0 = [0.5,0.5]
    tm = TrajectoryManager(x0,h,N,ci)
    tm.barrier_cost = 1.0

    println("tm.R = ", tm.R)
    tm.R = 0.2*eye(2)

    cache = make_cache2(f)

    return ErgodicPolicySE2(execution_horizon,
                         9999999,       # index should just start very large
                         em,
                         tm,
                         cache,
                         visualize,
                         solver,
                         [[0.0]],
                         [[0.0]]
                        )
end

function FEBOL.action(m::SearchDomain, x::Vehicle, o, f::DF, p::ErgodicPolicySE2)

    
    L = m.length
    if p.horizon_index >= p.execution_horizon

        # generate EID from filter
        tic()
        p.em.phi = mutual_information(f, p.cache)
        t = toq()
        println("time to eid = ", t)

        # normalize and decompose
        tic()
        normalize!(p.em)
        t = toq()
        println("time to normalize = ", t)
        tic()
        decompose!(p.em)
        t = toq()
        println("time to decompose = ", t)

        # right now, let's just make a new trajectory each time
        p.tm.x0 = [x.x/L, x.y/L]
        tic()
        p.xd, p.ud = pto_trajectory(p.em, p.tm)
        t = toq()
        println("solve time = ", t)

        p.horizon_index = 0
    end

    p.horizon_index += 1

    if p.visualize
        xd2 = deepcopy(p.xd)
        for xi in xd2
            xi[1] *= L
            xi[2] *= L
        end
        figure("Simulation")
        plot_trajectory(xd2)

        figure("MI")
        ErgodicControlPlots.cla()
        plot(p.em, p.xd)
    end



    a = (p.ud[p.horizon_index][1] * L, p.ud[p.horizon_index][2] * L, 0.0)
    println("a = ", a)
    return a
end
