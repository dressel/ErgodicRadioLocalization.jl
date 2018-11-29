
mutable struct ErgodicPolicyR2 <: FEBOL.Policy
    execution_horizon::Int      # how long should we execute
    horizon_index::Int          # 

    em::ErgodicManagerR2
    tm::TrajectoryManager

    cache::Array{Float64,3}
    visualize::Bool

    solver::Symbol

    xd::VVF
    ud::VVF

    verbose::Bool
end

function ErgodicPolicyR2(f;
                       K::Int = 5,
                       planning_horizon::Int = 50,
                       execution_horizon::Int = 5,
                       visualize::Bool=true,
                       solver::Symbol=:PTO,
                       verbose::Bool=false,
                       R::Matrix{Float64}=0.01 * eye(2)
                      )

    # ergodic manager
    N = planning_horizon
    d = Domain(f.n)
    em = ErgodicManagerR2(d, K)

    # trajectory manager
    ci = ConstantInitializer([0.0,0.0])
    h = 1.
    x0 = [0.5,0.5]
    tm = TrajectoryManager(x0,h,N,ci)
    tm.barrier_cost = 1.0

    tm.R = R

    cache = make_cache(f)

    return ErgodicPolicyR2(execution_horizon,
                         execution_horizon,  # index should be >= exec hrzn
                         em,
                         tm,
                         cache,
                         visualize,
                         solver,
                         [[0.0]],
                         [[0.0]],
                         verbose
                        )
end

function FEBOL.action(m::SearchDomain, x::Vehicle, o, f::DF, p::ErgodicPolicyR2)

    L = m.length
    if p.horizon_index >= p.execution_horizon

        # generate EID from filter
        p.em.phi = mutual_information(f, p.cache)

        # normalize and decompose
        normalize!(p.em)
        decompose!(p.em)

        # right now, let's just make a new trajectory each time
        p.tm.x0 = [x.x/L, x.y/L]
        if p.solver == :SMC
            p.xd, p.ud = smc_trajectory(p.em, p.tm, umax=5/L)
        else
            tic()
            p.xd, p.ud = pto_trajectory(p.em, p.tm, verbose=p.verbose, max_iters=300)
            t = toq()
            if p.verbose
                println("solve time = ", t)
            end
        end

        p.horizon_index = 0
    end

    p.horizon_index += 1

    a = (p.ud[p.horizon_index][1] * L, p.ud[p.horizon_index][2] * L, 0.0)

    if p.verbose
        println("a = ", a)
    end

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

        # push this into some sort of thing
    end


    return a
end

reset!()
