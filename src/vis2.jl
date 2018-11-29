using PyPlot: 
    imshow,
    xlabel,
    ylabel,
    contour,
    figure,
    pause,
    hold,
    axis,
    title,
    scatter,
    gcf,
    savefig,
    matplotlib,
    rc,
    tick_params,
    clf,
    cla
function vis2(m::SearchDomain, uav::SimUnit;
                   pause_time=0.3,
                   alpha=0.1,
                   show_mean::Bool=false,
                   show_cov::Bool=false,
                   save_gif::Bool=true
                  )

    frames = Frames(MIME("image/png"), fps=5)

    x_frames = Frames(MIME("image/png"), fps=5)
    es_frames = Frames(MIME("image/png"), fps=5)

    #println("rc_context = ", rc_context())
    #rc("font", family="Times New Roman", size=16)
    #rc("text", usetext=true)

    figure("Simulation")
    plot(m, uav.f, uav.x, alpha=alpha)
    #title("i = 0")
    title("\$t\$ = 0 s")
    push!(frames, gcf())

    # What was the cost to getting this first observation?
    cost_sum = get_cost(uav, m)

    # before doing anything else, we observe
    #  and update filter once
    o = observe(m, uav.x)
    update!(uav, o)

    # This was our first step; steps count number of observations
    step_count = 1

    # plot if need be
    plot(m, uav.f, uav.x, alpha=alpha)
    #title("i = $(step_count)")
    title("\$t\$ = $(step_count) s")
    push!(frames, gcf())


    while !is_complete(uav.f, uav.tc, step_count)
        # act
        #tic()
        a = action(m, uav, o)
        figure("Simulation")
        push!(x_frames, gcf())
        figure("MI")
        push!(es_frames, gcf())
        #ta = toq()
        #println("time = ", ta)
        act!(m, uav.x, a)

        move_target!(m)

        # get cost and update step count
        cost_val = get_cost(uav, m, a)
        cost_sum += cost_val
        step_count += 1

        # observe and update
        o = observe(m, uav.x)
        update!(uav, o)

        # plot if need be
        pause(pause_time)
        figure("Simulation")
        cla()
        plot(m, uav.f, uav.x, alpha=alpha, show_mean=show_mean, show_cov=show_cov)

        #max_b = maximum(uav.f.b)
        #title("i = $(step_count), cost = $(round(cost_val,3))")
        #title("\$t\$ = $(step_count) s, cost = $(round(cost_val,3))")
        title("\$t\$ = $(step_count) s")
        push!(frames, gcf())

        # temporary...
        #cm = covariance(uav.f)      # covariance matrix
        #ev = eigvals( covariance(uav.f) )
        #println("ev[1] = ", round(sqrt(ev[1]),3), ", ev[2] = ", round(sqrt(ev[2]),3))
        #num_out = 0
        #parts = uav.f._b.particles
        #for praxar in parts
        #    if praxar[1] > m.length || praxar[1] < 0.0 || praxar[2] > m.length || praxar[2] < 0.0
        #        num_out += 1
        #    end
        #end
        #p_out = round(100.0 * num_out / length(parts), 2)
        #println("percentage of particles out: ", p_out)
        #ce = cheap_entropy(uav.f, m.length, 50)
        #println("\tcheap_entropy = ", round(ce,3))
        #ce2 = cheap_entropy2(uav.f, m.length, 50)
        #println("\tcheap_entropy2 = ", round(ce2,3))


    end
    write("temp.gif", frames)

    figure("Simulation")
    push!(x_frames, gcf())
    figure("MI")
    push!(es_frames, gcf())

    write("temp1.gif", x_frames)
    write("temp2.gif", es_frames)

    return cost_sum
end
