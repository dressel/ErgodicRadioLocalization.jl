# bearing.jl
# adding noise to bearing modality


struct NoisyBearing <: FEBOL.Sensor
    bo::BearingOnly
end

NoisyBearing(ns::Real) = NoisyBearing( BearingOnly(ns) )

function FEBOL.observe(theta::FEBOL.LocTuple, s::NoisyBearing, p::Pose)
    truth = true_bearing(p, theta)
    noise = s.bo.noise_sigma * randn()
    if p[1] < 200.0

        # determine line to radio source
        dx = theta[1] - p[1]        # x-distance to jammer
        dy = theta[2] - p[2]        # y-distance to jammer

        y_at_200 = p[2] + (dy / dx) * (200.0 - p[1])

        # we get the wall...
        if y_at_200 > 150.0

            y1 = p[2]
            y2 = theta[2]
            d = theta[1] - p[1]
            x2 = d / (1.0 + y2/y1)

            r_x = theta[1] - x2         # location of reflection
            truth = true_bearing(p, (r_x,0.0))
            noise = 15.0 * randn()
        end
    end
    return mod(truth + noise, 360.0)
end

# This shouldn't matter, because we only observe
#function O(bo::NoisyBearing, theta::TargetTuple, p::Pose, o)
#
#    # Calculate true bearing, and find distance to bin edges
#    ang_deg = true_bearing(p, theta)
#    o_diff = fit_180(o - ang_deg)
#
#    # now look at probability
#    d = Normal(0, bo.noise_sigma)
#    p = pdf(d, o_diff)
#    return p
#end
