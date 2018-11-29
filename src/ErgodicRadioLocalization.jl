module ErgodicRadioLocalization

using FEBOL
using ErgodicControl
using ErgodicControlPlots
using Reel

export ErgodicPolicyR2
export ErgodicPolicySE2
export vis2

export NoisyBearing

include("r2.jl")
include("se2.jl")
include("bearing.jl")
include("df2.jl")
include("vis2.jl")

end # module
