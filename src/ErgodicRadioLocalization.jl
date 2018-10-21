module ErgodicRadioLocalization

using FEBOL
using ErgodicControl
using ErgodicControlPlots

export ErgodicPolicyR2
export ErgodicPolicySE2

include("r2.jl")
include("se2.jl")

end # module
