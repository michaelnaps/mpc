
const defFloat = Float64
const defInt = Int64
const Δt = defFloat( 1e-1 )

using Plots
using DelimitedFiles
using Measures
using LaTeXStrings

using StatsBase
using LinearAlgebra

# Define a custom color palette and figure defaults.
colorlist = [:indianred, :cornflowerblue, :darkorchid, :olivedrab, :darkgoldenrod]
default(
    guidefont=font(10, "Computer Modern"),  # axis labels
    tickfont=font(10, "Computer Modern"),   # tick labels
    legendfont=font(10, "Computer Modern"), # legend
    titlefont=font(10, "Computer Modern"),  # title
    palette=colorlist,
    xformatter=:plain, dpi=600
)

function saveplot(plt, filename::String;
    background::Symbol=:transparent, legend_background_color::Symbol=:white, args...)
    plot!( plt; background=background, legend_background_color=legend_background_color, args... )
    savefig( plt,  filename )
    plot!( plt; background=:white )
    return plt
end;
